/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Mark Naeem
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * The names of the contributors may NOT be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Mark Naeem
 */

#include <swerve_steering_controller/odometry.h>

#include <boost/bind.hpp>

namespace swerve_steering_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_x_(0.0)
  , linear_y_(0.0)
  , angular_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_acc_x_(RollingWindow::window_size = velocity_rolling_window_size)
  , linear_acc_y_(RollingWindow::window_size = velocity_rolling_window_size)
  , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , integrate_fun_(boost::bind(&Odometry::integrateRungeKutta2, this, _1, _2, _3, _4))
  {
  }

  void Odometry::init(const ros::Time& time, double infinity_tolerance, double intersection_tolerance)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    inf_tol_ = infinity_tolerance;
    intersection_tol_ = intersection_tolerance;
    timestamp_ = time;
  }

  bool Odometry::update(std::vector<double> wheels_omega, std::vector<double> holders_theta, std::vector<int> directions, const ros::Time &time, std::array<double,2>* intersection_point)
  {
    for (size_t i=0; i<wheels_num_; ++i)
    {
      wheels_omega[i] = fabs(wheels_omega[i]);
      if (directions[i]<0) // i think this will make a problem when actual omega is too +ve big and the command omega is -ve 
      {                    // as it will take time to reverse and signal will be wrong all this
            holders_theta[i] = utils::theta_map(holders_theta[i]+M_PI);
      }
    }

    //intersection point 
    double theta1,m1,b1,theta,m,b;
    std::vector<std::array<double,2>> intersections;
    for (size_t j=0; j<wheels_num_-1; ++j)
    {
      theta1 = utils::theta_map(holders_theta[j]+M_PI_2);
      m1 = tan(theta1);
      b1 = wheels_positions_[j][1]- m1 * wheels_positions_[j][0];
      for (size_t i=j+1; i<wheels_num_; ++i)
      {
        theta = utils::theta_map(holders_theta[i]+M_PI_2);
        m = tan(theta);
        b = wheels_positions_[i][1]- m * wheels_positions_[i][0];

        if ( utils::isclose(theta1,theta) || utils::isclose(theta1,theta,0.00001,2*M_PI) ) 
        {
          intersections.push_back({INFINITY,INFINITY});
        }
        else if ( utils::isclose(theta1,theta,0.00001,M_PI) || utils::isclose(theta1,theta,0.00001,-M_PI) ) 
        {
          intersections.push_back({(wheels_positions_[j][0]+wheels_positions_[i][0])/2 , (wheels_positions_[j][1]+wheels_positions_[i][1])/2});
        }
        else
        {
          if (fabs(((b1-b)/(m-m1)))>inf_tol_ || fabs(((m*b1-m1*b)/(m-m1)))>inf_tol_ )
          {
            intersections.push_back({INFINITY,INFINITY});
          }
          else
          {
            intersections.push_back({ ((b1-b)/(m-m1)) ,((m*b1-m1*b)/(m-m1)) });
          }
        }
      }
    }


    double linear_x,linear_x_vh,linear_y,linear_y_vh,angular;
    std::array<double,2> average_intersection = {0,0} ;

    //detecting if all or some of the intersections is inf
    bool inf_all = true;
    for (const auto& it: intersections)
    {
      if (! (std::isinf(it[0]) || std::isinf(it[1])))
      {
        inf_all = false;
        break;
      }
    }

    if (inf_all)
    {
      // ROS_INFO_STREAM("inf_all");
      angular=0;
      for (int i=0; i<wheels_num_; ++i)
      {
        linear_x_vh += ( wheels_omega[i]*wheels_radii_[i]*cos(holders_theta[i]) ) / wheels_num_;
        linear_y_vh += ( wheels_omega[i]*wheels_radii_[i]*sin(holders_theta[i]) ) / wheels_num_;
        intersection_point->at(0) = INFINITY; //just to visualize it on rqt_plot through the publisher
        intersection_point->at(1) = INFINITY; //just to visualize it on rqt_plot through the publisher
      }
    }
    else
    {
      for (size_t i=0; i<intersections.size(); ++i)
      {
        if((fabs(intersections[i][0]-intersections[i-1][0])>intersection_tol_ || fabs(intersections[i][1]-intersections[i-1][1])>intersection_tol_) && i!=0)
        {
          // ROS_ERROR_STREAM("intersections are not close enough to get an average, dropping!");
          for (size_t i=0; i<wheels_num_; ++i)
          {
            // ROS_WARN_STREAM("theta, omega: "<<holders_theta[i]<<" "<<wheels_omega[i]);
          }
          for (size_t i=0; i<intersections.size(); ++i)
          {
            // ROS_WARN_STREAM("intersection i:"<<i<<" , "<<intersections[i][0]<<"  "<<intersections[i][1]);
          }
          return false;
        }
        else
        {
            average_intersection[0] += intersections[i][0] / intersections.size();
            average_intersection[1] += intersections[i][1] / intersections.size();
        }
      }
      // ROS_INFO_STREAM("average intersection: "<<average_intersection[0]<<" "<<average_intersection[1]);
      intersection_point->at(0) = average_intersection[0]; //just to visualize it on rqt_plot through the publisher
      intersection_point->at(1) = average_intersection[1]; //just to visualize it on rqt_plot through the publisher
      for (int i=0; i<wheels_num_; ++i)
      {
        //ignore the wheel if the intersection is on its center of rotation
        if (utils::isclose(average_intersection[0],wheels_positions_[i][0])&&utils::isclose(average_intersection[1],wheels_positions_[i][1]))
        {
          continue;
        }
        auto icr_wh = std::array<double,2>{wheels_positions_[i][0]-average_intersection[0] , wheels_positions_[i][1]-average_intersection[1]};
        if (isinf(icr_wh[0])||isinf(icr_wh[1]))
            ROS_WARN_STREAM("icr_wh is inf");
        if (utils::isclose(icr_wh[0],0)||utils::isclose(icr_wh[1],0))
            ROS_WARN_STREAM("icr_wh for wheel "<< i <<" is zero. icr is just over it!");

        angular += (wheels_omega[i]*wheels_radii_[i]*sin(holders_theta[i]))/(2*icr_wh[0]) - (wheels_omega[i]*wheels_radii_[i]*cos(holders_theta[i]))/(2*icr_wh[1]);
        // if (isinf(angular)) ROS_WARN_STREAM("angular is the problem");
      }
      angular/=wheels_num_; 
      linear_x_vh =      average_intersection[1] * angular;
      linear_y_vh = -1 * average_intersection[0] * angular;
    }

    if(isnan(linear_x_vh)||isnan(linear_y_vh)||isnan(angular))
    {
      ROS_ERROR_STREAM("estimated vx,vy or wz is nan");
      for (const auto& it: holders_theta)
      {
        ROS_INFO_STREAM("theta "<<it);
      }
      for (const auto& it: wheels_omega)
      {
        ROS_INFO_STREAM("omega "<<it);
      }
      for (const auto& it: intersections)
      {
        ROS_INFO_STREAM("intersection "<<it[0]<<" "<<it[1]);
      }
      ROS_INFO_STREAM("average intersection: "<<average_intersection[0]<<" "<<average_intersection[1]);
      ROS_INFO_STREAM("linearx "<<linear_x_vh<<" lineary "<<linear_y_vh<<" angular "<<angular);
      return false;
    }
    if(isinf(linear_x_vh)||isinf(linear_y_vh)||isinf(angular))
    {
      ROS_ERROR_STREAM("estimated vx,vy or wz is inf");
      for (const auto& it: holders_theta)
      {
        ROS_INFO_STREAM("theta "<<it);
      }
      for (const auto& it: wheels_omega)
      {
        ROS_INFO_STREAM("omega "<<it);
      }
      for (const auto& it: intersections)
      {
        ROS_INFO_STREAM("intersection "<<it[0]<<" "<<it[1]);
      }
      ROS_INFO_STREAM("average intersection: "<<average_intersection[0]<<" "<<average_intersection[1]);
      ROS_INFO_STREAM("linearx "<<linear_x_vh<<" lineary "<<linear_y_vh<<" angular "<<angular);
      return false;
    }

    // ROS_INFO_STREAM("linearx "<<linear_x_vh<<" lineary "<<linear_y_vh<<" angular "<<angular);

    linear_x = linear_x_vh * cos(heading_) - linear_y_vh * sin(heading_);
    linear_y = linear_x_vh * sin(heading_) + linear_y_vh * cos(heading_);

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Integrate odometry:
    integrate_fun_(linear_x_,linear_y_, angular_,dt);


    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_x_ (linear_x);
    linear_acc_y_ (linear_y);
    angular_acc_  (angular);

    linear_x_ = bacc::rolling_mean(linear_acc_x_);
    linear_y_ = bacc::rolling_mean(linear_acc_y_);
    angular_ = bacc::rolling_mean(angular_acc_);
    
    // ROS_INFO_STREAM(std::endl);
    return true;
  }

  void Odometry::setWheelsParams(std::vector<double> radii, std::vector<std::array<double,2>> positions) 
  {
    this->wheels_radii_ =  std::move(radii);
    this->wheels_positions_ = std::move(positions);
    wheels_num_ = wheels_radii_.size();
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;
    resetAccumulators();
  }

  void Odometry::integrateRungeKutta2(double linear_x, double linear_y, double angular, const double dt)
  {
    const double direction = heading_ + angular;

    /// Runge-Kutta 2nd order integration:
    x_       += ( linear_x * cos(direction) - linear_y * sin(direction) ) * dt;
    y_       += ( linear_x * sin(direction) + linear_y * cos(direction) ) * dt;
    heading_ += angular * dt;
  }

  void Odometry::integrateExact(double linear_x, double linear_y, double angular, const double dt)
  {
    /*Not done correctly*/
    /*NOT USED*/
    if (fabs(angular) < 1e-6)
      integrateRungeKutta2(linear_x, linear_y, angular,dt);
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double rx = linear_x/angular;
      const double ry = linear_y/angular;
      heading_ += angular;
      x_       +=  rx * (sin(heading_) - sin(heading_old));
      y_       += -rx * (cos(heading_) - cos(heading_old));
    }
  }

  void Odometry::resetAccumulators()
  {
    linear_acc_x_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    linear_acc_y_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_  = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }

} // namespace swerve_steering_controller
