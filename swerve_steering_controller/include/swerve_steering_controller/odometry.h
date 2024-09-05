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

#pragma once

#include <swerve_steering_controller/utils.h>
#include <numeric>
#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

namespace swerve_steering_controller
{
    namespace bacc = boost::accumulators;

    class Odometry
    {
        public:

        /// Integration function, used to integrate the odometry:
        typedef boost::function<void(double, double,double, double)> IntegrationFunction;

        Odometry(size_t velocity_rolling_window_size = 10);

        void init(const ros::Time &time, double infinity_tolerance, double intersection_tolerance);

        bool update(std::vector<double> wheels_omega, std::vector<double> holders_theta, std::vector<int> directions, const ros::Time &time, std::array<double,2>* intersection_point);

        double getHeading() const
        {
            return heading_;
        }

        double getX() const
        {
            return x_;
        }

        double getY() const
        {
            return y_;
        }

        double getLinearX() const
        {
            return linear_x_;
        }

        double getLinearY() const
        {
            return linear_y_;
        }

        double getAngular() const
        {
            return angular_;
        }

        void setWheelsParams(std::vector<double> radii, std::vector<std::array<double,2>> positions);

        void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

        private:

        /// Rolling mean accumulator and window:
        typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
        typedef bacc::tag::rolling_window RollingWindow;

        void integrateRungeKutta2(double linear_x, double linear_y, double angular, const double dt);

        void integrateExact(double linear_x, double linear_y, double angular, const double dt);

        void resetAccumulators();

        /// Current timestamp:
        ros::Time timestamp_;

        size_t wheels_num_;

        /// Current pose:
        double x_;        //   [m]
        double y_;        //   [m]
        double heading_;  // [rad]

        /// Current velocity:
        double linear_x_;  //   [m/s]
        double linear_y_;  //   [m/s]
        double angular_; // [rad/s]

        double intersection_tol_;
        double inf_tol_;

        /// Wheel kinematic parameters [m]:
        std::vector<double> wheels_radii_;
        std::vector<std::array<double,2>> wheels_positions_;

        /// Previou wheels and holders position/state [rad]:
        std::vector<double> wheels_old_pomega_;
        std::vector<double> holders_old_theta_;

        /// Rolling mean accumulators for the linar and angular velocities:
        size_t velocity_rolling_window_size_;
        RollingMeanAcc linear_acc_x_;
        RollingMeanAcc linear_acc_y_;
        RollingMeanAcc angular_acc_;

        /// Integration funcion, used to integrate the odometry:
        IntegrationFunction integrate_fun_;
    };
}