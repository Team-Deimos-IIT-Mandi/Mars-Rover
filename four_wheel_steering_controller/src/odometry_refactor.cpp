/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Irstea
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of Irstea nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <four_wheel_steering_controller/odometry.h>
#include <boost/bind.hpp>
#include <cmath>
#include <ros/ros.h> // Include ROS header for ROS_ERROR and ROS_WARN

namespace four_wheel_steering_controller
{
namespace bacc = boost::accumulators;

  /**
   * @brief Constructs an Odometry object.
   * @param velocity_rolling_window_size The size of the rolling window for velocity calculations.
   */
  Odometry::Odometry(size_t velocity_rolling_window_size)
    : last_update_timestamp_(0.0),
      x_(0.0),
      y_(0.0),
      heading_(0.0),
      linear_(0.0),
      linear_x_(0.0),
      linear_y_(0.0),
      angular_(0.0),
      steering_track_(0.0),
      wheel_steering_y_offset_(0.0),
      wheel_radius_(0.0),
      wheel_base_(0.0),
      velocity_rolling_window_size_(velocity_rolling_window_size),
      linear_accel_acc_(RollingWindow::window_size = velocity_rolling_window_size),
      linear_jerk_acc_(RollingWindow::window_size = velocity_rolling_window_size),
      front_steer_vel_acc_(RollingWindow::window_size = velocity_rolling_window_size),
      rear_steer_vel_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  {
  }

  /**
   * @brief Initializes the odometry module.
   * @param time The current ROS time.
   */
  void Odometry::init(const ros::Time& time)
  {
    resetAccumulators();
    last_update_timestamp_ = time;
    linear_vel_prev_ = 0.0;
    linear_accel_prev_ = 0.0;
    front_steer_prev_ = 0.0;
    rear_steer_prev_ = 0.0;
  }

  /**
   * @brief Updates the odometry state based on wheel speeds and steering angles.
   * @param fl_speed Front left wheel speed.
   * @param fr_speed Front right wheel speed.
   * @param rl_speed Rear left wheel speed.
   * @param rr_speed Rear right wheel speed.
   * @param front_steering The equivalent front steering angle.
   * @param rear_steering The equivalent rear steering angle.
   * @param time The current ROS time.
   * @return True if the update was successful, false otherwise.
   */
  bool Odometry::update(const double &fl_speed, const double &fr_speed,
                        const double &rl_speed, const double &rr_speed,
                        double front_steering, double rear_steering, const ros::Time &time)
  {
    const double dt = (time - last_update_timestamp_).toSec();
    if (dt < 0.0001) {
      ROS_WARN("Update interval too small, skipping odometry update.");
      return false;
    }
    last_update_timestamp_ = time;

    // Calculate linear speeds for the front and rear axles
    const double front_linear_speed = calculateLinearSpeed(fl_speed, fr_speed, front_steering);
    const double rear_linear_speed = calculateLinearSpeed(rl_speed, rr_speed, rear_steering);

    // Calculate angular velocity
    const double front_tmp = cos(front_steering) * (tan(front_steering) - tan(rear_steering)) / wheel_base_;
    const double rear_tmp = cos(rear_steering) * (tan(front_steering) - tan(rear_steering)) / wheel_base_;
    angular_ = (front_linear_speed * front_tmp + rear_linear_speed * rear_tmp) / 2.0;

    // Calculate combined linear velocities in the robot's frame
    linear_x_ = (front_linear_speed * cos(front_steering) + rear_linear_speed * cos(rear_steering)) / 2.0;
    linear_y_ = (front_linear_speed * sin(front_steering) + rear_linear_speed * sin(rear_steering)) / 2.0;
    linear_ = copysign(1.0, linear_x_) * sqrt(pow(linear_x_, 2) + pow(linear_y_, 2));

    // Integrate odometry
    integrateXY(linear_x_ * dt, linear_y_ * dt, angular_ * dt);

    // Update rolling window accumulators
    updateRollingWindowStats(dt, front_steering, rear_steering);

    return true;
  }

  /**
   * @brief Sets the geometric parameters of the robot.
   * @param steering_track The distance between the steering joints.
   * @param wheel_steering_y_offset The offset between the wheel and steering joint on the y-axis.
   * @param wheel_radius The radius of the wheels.
   * @param wheel_base The distance between the front and rear axles.
   */
  void Odometry::setWheelParams(double steering_track, double wheel_steering_y_offset, double wheel_radius, double wheel_base)
  {
    if (steering_track <= 0 || wheel_radius <= 0 || wheel_base <= 0) {
      ROS_ERROR("Invalid wheel parameters. Must be positive.");
      return;
    }
    steering_track_ = steering_track;
    wheel_steering_y_offset_ = wheel_steering_y_offset;
    wheel_radius_ = wheel_radius;
    wheel_base_ = wheel_base;
  }

  /**
   * @brief Sets the size of the rolling window for velocity calculations.
   * @param velocity_rolling_window_size The new size of the window.
   */
  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    if (velocity_rolling_window_size == 0) {
      ROS_ERROR("Rolling window size cannot be zero.");
      return;
    }
    velocity_rolling_window_size_ = velocity_rolling_window_size;
    resetAccumulators();
  }

  /**
   * @brief Integrates the linear and angular velocities to update position and heading.
   * @param linear_x Linear velocity in the x-direction.
   * @param linear_y Linear velocity in the y-direction.
   * @param angular Angular velocity around the z-axis.
   */
  void Odometry::integrateXY(double linear_x, double linear_y, double angular)
  {
    const double delta_x = linear_x * cos(heading_) - linear_y * sin(heading_);
    const double delta_y = linear_x * sin(heading_) + linear_y * cos(heading_);

    x_ += delta_x;
    y_ += delta_y;
    heading_ += angular;
  }
  
  /**
   * @brief Calculates the linear speed of an axle based on its wheel speeds and steering angle.
   * @param left_speed The left wheel speed.
   * @param right_speed The right wheel speed.
   * @param steering_angle The equivalent steering angle for the axle.
   * @return The calculated linear speed for the axle.
   */
  double Odometry::calculateLinearSpeed(double left_speed, double right_speed, double steering_angle)
  {
      const double tmp = cos(steering_angle) * (tan(steering_angle) - tan(steering_angle)) / wheel_base_;
      const double left_tmp = tmp / sqrt(1 - steering_track_ * tmp * cos(steering_angle) + pow(steering_track_ * tmp / 2, 2));
      const double right_tmp = tmp / sqrt(1 + steering_track_ * tmp * cos(steering_angle) + pow(steering_track_ * tmp / 2, 2));

      const double left_speed_tmp = left_speed * (1 / (1 - wheel_steering_y_offset_ * left_tmp));
      const double right_speed_tmp = right_speed * (1 / (1 - wheel_steering_y_offset_ * right_tmp));

      return wheel_radius_ * copysign(1.0, left_speed_tmp + right_speed_tmp) *
             sqrt((pow(left_speed, 2) + pow(right_speed, 2)) / (2 + pow(steering_track_ * tmp, 2) / 2.0));
  }

  /**
   * @brief Updates rolling window accumulators for acceleration and jerk.
   * @param dt The time step.
   * @param front_steering The front steering angle.
   * @param rear_steering The rear steering angle.
   */
  void Odometry::updateRollingWindowStats(double dt, double front_steering, double rear_steering)
  {
    linear_accel_acc_((linear_ - linear_vel_prev_) / dt);
    linear_vel_prev_ = linear_;
    linear_jerk_acc_((bacc::rolling_mean(linear_accel_acc_) - linear_accel_prev_) / dt);
    linear_accel_prev_ = bacc::rolling_mean(linear_accel_acc_);
    front_steer_vel_acc_((front_steering - front_steer_prev_) / dt);
    front_steer_prev_ = front_steering;
    rear_steer_vel_acc_((rear_steering - rear_steer_prev_) / dt);
    rear_steer_prev_ = rear_steering;
  }

  /**
   * @brief Resets all boost accumulators.
   */
  void Odometry::resetAccumulators()
  {
    linear_accel_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    linear_jerk_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    front_steer_vel_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    rear_steer_vel_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }
} // namespace four_wheel_steering_controller