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

#include <controller_interface/multi_interface_controller.h>

#include <control_msgs/JointTrajectoryControllerState.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>

#include <vector>
#include <cmath>

#include <swerve_steering_controller/utils.h>
#include <swerve_steering_controller/odometry.h>
#include <swerve_steering_controller/wheel.h>
#include <swerve_steering_controller/speed_limiter.h>


namespace swerve_steering_controller
{
    class SwerveSteeringController
          : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface, hardware_interface::PositionJointInterface>
  {
      public: 
          SwerveSteeringController();

          bool init(hardware_interface::RobotHW* robot_hw,
                    ros::NodeHandle& root_nh,
                    ros::NodeHandle& controller_nh);


          void update(const ros::Time& time, const ros::Duration& period);

          void starting(const ros::Time& time);

          void stopping(const ros::Time& time);

      private:
          std::string name_;

          std::string base_frame_id_;
          std::string odom_frame_id_;

          ros::Duration publish_period_;
          ros::Time last_state_publish_time_;

          std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_publisher_;
          std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_publisher_;
          std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Point> > avg_intersection_publisher_;

          Odometry odometry_;
          
          double infinity_tol_;
          double intersection_tol_;
          
          /// Speed limiters:
          utils::command last1_cmd_;
          utils::command last0_cmd_;
          SpeedLimiter limiter_lin_x_;
          SpeedLimiter limiter_lin_y_;
          SpeedLimiter limiter_ang_;



          /// Previous time and velocities from the encoders:
          ros::Time time_previous_;
          std::vector<double> wheels_velocities_previous_;
          std::vector<double> holders_velocities_previous_;

          std::vector<double> wheels_desired_velocities_previous_;
          std::vector<double> holders_desired_velocities_previous_;
          std::vector<double> holders_desired_positions_previous_;

          bool enable_odom_tf_;
          bool publish_wheel_joint_controller_state_;

          size_t wheel_joints_size_;

          std::vector<wheel> wheels_;
          std::vector<hardware_interface::JointHandle>  wheels_joints_handles_;
          std::vector<hardware_interface::JointHandle> holders_joints_handles_;

          utils::command cmd_;
          ros::Subscriber cmd_subscriber_;
          realtime_tools::RealtimeBuffer<utils::command> commands_buffer_;

          std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState> > controller_state_pub_;

          void cmd_callback(const geometry_msgs::Twist& command);

          bool getWheelParams(ros::NodeHandle& controller_nh, const std::string& wheel_param, const std::string& holder_param, std::vector<std::string>& wheel_names, std::vector<std::string>& holder_names);
          bool getXmlStringList(ros::NodeHandle& node_handler, const std::string& list_param, std::vector<std::string>& returned_names);

          void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  
          // Controller state publisher.. includes the whels and the holders 
          std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState> > controller_state_publisher;

          void set_to_initial_state();

          void publishWheelData(const ros::Time& time, const ros::Duration& period, std::vector<double> wheels_desired_velocities, std::vector<double> holders_desired_positions);
  
    };
}// namespace swerve_steering_controller
  
