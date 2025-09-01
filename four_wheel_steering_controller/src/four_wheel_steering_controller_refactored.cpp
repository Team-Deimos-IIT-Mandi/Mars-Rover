#include <cmath>
#include <four_wheel_steering_controller/four_wheel_steering_controller.h>
#include <tf/transform_datatypes.h>
#include <urdf_geometry_parser/urdf_geometry_parser.h>

namespace four_wheel_steering_controller {

/*********************************************************************
 * Command Class
 *********************************************************************/
class FourWheelSteeringController::RobotCommand {
public:
  RobotCommand() : lin_x(0.0), lin_y(0.0), ang(0.0), front_steering(0.0), rear_steering(0.0), stamp(0.0) {}

  double lin_x, lin_y, ang;
  double front_steering, rear_steering;
  ros::Time stamp;
};

/*********************************************************************
 * Constructor
 *********************************************************************/
FourWheelSteeringController::FourWheelSteeringController()
  : command_twist_(), command_four_wheel_steering_(),
    track_(0.0), wheel_steering_y_offset_(0.0),
    wheel_radius_(0.0), wheel_base_(0.0),
    cmd_vel_timeout_(0.5), base_frame_id_("base_link"),
    enable_odom_tf_(true), enable_twist_cmd_(false) {}

/*********************************************************************
 * Initialization (called once at startup)
 *********************************************************************/
bool FourWheelSteeringController::init(hardware_interface::RobotHW *robot_hw,
                                       ros::NodeHandle& root_nh,
                                       ros::NodeHandle &controller_nh)
{
  const std::string complete_ns = controller_nh.getNamespace();
  name_ = complete_ns.substr(complete_ns.find_last_of("/") + 1);

  if (!loadJoints(robot_hw, controller_nh)) {
    return false;
  }
  if (!loadParameters(root_nh, controller_nh)) {
    return false;
  }
  setupPublishersAndSubscribers(root_nh, controller_nh);

  return true;
}

/*********************************************************************
 * Main update loop: called at controller rate
 *********************************************************************/
void FourWheelSteeringController::update(const ros::Time& time, const ros::Duration& period)
{
  updateOdometry(time);
  updateCommand(time, period);
}

/*********************************************************************
 * Lifecycle hooks
 *********************************************************************/
void FourWheelSteeringController::starting(const ros::Time& time)
{
  brake();
  last_state_publish_time_ = time;
  odometry_.init(time);
  last1_cmd_ = RobotCommand();
  last0_cmd_ = RobotCommand();
}

void FourWheelSteeringController::stopping(const ros::Time& /*time*/)
{
  brake();
}

/*********************************************************************
 * Helper: Load all joint handles
 *********************************************************************/
bool FourWheelSteeringController::loadJoints(hardware_interface::RobotHW *robot_hw,
                                             ros::NodeHandle& controller_nh)
{
  try {
    std::vector<std::string> front_wheel_names, rear_wheel_names, front_steering_names, rear_steering_names;
    if (!getWheelNames(controller_nh, "front_wheel", front_wheel_names) ||
        !getWheelNames(controller_nh, "rear_wheel", rear_wheel_names) ||
        !getWheelNames(controller_nh, "front_steering", front_steering_names) ||
        !getWheelNames(controller_nh, "rear_steering", rear_steering_names))
    {
      return false;
    }

    if (front_wheel_names.size() != 2 || rear_wheel_names.size() != 2 ||
        front_steering_names.size() != 2 || rear_steering_names.size() != 2)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid joint configuration. Expected 2 wheels and 2 steering joints per axle.");
      return false;
    }

    front_wheel_joints_.resize(2);
    rear_wheel_joints_.resize(2);
    front_steering_joints_.resize(2);
    rear_steering_joints_.resize(2);

    auto* vel_joint_hw = robot_hw->get<hardware_interface::VelocityJointInterface>();
    auto* pos_joint_hw = robot_hw->get<hardware_interface::PositionJointInterface>();

    for (size_t i = 0; i < 2; ++i)
    {
      front_wheel_joints_[i] = vel_joint_hw->getHandle(front_wheel_names[i]);
      rear_wheel_joints_[i]  = vel_joint_hw->getHandle(rear_wheel_names[i]);
      front_steering_joints_[i] = pos_joint_hw->getHandle(front_steering_names[i]);
      rear_steering_joints_[i]  = pos_joint_hw->getHandle(rear_steering_names[i]);
    }
  } catch(const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM_NAMED(name_, "Failed to get joint handles: " << e.what());
    return false;
  }
  return true;
}

/*********************************************************************
 * Helper: Load controller parameters
 *********************************************************************/
bool FourWheelSteeringController::loadParameters(ros::NodeHandle& root_nh,
                                                 ros::NodeHandle& controller_nh)
{
  double publish_rate;
  getParamAndLog(controller_nh, "publish_rate", publish_rate, 50.0);
  publish_period_ = ros::Duration(1.0 / publish_rate);

  getParamAndLog(controller_nh, "cmd_vel_timeout", cmd_vel_timeout_, 0.5);
  getParamAndLog(controller_nh, "base_frame_id", base_frame_id_, std::string("base_link"));
  getParamAndLog(controller_nh, "enable_odom_tf", enable_odom_tf_, true);
  getParamAndLog(controller_nh, "open_loop", open_loop_, false);

  int velocity_rolling_window_size = 10;
  getParamAndLog(controller_nh, "velocity_rolling_window_size", velocity_rolling_window_size, 10);
  odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

  getParamAndLog(controller_nh, "linear/x/has_velocity_limits", limiter_lin_.has_velocity_limits, false);
  getParamAndLog(controller_nh, "linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits, false);
  getParamAndLog(controller_nh, "linear/x/max_velocity", limiter_lin_.max_velocity, 0.0);
  getParamAndLog(controller_nh, "linear/x/min_velocity", limiter_lin_.min_velocity, -limiter_lin_.max_velocity);
  getParamAndLog(controller_nh, "linear/x/max_acceleration", limiter_lin_.max_acceleration, 0.0);
  getParamAndLog(controller_nh, "linear/x/min_acceleration", limiter_lin_.min_acceleration, -limiter_lin_.max_acceleration);

  getParamAndLog(controller_nh, "angular/z/has_velocity_limits", limiter_ang_.has_velocity_limits, false);
  getParamAndLog(controller_nh, "angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, false);
  getParamAndLog(controller_nh, "angular/z/max_velocity", limiter_ang_.max_velocity, 0.0);
  getParamAndLog(controller_nh, "angular/z/min_velocity", limiter_ang_.min_velocity, -limiter_ang_.max_velocity);
  getParamAndLog(controller_nh, "angular/z/max_acceleration", limiter_ang_.max_acceleration, 0.0);
  getParamAndLog(controller_nh, "angular/z/min_acceleration", limiter_ang_.min_acceleration, -limiter_ang_.max_acceleration);

  urdf_geometry_parser::UrdfGeometryParser uvk(root_nh, base_frame_id_);
  if (!controller_nh.getParam("track", track_))
    if (!uvk.getDistanceBetweenJoints(front_wheel_joints_[0].getName(), front_wheel_joints_[1].getName(), track_)) return false;

  if (!controller_nh.getParam("wheel_radius", wheel_radius_))
    if (!uvk.getJointRadius(front_wheel_joints_[0].getName(), wheel_radius_)) return false;

  if (!controller_nh.getParam("wheel_base", wheel_base_))
    if (!uvk.getDistanceBetweenJoints(front_wheel_joints_[0].getName(), rear_wheel_joints_[0].getName(), wheel_base_)) return false;

  if (!uvk.getDistanceBetweenJoints(front_steering_joints_[0].getName(), front_wheel_joints_[0].getName(), wheel_steering_y_offset_)) return false;

  odometry_.setWheelParams(track_ - 2*wheel_steering_y_offset_, wheel_steering_y_offset_, wheel_radius_, wheel_base_);
  return true;
}

/*********************************************************************
 * Helper: Setup publishers & subscribers
 *********************************************************************/
void FourWheelSteeringController::setupPublishersAndSubscribers(ros::NodeHandle& root_nh,
                                                                ros::NodeHandle& controller_nh)
{
  setOdomPubFields(root_nh, controller_nh);
  sub_command_ = controller_nh.subscribe("cmd_vel", 1, &FourWheelSteeringController::cmdVelCallback, this);
  sub_command_four_wheel_steering_ = controller_nh.subscribe("cmd_four_wheel_steering", 1, &FourWheelSteeringController::cmdFourWheelSteeringCallback, this);
}

/*********************************************************************
 * Update Odometry
 *********************************************************************/
void FourWheelSteeringController::updateOdometry(const ros::Time& time)
{
  double fl_speed = front_wheel_joints_[0].getVelocity();
  double fr_speed = front_wheel_joints_[1].getVelocity();
  double rl_speed = rear_wheel_joints_[0].getVelocity();
  double rr_speed = rear_wheel_joints_[1].getVelocity();

  double fl_steering = front_steering_joints_[0].getPosition();
  double fr_steering = front_steering_joints_[1].getPosition();
  double rl_steering = rear_steering_joints_[0].getPosition();
  double rr_steering = rear_steering_joints_[1].getPosition();

  double front_steering_pos = computeEquivalentSteering(fl_steering, fr_steering);
  double rear_steering_pos  = computeEquivalentSteering(rl_steering, rr_steering);

  odometry_.update(fl_speed, fr_speed, rl_speed, rr_speed,
                   front_steering_pos, rear_steering_pos, time);

  if ((time - last_state_publish_time_).toSec() >= publish_period_.toSec())
    publishOdometry(time, front_steering_pos, rear_steering_pos);
}

/*********************************************************************
 * Update Command
 *********************************************************************/
void FourWheelSteeringController::updateCommand(const ros::Time& time, const ros::Duration& period)
{
  RobotCommand curr_cmd_twist = *(command_twist_.readFromRT());
  RobotCommand curr_cmd_4ws = *(command_four_wheel_steering_.readFromRT());

  bool use_twist = (curr_cmd_twist.stamp >= curr_cmd_4ws.stamp);
  RobotCommand* current_command = use_twist ? &curr_cmd_twist : &curr_cmd_4ws;

  if ((time - current_command->stamp).toSec() > cmd_vel_timeout_) {
    current_command->lin_x = 0.0;
    current_command->lin_y = 0.0;
    current_command->ang = 0.0;
    current_command->front_steering = 0.0;
    current_command->rear_steering = 0.0;
  }

  if (use_twist) {
    applyTwistCommand(curr_cmd_twist, period);
  } else {
    applyFourWheelSteeringCommand(curr_cmd_4ws, period);
  }

  // Publish debug data
  if (velo_and_pos_->trylock()){
    velo_and_pos_->msg_.data = {front_wheel_joints_[0].getCommand(), front_wheel_joints_[1].getCommand(),
                                rear_wheel_joints_[0].getCommand(), rear_wheel_joints_[1].getCommand(),
                                front_steering_joints_[0].getCommand(), front_steering_joints_[1].getCommand(),
                                rear_steering_joints_[0].getCommand(), rear_steering_joints_[1].getCommand()};
    velo_and_pos_->unlockAndPublish();
  }
}

/*********************************************************************
 * Helpers for updateOdometry
 *********************************************************************/
double FourWheelSteeringController::computeEquivalentSteering(double left, double right)
{
  if (fabs(left) > 0.001 || fabs(right) > 0.001)
    return atan(2 * tan(left) * tan(right) / (tan(left) + tan(right)));
  return 0.0;
}

void FourWheelSteeringController::publishOdometry(const ros::Time& time,
                                                  double front_steering, double rear_steering)
{
  last_state_publish_time_ = time; // Update to current time for precise period calculation
  geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

  if (odom_pub_->trylock())
  {
    odom_pub_->msg_.header.stamp = time;
    odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
    odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
    odom_pub_->msg_.pose.pose.orientation = orientation;
    odom_pub_->msg_.twist.twist.linear.x = odometry_.getLinearX();
    odom_pub_->msg_.twist.twist.linear.y = odometry_.getLinearY();
    odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
    odom_pub_->unlockAndPublish();
  }

  if (odom_4ws_pub_->trylock())
  {
    odom_4ws_pub_->msg_.header.stamp = time;
    odom_4ws_pub_->msg_.data.speed = odometry_.getLinear();
    odom_4ws_pub_->msg_.data.acceleration = odometry_.getLinearAcceleration();
    odom_4ws_pub_->msg_.data.jerk = odometry_.getLinearJerk();
    odom_4ws_pub_->msg_.data.front_steering_angle = front_steering;
    odom_4ws_pub_->msg_.data.front_steering_angle_velocity = odometry_.getFrontSteerVel();
    odom_4ws_pub_->msg_.data.rear_steering_angle  = rear_steering;
    odom_4ws_pub_->msg_.data.rear_steering_angle_velocity = odometry_.getRearSteerVel();
    odom_4ws_pub_->unlockAndPublish();
  }

  if (enable_odom_tf_ && tf_odom_pub_->trylock())
  {
    geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
    odom_frame.header.stamp = time;
    odom_frame.transform.translation.x = odometry_.getX();
    odom_frame.transform.translation.y = odometry_.getY();
    odom_frame.transform.rotation = orientation;
    tf_odom_pub_->unlockAndPublish();
  }
}

/*********************************************************************
 * Helpers for updateCommand
 *********************************************************************/
void FourWheelSteeringController::applyTwistCommand(const RobotCommand& cmd, const ros::Duration& period)
{
  RobotCommand limited_cmd = cmd;
  limiter_lin_.limit(limited_cmd.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x, period.toSec());
  limiter_ang_.limit(limited_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, period.toSec());

  last1_cmd_ = last0_cmd_;
  last0_cmd_ = limited_cmd;

  double vel_left_front = 0, vel_right_front = 0;
  double vel_left_rear = 0, vel_right_rear = 0;
  double front_left_steering = 0, front_right_steering = 0;
  double rear_left_steering = 0, rear_right_steering = 0;

  const double steering_track = track_ - 2*wheel_steering_y_offset_;

  if (fabs(limited_cmd.lin_x) > 0.001)
  {
    const double vel_steering_offset = (limited_cmd.ang * wheel_steering_y_offset_) / wheel_radius_;
    const double sign = copysign(1.0, limited_cmd.lin_x);
    vel_left_front = sign * std::hypot((limited_cmd.lin_x - limited_cmd.ang * steering_track / 2.0), (wheel_base_ * limited_cmd.ang / 2.0)) / wheel_radius_ - vel_steering_offset;
    vel_right_front = sign * std::hypot((limited_cmd.lin_x + limited_cmd.ang * steering_track / 2.0), (wheel_base_ * limited_cmd.ang / 2.0)) / wheel_radius_ + vel_steering_offset;
    vel_left_rear = sign * std::hypot((limited_cmd.lin_x - limited_cmd.ang * steering_track / 2.0), (wheel_base_ * limited_cmd.ang / 2.0)) / wheel_radius_ - vel_steering_offset;
    vel_right_rear = sign * std::hypot((limited_cmd.lin_x + limited_cmd.ang * steering_track / 2.0), (wheel_base_ * limited_cmd.ang / 2.0)) / wheel_radius_ + vel_steering_offset;
  }

  if (fabs(2.0 * limited_cmd.lin_x) > fabs(limited_cmd.ang * steering_track) && fabs(limited_cmd.lin_y) < 0.001)
  {
    front_left_steering = atan(limited_cmd.ang * wheel_base_ / (2.0 * limited_cmd.lin_x - limited_cmd.ang * steering_track));
    front_right_steering = atan(limited_cmd.ang * wheel_base_ / (2.0 * limited_cmd.lin_x + limited_cmd.ang * steering_track));
    rear_left_steering = -front_left_steering;
    rear_right_steering = -front_right_steering;
  }
  else if (fabs(limited_cmd.lin_x) > 0.001 && fabs(limited_cmd.lin_y) < 0.001)
  {
    front_left_steering = copysign(M_PI_2, limited_cmd.ang);
    front_right_steering = copysign(M_PI_2, limited_cmd.ang);
    rear_left_steering = -front_left_steering;
    rear_right_steering = -front_right_steering;
  }
  else if (fabs(limited_cmd.lin_x) > 0.001 && fabs(limited_cmd.lin_y) >= 0.001 && fabs(limited_cmd.ang) < 0.001)
  {
    front_left_steering = atan(limited_cmd.lin_y / limited_cmd.lin_x);
    front_right_steering = atan(limited_cmd.lin_y / limited_cmd.lin_x);
    rear_left_steering = front_left_steering;
    rear_right_steering = front_right_steering;
  }

  setWheelCommands(vel_left_front, vel_right_front, vel_left_rear, vel_right_rear,
                   front_left_steering, front_right_steering, rear_left_steering, rear_right_steering);
}

void FourWheelSteeringController::applyFourWheelSteeringCommand(const RobotCommand& cmd, const ros::Duration& period)
{
  RobotCommand limited_cmd = cmd;
  limiter_lin_.limit(limited_cmd.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x, period.toSec());
  last1_cmd_ = last0_cmd_;
  last0_cmd_.lin_x = limited_cmd.lin_x;

  limited_cmd.front_steering = clamp(limited_cmd.front_steering, -M_PI_2, M_PI_2);
  limited_cmd.rear_steering = clamp(limited_cmd.rear_steering, -M_PI_2, M_PI_2);

  double vel_left_front = 0, vel_right_front = 0;
  double vel_left_rear = 0, vel_right_rear = 0;
  double front_left_steering = 0, front_right_steering = 0;
  double rear_left_steering = 0, rear_right_steering = 0;

  const double steering_track = track_ - 2 * wheel_steering_y_offset_;
  const double tan_front_steering = tan(limited_cmd.front_steering);
  const double tan_rear_steering = tan(limited_cmd.rear_steering);
  const double steering_diff = steering_track * (tan_front_steering - tan_rear_steering) / 2.0;

  if (fabs(wheel_base_ - fabs(steering_diff)) > 0.001)
  {
    front_left_steering = atan(wheel_base_ * tan_front_steering / (wheel_base_ - steering_diff));
    front_right_steering = atan(wheel_base_ * tan_front_steering / (wheel_base_ + steering_diff));
    rear_left_steering = atan(wheel_base_ * tan_rear_steering / (wheel_base_ - steering_diff));
    rear_right_steering = atan(wheel_base_ * tan_rear_steering / (wheel_base_ + steering_diff));
  }

  if (fabs(limited_cmd.lin_x) > 0.001)
  {
    double l_front = 0;
    if (fabs(tan(front_left_steering) - tan(front_right_steering)) > 0.01)
    {
      l_front = tan(front_right_steering) * tan(front_left_steering) * steering_track / (tan(front_left_steering) - tan(front_right_steering));
    }
    double l_rear = 0;
    if (fabs(tan(rear_left_steering) - tan(rear_right_steering)) > 0.01)
    {
      l_rear = tan(rear_right_steering) * tan(rear_left_steering) * steering_track / (tan(rear_left_steering) - tan(rear_right_steering));
    }

    const double angular_speed_cmd = limited_cmd.lin_x * (tan_front_steering - tan_rear_steering) / wheel_base_;
    const double vel_steering_offset = (angular_speed_cmd * wheel_steering_y_offset_) / wheel_radius_;
    const double sign = copysign(1.0, limited_cmd.lin_x);

    vel_left_front = sign * std::hypot((limited_cmd.lin_x - angular_speed_cmd * steering_track / 2.0), (l_front * angular_speed_cmd)) / wheel_radius_ - vel_steering_offset;
    vel_right_front = sign * std::hypot((limited_cmd.lin_x + angular_speed_cmd * steering_track / 2.0), (l_front * angular_speed_cmd)) / wheel_radius_ + vel_steering_offset;
    vel_left_rear = sign * std::hypot((limited_cmd.lin_x - angular_speed_cmd * steering_track / 2.0), (l_rear * angular_speed_cmd)) / wheel_radius_ - vel_steering_offset;
    vel_right_rear = sign * std::hypot((limited_cmd.lin_x + angular_speed_cmd * steering_track / 2.0), (l_rear * angular_speed_cmd)) / wheel_radius_ + vel_steering_offset;
  }

  setWheelCommands(vel_left_front, vel_right_front, vel_left_rear, vel_right_rear,
                   front_left_steering, front_right_steering, rear_left_steering, rear_right_steering);
}

/*********************************************************************
 * Brake robot
 *********************************************************************/
void FourWheelSteeringController::brake()
{
  for (auto& j : front_wheel_joints_) j.setCommand(0.0);
  for (auto& j : rear_wheel_joints_)  j.setCommand(0.0);
  for (auto& j : front_steering_joints_) j.setCommand(0.0);
  for (auto& j : rear_steering_joints_)  j.setCommand(0.0);
}

/*********************************************************************
 * Set commands to joints
 *********************************************************************/
void FourWheelSteeringController::setWheelCommands(double vel_lf, double vel_rf, double vel_lr, double vel_rr,
                                                   double pos_lf, double pos_rf, double pos_lr, double pos_rr)
{
  front_wheel_joints_[0].setCommand(vel_lf);
  front_wheel_joints_[1].setCommand(vel_rf);
  rear_wheel_joints_[0].setCommand(vel_lr);
  rear_wheel_joints_[1].setCommand(vel_rr);

  front_steering_joints_[0].setCommand(pos_lf);
  front_steering_joints_[1].setCommand(pos_rf);
  rear_steering_joints_[0].setCommand(pos_lr);
  rear_steering_joints_[1].setCommand(pos_rr);
}

/*********************************************************************
 * Command callbacks
 *********************************************************************/
void FourWheelSteeringController::cmdVelCallback(const geometry_msgs::Twist& command)
{
  if (!isRunning()) return;

  if (std::isnan(command.angular.z) || std::isnan(command.linear.x)) {
    ROS_WARN("Received NaN in geometry_msgs::Twist. Ignoring command.");
    return;
  }
  RobotCommand command_struct;
  command_struct.ang = command.angular.z;
  command_struct.lin_x = command.linear.x;
  command_struct.lin_y = command.linear.y;
  command_struct.stamp = ros::Time::now();
  command_twist_.writeFromNonRT(command_struct);
}

void FourWheelSteeringController::cmdFourWheelSteeringCallback(const four_wheel_steering_msgs::FourWheelSteering& command)
{
  if (!isRunning()) return;

  if (std::isnan(command.front_steering_angle) || std::isnan(command.rear_steering_angle) || std::isnan(command.speed)) {
    ROS_WARN("Received NaN in four_wheel_steering_msgs::FourWheelSteering. Ignoring command.");
    return;
  }
  RobotCommand command_struct;
  command_struct.front_steering = command.front_steering_angle;
  command_struct.rear_steering = command.rear_steering_angle;
  command_struct.lin_x = command.speed;
  command_struct.stamp = ros::Time::now();
  command_four_wheel_steering_.writeFromNonRT(command_struct);
}

/*********************************************************************
 * Helper: Retrieve joint names from parameter server
 *********************************************************************/
bool FourWheelSteeringController::getWheelNames(ros::NodeHandle& controller_nh,
                                                const std::string& wheel_param,
                                                std::vector<std::string>& wheel_names)
{
  XmlRpc::XmlRpcValue wheel_list;
  if (!controller_nh.getParam(wheel_param, wheel_list))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve wheel param '" << wheel_param << "'.");
    return false;
  }

  if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (wheel_list.size() == 0)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' is an empty list");
      return false;
    }
    wheel_names.resize(wheel_list.size());
    for (int i = 0; i < wheel_list.size(); ++i)
    {
      if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' #" << i << " isn't a string.");
        return false;
      }
      wheel_names[i] = static_cast<std::string>(wheel_list[i]);
    }
  }
  else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    wheel_names.push_back(static_cast<std::string>(wheel_list));
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' is neither a list of strings nor a string.");
    return false;
  }
  return true;
}

/*********************************************************************
 * Helper: Setup odometry publishers
 *********************************************************************/
void FourWheelSteeringController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  XmlRpc::XmlRpcValue pose_cov_list, twist_cov_list;
  controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);

  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = base_frame_id_;
  odom_pub_->msg_.pose.pose.position.z = 0;

  for (int i = 0; i < 6; ++i)
  {
    odom_pub_->msg_.pose.covariance[i * 7] = static_cast<double>(pose_cov_list[i]);
    odom_pub_->msg_.twist.covariance[i * 7] = static_cast<double>(twist_cov_list[i]);
  }

  odom_4ws_pub_.reset(new realtime_tools::RealtimePublisher<four_wheel_steering_msgs::FourWheelSteeringStamped>(controller_nh, "odom_steer", 100));
  odom_4ws_pub_->msg_.header.frame_id = "odom";

  velo_and_pos_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>(controller_nh, "velo_and_pos", 100));

  tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
  tf_odom_pub_->msg_.transforms.resize(1);
  tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
  tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
  tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
}

// Ensure the clamp function is defined or included
template <typename T>
T FourWheelSteeringController::clamp(T val, T min_val, T max_val)
{
  return std::max(min_val, std::min(val, max_val));
}

// Helper function to get parameter and log
template <typename T>
void FourWheelSteeringController::getParamAndLog(ros::NodeHandle& nh, const std::string& name, T& value, const T& default_value)
{
  if (!nh.getParam(name, value))
  {
    value = default_value;
    ROS_WARN_STREAM_NAMED(this->name_, "Failed to get '" << name << "' parameter from server. Using default value: " << value);
  }
}

} // namespace four_wheel_steering_controller