#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

// Custom hardware interface class that inherits from RobotHW
class MarsRoverHW : public hardware_interface::RobotHW
{
public:
    MarsRoverHW() 
    {
        // Register interfaces
        hardware_interface::JointStateHandle state_handle_1("wheel_1", &pos[0], &vel[0], &eff[0]);
        hardware_interface::JointStateHandle state_handle_2("wheel_2", &pos[1], &vel[1], &eff[1]);
        hardware_interface::JointStateHandle state_handle_3("wheel_3", &pos[2], &vel[2], &eff[2]);
        hardware_interface::JointStateHandle state_handle_4("wheel_4", &pos[3], &vel[3], &eff[3]);
        
        hardware_interface::JointStateHandle state_handle_5("steer1", &pos[4], &vel[4], &eff[4]);
        hardware_interface::JointStateHandle state_handle_6("steer2", &pos[5], &vel[5], &eff[5]);
        hardware_interface::JointStateHandle state_handle_7("steer_3", &pos[6], &vel[6], &eff[6]);
        hardware_interface::JointStateHandle state_handle_8("steer4", &pos[7], &vel[7], &eff[7]);

        joint_state_interface.registerHandle(state_handle_1);
        joint_state_interface.registerHandle(state_handle_2);
        joint_state_interface.registerHandle(state_handle_3);
        joint_state_interface.registerHandle(state_handle_4);
        joint_state_interface.registerHandle(state_handle_5);
        joint_state_interface.registerHandle(state_handle_6);
        joint_state_interface.registerHandle(state_handle_7);
        joint_state_interface.registerHandle(state_handle_8);
        
        ROS_INFO("Handle Registered");
        registerInterface(&joint_state_interface);

        // Register velocity interface for wheels
        hardware_interface::JointHandle vel_handle_1(joint_state_interface.getHandle("wheel_1"), &cmd_vel[0]);
        hardware_interface::JointHandle vel_handle_2(joint_state_interface.getHandle("wheel_2"), &cmd_vel[1]);
        hardware_interface::JointHandle vel_handle_3(joint_state_interface.getHandle("wheel_3"), &cmd_vel[2]);
        hardware_interface::JointHandle vel_handle_4(joint_state_interface.getHandle("wheel_4"), &cmd_vel[3]);
        
        vel_joint_interface.registerHandle(vel_handle_1);
        vel_joint_interface.registerHandle(vel_handle_2);
        vel_joint_interface.registerHandle(vel_handle_3);
        vel_joint_interface.registerHandle(vel_handle_4);
        
        registerInterface(&vel_joint_interface);

        // Register position interface for steering
        ROS_INFO("Hello -Balloon boy");
        hardware_interface::JointHandle pos_handle_1(joint_state_interface.getHandle("steer1"), &cmd_pos[0]);
        hardware_interface::JointHandle pos_handle_2(joint_state_interface.getHandle("steer2"), &cmd_pos[1]);
        hardware_interface::JointHandle pos_handle_3(joint_state_interface.getHandle("steer_3"), &cmd_pos[2]);
        hardware_interface::JointHandle pos_handle_4(joint_state_interface.getHandle("steer4"), &cmd_pos[3]);
        ROS_INFO("Hello -Balloon boy");
        pos_joint_interface.registerHandle(pos_handle_1);
        pos_joint_interface.registerHandle(pos_handle_2);
        pos_joint_interface.registerHandle(pos_handle_3);
        pos_joint_interface.registerHandle(pos_handle_4);
        
        registerInterface(&pos_joint_interface);
    }

    void read() {
        // Read hardware state (in a real robot, you'd read sensors here)
    }

    void write() {
        // Write commands to hardware (in a real robot, you'd command motors here)
    }

private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface vel_joint_interface;
    hardware_interface::PositionJointInterface pos_joint_interface;
    
    double cmd_pos[4] = {0.0, 0.0, 0.0, 0.0};
    double cmd_vel[4] = {0.0, 0.0, 0.0, 0.0};
    double pos[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double vel[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double eff[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "mars_rover_hw_interface");
    ros::NodeHandle nh;
    ROS_INFO("In main");
    // Create the hardware interface
    MarsRoverHW hw;
    
    // Create the controller manager
 
    controller_manager::ControllerManager cm(&hw, nh);
    ROS_INFO("Started Controller Manager");
    // Control loop
    ros::Rate rate(100); // 100Hz
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        // Read hardware state
        hw.read();

        // Update controllers
        ros::Time now = ros::Time::now();
        ros::Duration period = rate.expectedCycleTime();
        cm.update(now, period);

        // Write hardware commands
        hw.write();

        rate.sleep();
    }

    return 0;
}