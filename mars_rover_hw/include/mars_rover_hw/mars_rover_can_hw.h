#ifndef MARS_ROVER_CAN_HW_H
#define MARS_ROVER_CAN_HW_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <map>

// Linux SocketCAN headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

namespace mars_rover_hw {

class MarsRoverCANHW : public hardware_interface::RobotHW {
public:
    MarsRoverCANHW();
    ~MarsRoverCANHW();
    
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);

private:
    // CAN socket file descriptor
    int can_socket_;
    std::string can_interface_;
    
    // Joint configuration
    std::vector<std::string> joint_names_;
    int num_joints_;
    
    // Joint state data (read from hardware)
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    
    // Joint command data (write to hardware)
    std::vector<double> joint_velocity_command_;
    
    // CAN IDs for each motor
    std::map<std::string, uint32_t> motor_can_ids_;
    
    // Hardware interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    
    // Helper methods
    bool initCAN();
    void closeCAN();
    bool sendCANFrame(uint32_t can_id, const uint8_t* data, uint8_t len);
    bool readCANFrame(struct can_frame& frame);
};

} // namespace mars_rover_hw

#endif // MARS_ROVER_CAN_HW_H
