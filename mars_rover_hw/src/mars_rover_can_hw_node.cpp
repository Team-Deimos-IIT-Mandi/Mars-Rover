#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "mars_rover_hw/mars_rover_can_hw.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mars_rover_hw_node");
    ros::NodeHandle nh;
    ros::NodeHandle robot_hw_nh("~");
    
    // Create hardware interface
    mars_rover_hw::MarsRoverCANHW robot;
    
    // Initialize hardware
    if (!robot.init(nh, robot_hw_nh)) {
        ROS_FATAL("Failed to initialize Mars Rover hardware interface");
        return -1;
    }
    
    // Create controller manager
    controller_manager::ControllerManager cm(&robot, nh);
    
    // Setup async spinner for handling callbacks
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    // Control loop timing
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(50.0); // 50Hz control loop
    
    ROS_INFO("Starting Mars Rover hardware interface control loop at 50Hz");
    
    while (ros::ok()) {
        const ros::Time now = ros::Time::now();
        const ros::Duration period = now - prev_time;
        
        // Read from hardware
        robot.read(now, period);
        
        // Update controllers
        cm.update(now, period);
        
        // Write to hardware
        robot.write(now, period);
        
        prev_time = now;
        rate.sleep();
    }
    
    spinner.stop();
    return 0;
}
