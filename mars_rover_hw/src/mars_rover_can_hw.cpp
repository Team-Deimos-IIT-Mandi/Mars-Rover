#include "mars_rover_hw/mars_rover_can_hw.h"

namespace mars_rover_hw {

MarsRoverCANHW::MarsRoverCANHW() 
    : can_socket_(-1), 
      can_interface_("can0"),
      num_joints_(4) {
}

MarsRoverCANHW::~MarsRoverCANHW() {
    closeCAN();
}

bool MarsRoverCANHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
    // Get parameters from ROS parameter server
    robot_hw_nh.param<std::string>("can_interface", can_interface_, "can0");
    
    // Define joint names (4 drive wheels)
    joint_names_ = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
    num_joints_ = joint_names_.size();
    
    // Define CAN IDs for each motor (matching your Arduino format)
    motor_can_ids_["wheel_fl"] = 0x123;
    motor_can_ids_["wheel_fr"] = 0x124;
    motor_can_ids_["wheel_rl"] = 0x125;
    motor_can_ids_["wheel_rr"] = 0x126;
    
    // Resize state/command vectors
    joint_position_.resize(num_joints_, 0.0);
    joint_velocity_.resize(num_joints_, 0.0);
    joint_effort_.resize(num_joints_, 0.0);
    joint_velocity_command_.resize(num_joints_, 0.0);
    
    // Register joint state interface
    for (int i = 0; i < num_joints_; i++) {
        hardware_interface::JointStateHandle state_handle(
            joint_names_[i],
            &joint_position_[i],
            &joint_velocity_[i],
            &joint_effort_[i]
        );
        joint_state_interface_.registerHandle(state_handle);
        
        // Register velocity command interface
        hardware_interface::JointHandle velocity_handle(
            joint_state_interface_.getHandle(joint_names_[i]),
            &joint_velocity_command_[i]
        );
        velocity_joint_interface_.registerHandle(velocity_handle);
    }
    
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    
    // Initialize CAN bus
    if (!initCAN()) {
        ROS_ERROR("Failed to initialize CAN interface!");
        return false;
    }
    
    ROS_INFO("Mars Rover CAN Hardware Interface initialized successfully");
    return true;
}

bool MarsRoverCANHW::initCAN() {
    struct sockaddr_can addr;
    struct ifreq ifr;
    
    // Create CAN socket
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        ROS_ERROR("Error creating CAN socket: %s", strerror(errno));
        return false;
    }
    
    // Get interface index
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        ROS_ERROR("Error getting interface index for %s: %s", 
                  can_interface_.c_str(), strerror(errno));
        close(can_socket_);
        return false;
    }
    
    // Bind socket to CAN interface
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ROS_ERROR("Error binding CAN socket: %s", strerror(errno));
        close(can_socket_);
        return false;
    }
    
    // Set socket to non-blocking mode
    int flags = fcntl(can_socket_, F_GETFL, 0);
    fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
    
    ROS_INFO("CAN socket initialized on interface: %s", can_interface_.c_str());
    return true;
}

void MarsRoverCANHW::closeCAN() {
    if (can_socket_ >= 0) {
        close(can_socket_);
        can_socket_ = -1;
        ROS_INFO("CAN socket closed");
    }
}

void MarsRoverCANHW::read(const ros::Time& time, const ros::Duration& period) {
    // Read CAN frames (encoder feedback from motors)
    struct can_frame frame;
    while (readCANFrame(frame)) {
        // Parse feedback based on CAN ID
        // For now, assume open-loop (velocity command = actual velocity)
        ROS_DEBUG_THROTTLE(5.0, "Received CAN frame ID: 0x%X", frame.can_id);
    }
    
    // Update joint states (open-loop for now)
    for (int i = 0; i < num_joints_; i++) {
        joint_velocity_[i] = joint_velocity_command_[i];
        joint_position_[i] += joint_velocity_[i] * period.toSec();
    }
}

void MarsRoverCANHW::write(const ros::Time& time, const ros::Duration& period) {
    // Send velocity commands to motors via CAN
    for (int i = 0; i < num_joints_; i++) {
        uint32_t can_id = motor_can_ids_[joint_names_[i]];
        
        // Convert velocity to speed/direction (matching Arduino format)
        // Velocity range: -2.0 to +2.0 rad/s -> Speed: 0-1000
        double velocity = joint_velocity_command_[i];
        uint16_t speed = static_cast<uint16_t>(std::abs(velocity) * 500.0);
        uint8_t direction = (velocity >= 0) ? 0 : 1;
        
        // Clamp speed
        if (speed > 1000) speed = 1000;
        
        // Pack CAN frame data (same format as Arduino)
        uint8_t data[8] = {0};
        data[0] = speed & 0xFF;           // Low byte
        data[1] = (speed >> 8) & 0xFF;    // High byte
        data[2] = direction;               // Direction
        // data[3-7] = 0 (padding)
        
        // Send CAN frame
        if (!sendCANFrame(can_id, data, 8)) {
            ROS_WARN_THROTTLE(1.0, "Failed to send CAN frame for %s", 
                             joint_names_[i].c_str());
        }
    }
}

bool MarsRoverCANHW::sendCANFrame(uint32_t can_id, const uint8_t* data, uint8_t len) {
    struct can_frame frame;
    
    frame.can_id = can_id;
    frame.can_dlc = len;
    memcpy(frame.data, data, len);
    
    int nbytes = ::write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        ROS_DEBUG("CAN write error: %s", strerror(errno));
        return false;
    }
    
    return true;
}

bool MarsRoverCANHW::readCANFrame(struct can_frame& frame) {
    int nbytes = ::read(can_socket_, &frame, sizeof(struct can_frame));
    
    if (nbytes < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            ROS_DEBUG("CAN read error: %s", strerror(errno));
        }
        return false;
    }
    
    if (nbytes < sizeof(struct can_frame)) {
        ROS_WARN("Incomplete CAN frame received");
        return false;
    }
    
    return true;
}

} // namespace mars_rover_hw
