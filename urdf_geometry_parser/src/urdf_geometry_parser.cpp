#include <urdf_geometry_parser/urdf_geometry_parser.h>
#include <cmath>
#include <stdexcept>
#include <sstream>

/**
 * @file urdf_geometry_parser.cpp
 * @brief Implementation of URDF geometry parser for extracting geometric properties from robot models
 */

namespace urdf_geometry_parser {

/**
 * @namespace Constants
 * @brief Mathematical and conversion constants
 */
namespace Constants {
    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    constexpr double EPSILON = 1e-9;
}

/**
 * @namespace ValidationUtils
 * @brief Utility functions for validating URDF components
 */
namespace ValidationUtils {
    
    /**
     * @brief Validate if a link pointer is valid and not null
     * @param link Shared pointer to URDF link
     * @param context Context string for error reporting
     * @return true if link is valid, false otherwise
     */
    bool isLinkValid(const urdf::LinkConstSharedPtr& link, const std::string& context = "") {
        if (!link) {
            ROS_ERROR_STREAM("Link is NULL" << (context.empty() ? "" : " in context: " + context));
            return false;
        }
        return true;
    }
    
    /**
     * @brief Validate if a joint pointer is valid and not null
     * @param joint Shared pointer to URDF joint
     * @param context Context string for error reporting
     * @return true if joint is valid, false otherwise
     */
    bool isJointValid(const urdf::JointConstSharedPtr& joint, const std::string& context = "") {
        if (!joint) {
            ROS_ERROR_STREAM("Joint is NULL" << (context.empty() ? "" : " in context: " + context));
            return false;
        }
        return true;
    }
    
    /**
     * @brief Validate collision geometry exists for a link
     * @param link Shared pointer to URDF link
     * @return true if collision geometry exists, false otherwise
     */
    bool hasValidCollisionGeometry(const urdf::LinkConstSharedPtr& link) {
        if (!isLinkValid(link, "collision geometry validation")) {
            return false;
        }
        
        if (!link->collision) {
            ROS_ERROR_STREAM("Link '" << link->name 
                           << "' does not have collision description. "
                           << "Add collision description for link to URDF.");
            return false;
        }
        
        if (!link->collision->geometry) {
            ROS_ERROR_STREAM("Link '" << link->name 
                           << "' does not have collision geometry description. "
                           << "Add collision geometry description for link to URDF.");
            return false;
        }
        
        return true;
    }
}

/**
 * @namespace GeometryCheckers
 * @brief Functions for checking specific geometry types
 */
namespace GeometryCheckers {
    
    /**
     * @brief Check if the link is modeled as a cylinder
     * @param link Shared pointer to URDF link
     * @return true if the link is modeled as a cylinder; false otherwise
     */
    bool isCylinder(const urdf::LinkConstSharedPtr& link) {
        if (!ValidationUtils::hasValidCollisionGeometry(link)) {
            return false;
        }
        
        if (link->collision->geometry->type != urdf::Geometry::CYLINDER) {
            ROS_ERROR_STREAM("Link '" << link->name << "' does not have cylinder geometry. "
                           << "Current geometry type: " << link->collision->geometry->type);
            return false;
        }
        
        ROS_DEBUG_STREAM("Link '" << link->name << "' confirmed as cylinder geometry");
        return true;
    }
}

/**
 * @namespace GeometryExtractors
 * @brief Functions for extracting geometric properties
 */
namespace GeometryExtractors {
    
    /**
     * @brief Extract wheel radius from a cylindrical wheel link
     * @param wheel_link Shared pointer to wheel link
     * @param wheel_radius [out] Extracted wheel radius in meters
     * @return true if wheel radius was successfully extracted; false otherwise
     */
    bool extractWheelRadius(const urdf::LinkConstSharedPtr& wheel_link, double& wheel_radius) {
        try {
            if (!GeometryCheckers::isCylinder(wheel_link)) {
                ROS_ERROR_STREAM("Wheel link '" << wheel_link->name 
                               << "' is NOT modeled as a cylinder!");
                return false;
            }
            
            // Safely cast to cylinder geometry
            auto cylinder_geometry = static_cast<urdf::Cylinder*>(
                wheel_link->collision->geometry.get()
            );
            
            if (!cylinder_geometry) {
                ROS_ERROR_STREAM("Failed to cast geometry to cylinder for link '" 
                               << wheel_link->name << "'");
                return false;
            }
            
            wheel_radius = cylinder_geometry->radius;
            
            // Validate radius is positive
            if (wheel_radius <= 0.0) {
                ROS_ERROR_STREAM("Invalid wheel radius (" << wheel_radius 
                               << ") for link '" << wheel_link->name 
                               << "'. Radius must be positive.");
                return false;
            }
            
            ROS_DEBUG_STREAM("Successfully extracted wheel radius: " << wheel_radius 
                           << "m for link '" << wheel_link->name << "'");
            return true;
            
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Exception while extracting wheel radius for link '" 
                           << (wheel_link ? wheel_link->name : "NULL") << "': " << e.what());
            return false;
        }
    }
}

/**
 * @brief Constructor for UrdfGeometryParser
 * @param root_nh ROS node handle for parameter access
 * @param base_link Name of the base link for the robot
 * @throws std::runtime_error if URDF parsing fails
 */
UrdfGeometryParser::UrdfGeometryParser(ros::NodeHandle& root_nh, const std::string& base_link)
    : base_link_(base_link), model_(nullptr) {
    
    try {
        ROS_INFO_STREAM("Initializing URDF Geometry Parser with base link: '" << base_link << "'");
        
        if (!initializeRobotModel(root_nh)) {
            throw std::runtime_error("Failed to initialize robot model from URDF");
        }
        
        ROS_INFO("URDF Geometry Parser initialized successfully");
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to construct UrdfGeometryParser: " << e.what());
        throw;
    }
}

/**
 * @brief Initialize the robot model from ROS parameter server
 * @param root_nh ROS node handle
 * @return true if successful, false otherwise
 */
bool UrdfGeometryParser::initializeRobotModel(ros::NodeHandle& root_nh) {
    const std::string model_param_name = "robot_description";
    
    try {
        // Check if parameter exists
        if (!root_nh.hasParam(model_param_name)) {
            ROS_ERROR_STREAM("Robot description parameter '" << model_param_name 
                           << "' not found on parameter server");
            return false;
        }
        
        // Retrieve robot description string
        std::string robot_model_str;
        if (!root_nh.getParam(model_param_name, robot_model_str)) {
            ROS_ERROR_STREAM("Failed to retrieve robot description from parameter '" 
                           << model_param_name << "'");
            return false;
        }
        
        // Validate robot description is not empty
        if (robot_model_str.empty()) {
            ROS_ERROR_STREAM("Robot description parameter '" << model_param_name 
                           << "' is empty");
            return false;
        }
        
        // Parse URDF model
        model_ = urdf::parseURDF(robot_model_str);
        if (!model_) {
            ROS_ERROR_STREAM("Could not parse the URDF robot model from parameter '" 
                           << model_param_name << "'");
            return false;
        }
        
        // Validate base link exists in model
        if (!model_->getLink(base_link_)) {
            ROS_ERROR_STREAM("Base link '" << base_link_ << "' not found in URDF model");
            return false;
        }
        
        ROS_INFO_STREAM("Successfully parsed URDF model with " << model_->links_.size() 
                       << " links and " << model_->joints_.size() << " joints");
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception while initializing robot model: " << e.what());
        return false;
    }
}

/**
 * @brief Get the transform vector from a joint to a specified parent link
 * @param joint_name Name of the target joint
 * @param parent_link_name Name of the parent link to transform to
 * @param transform_vector [out] Resulting transform vector
 * @return true if successful, false otherwise
 */
bool UrdfGeometryParser::getTransformVector(const std::string& joint_name, 
                                           const std::string& parent_link_name,
                                           urdf::Vector3& transform_vector) {
    try {
        // Validate model exists
        if (!model_) {
            ROS_ERROR("URDF model is not initialized");
            return false;
        }
        
        // Validate input parameters
        if (joint_name.empty() || parent_link_name.empty()) {
            ROS_ERROR_STREAM("Invalid input parameters: joint_name='" << joint_name 
                           << "', parent_link_name='" << parent_link_name << "'");
            return false;
        }
        
        // Get initial joint
        urdf::JointConstSharedPtr joint = model_->getJoint(joint_name);
        if (!ValidationUtils::isJointValid(joint, "getTransformVector")) {
            ROS_ERROR_STREAM("Joint '" << joint_name 
                           << "' could not be retrieved from model description");
            return false;
        }
        
        // Initialize transform vector with joint's origin
        transform_vector = joint->parent_to_joint_origin_transform.position;
        ROS_DEBUG_STREAM("Starting transform calculation from joint '" << joint_name 
                        << "' to parent link '" << parent_link_name << "'");
        
        // Traverse up the kinematic tree until we reach the desired parent link
        std::set<std::string> visited_links; // Prevent infinite loops
        while (joint->parent_link_name != parent_link_name) {
            
            // Check for circular references
            if (visited_links.count(joint->parent_link_name) > 0) {
                ROS_ERROR_STREAM("Circular reference detected in kinematic chain while traversing from '"
                               << joint_name << "' to '" << parent_link_name << "'");
                return false;
            }
            visited_links.insert(joint->parent_link_name);
            
            // Get parent link
            urdf::LinkConstSharedPtr link_parent = model_->getLink(joint->parent_link_name);
            if (!ValidationUtils::isLinkValid(link_parent, "transform vector calculation")) {
                ROS_ERROR_STREAM("Parent link '" << joint->parent_link_name 
                               << "' could not be retrieved from model description");
                return false;
            }
            
            // Check if parent link has a parent joint
            if (!link_parent->parent_joint) {
                ROS_ERROR_STREAM("Link '" << joint->parent_link_name 
                               << "' does not have a parent joint. Cannot reach parent link '" 
                               << parent_link_name << "'");
                return false;
            }
            
            // Move to parent joint and accumulate transform
            joint = link_parent->parent_joint;
            transform_vector = transform_vector + joint->parent_to_joint_origin_transform.position;
            
            ROS_DEBUG_STREAM("Accumulated transform through joint '" << joint->name 
                           << "' with parent link '" << joint->parent_link_name << "'");
        }
        
        ROS_DEBUG_STREAM("Successfully calculated transform vector: [" 
                        << transform_vector.x << ", " << transform_vector.y 
                        << ", " << transform_vector.z << "]");
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception in getTransformVector: " << e.what());
        return false;
    }
}

/**
 * @brief Calculate Euclidean distance between two joints
 * @param first_joint_name Name of the first joint
 * @param second_joint_name Name of the second joint
 * @param distance [out] Calculated distance in meters
 * @return true if successful, false otherwise
 */
bool UrdfGeometryParser::getDistanceBetweenJoints(const std::string& first_joint_name,
                                                 const std::string& second_joint_name,
                                                 double& distance) {
    try {
        // Validate input parameters
        if (first_joint_name.empty() || second_joint_name.empty()) {
            ROS_ERROR_STREAM("Invalid joint names provided: first='" << first_joint_name 
                           << "', second='" << second_joint_name << "'");
            return false;
        }
        
        if (first_joint_name == second_joint_name) {
            ROS_WARN_STREAM("Distance calculation between identical joints: '" 
                          << first_joint_name << "'");
            distance = 0.0;
            return true;
        }
        
        // Get transform vectors for both joints
        urdf::Vector3 first_transform;
        if (!getTransformVector(first_joint_name, base_link_, first_transform)) {
            ROS_ERROR_STREAM("Failed to get transform vector for first joint '" 
                           << first_joint_name << "'");
            return false;
        }
        
        urdf::Vector3 second_transform;
        if (!getTransformVector(second_joint_name, base_link_, second_transform)) {
            ROS_ERROR_STREAM("Failed to get transform vector for second joint '" 
                           << second_joint_name << "'");
            return false;
        }
        
        // Calculate Euclidean distance using the Pythagorean theorem
        const double dx = first_transform.x - second_transform.x;
        const double dy = first_transform.y - second_transform.y;
        const double dz = first_transform.z - second_transform.z;
        
        distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        // Validate result
        if (distance < 0.0 || std::isnan(distance) || std::isinf(distance)) {
            ROS_ERROR_STREAM("Invalid distance calculated: " << distance);
            return false;
        }
        
        ROS_DEBUG_STREAM("Distance between joints '" << first_joint_name 
                        << "' and '" << second_joint_name << "': " << distance << "m");
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception in getDistanceBetweenJoints: " << e.what());
        return false;
    }
}

/**
 * @brief Get the radius of a wheel associated with a joint
 * @param joint_name Name of the joint
 * @param radius [out] Wheel radius in meters
 * @return true if successful, false otherwise
 */
bool UrdfGeometryParser::getJointRadius(const std::string& joint_name, double& radius) {
    try {
        // Validate model and input
        if (!model_) {
            ROS_ERROR("URDF model is not initialized");
            return false;
        }
        
        if (joint_name.empty()) {
            ROS_ERROR("Joint name cannot be empty");
            return false;
        }
        
        // Get joint from model
        urdf::JointConstSharedPtr joint = model_->getJoint(joint_name);
        if (!ValidationUtils::isJointValid(joint, "radius extraction")) {
            ROS_ERROR_STREAM("Joint '" << joint_name 
                           << "' could not be retrieved from model description");
            return false;
        }
        
        // Get child link (wheel link)
        urdf::LinkConstSharedPtr child_link = model_->getLink(joint->child_link_name);
        if (!ValidationUtils::isLinkValid(child_link, "wheel radius extraction")) {
            ROS_ERROR_STREAM("Child link '" << joint->child_link_name 
                           << "' for joint '" << joint_name 
                           << "' could not be retrieved from model description");
            return false;
        }
        
        // Extract wheel radius using geometry extractor
        if (!GeometryExtractors::extractWheelRadius(child_link, radius)) {
            ROS_ERROR_STREAM("Could not retrieve wheel radius for joint '" 
                           << joint_name << "' (child link: '" 
                           << joint->child_link_name << "')");
            return false;
        }
        
        ROS_DEBUG_STREAM("Successfully extracted radius " << radius 
                        << "m for joint '" << joint_name << "'");
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception in getJointRadius: " << e.what());
        return false;
    }
}

/**
 * @brief Get steering limits for a revolute joint
 * @param joint_name Name of the joint
 * @param steering_limit [out] Steering limit in radians (absolute value)
 * @return true if successful, false otherwise
 */
bool UrdfGeometryParser::getJointSteeringLimits(const std::string& joint_name,
                                               double& steering_limit) {
    try {
        // Validate model and input
        if (!model_) {
            ROS_ERROR("URDF model is not initialized");
            return false;
        }
        
        if (joint_name.empty()) {
            ROS_ERROR("Joint name cannot be empty");
            return false;
        }
        
        // Get joint from model
        urdf::JointConstSharedPtr joint = model_->getJoint(joint_name);
        if (!ValidationUtils::isJointValid(joint, "steering limits extraction")) {
            ROS_ERROR_STREAM("Joint '" << joint_name 
                           << "' could not be retrieved from model description");
            return false;
        }
        
        // Validate joint type
        if (joint->type != urdf::Joint::REVOLUTE) {
            ROS_ERROR_STREAM("Joint '" << joint_name 
                           << "' is not of type REVOLUTE. Current type: " << joint->type
                           << ". Steering limits can only be extracted from revolute joints.");
            return false;
        }
        
        // Validate joint limits exist
        if (!joint->limits) {
            ROS_ERROR_STREAM("Joint '" << joint_name 
                           << "' does not have limits defined. "
                           << "Add joint limits to URDF for revolute joints.");
            return false;
        }
        
        // Calculate steering limits
        const double lower_limit = std::abs(joint->limits->lower);
        const double upper_limit = std::abs(joint->limits->upper);
        
        // Use the smaller absolute limit as the steering constraint
        steering_limit = std::min(lower_limit, upper_limit);
        
        // Validate steering limit
        if (steering_limit < Constants::EPSILON) {
            ROS_WARN_STREAM("Very small steering limit (" << steering_limit 
                          << " rad) for joint '" << joint_name << "'");
        }
        
        if (steering_limit > M_PI) {
            ROS_WARN_STREAM("Large steering limit (" << steering_limit 
                          << " rad, " << steering_limit * Constants::RAD_TO_DEG 
                          << " deg) for joint '" << joint_name 
                          << "'. This may not be physically realistic for steering.");
        }
        
        ROS_DEBUG_STREAM("Joint '" << joint_name << "' steering limit: " 
                        << steering_limit << " rad (" 
                        << steering_limit * Constants::RAD_TO_DEG << " deg)");
        
        ROS_DEBUG_STREAM("Original joint limits - Lower: " << joint->limits->lower 
                        << " rad, Upper: " << joint->limits->upper << " rad");
        
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception in getJointSteeringLimits: " << e.what());
        return false;
    }
}

/**
 * @brief Check if the URDF model is properly initialized
 * @return true if model is valid, false otherwise
 */
bool UrdfGeometryParser::isModelValid() const {
    return (model_ != nullptr);
}

/**
 * @brief Get the base link name
 * @return Base link name as string
 */
std::string UrdfGeometryParser::getBaseLinkName() const {
    return base_link_;
}

/**
 * @brief Get the number of links in the URDF model
 * @return Number of links, or 0 if model is invalid
 */
size_t UrdfGeometryParser::getNumLinks() const {
    return model_ ? model_->links_.size() : 0;
}

/**
 * @brief Get the number of joints in the URDF model
 * @return Number of joints, or 0 if model is invalid
 */
size_t UrdfGeometryParser::getNumJoints() const {
    return model_ ? model_->joints_.size() : 0;
}

} // namespace urdf_geometry_parser