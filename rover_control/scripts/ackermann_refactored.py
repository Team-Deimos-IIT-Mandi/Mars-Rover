#!/usr/bin/env python3
"""
Simple ROS Node for converting Twist messages to Ackermann steering control.
This is a streamlined version that handles I2C communication with motor drivers 
and publishes steering commands via ROS topics.

Version: 2.0 (Refactored)
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from smbus3 import SMBus, i2c_msg
import struct
import math
import time
import sys
import traceback
from typing import Dict, List, Optional
import atexit


class HardwareConfiguration:
    """
    Configuration class containing hardware-specific constants and parameters.
    This centralizes all hardware-related configurations for easy modification.
    """
    
    # I2C Bus Configuration
    I2C_BUS_NUMBER = 1
    I2C_TIMEOUT = 5  # seconds
    I2C_RETRIES = 2
    I2C_COMMAND_DELAY = 0.01  # seconds between I2C commands
    
    # Motor Driver I2C Addresses
    DRIVER_ADDRESSES = {
        'FL': 0x34,  # Front Left
        'FR': 0x33,  # Front Right  
        'RL': 0x32,  # Rear Left
        'RR': 0x31   # Rear Right
    }
    
    # Steering Controller I2C Addresses
    STEERING_ADDRESSES = {
        'FR': 0x36,  # Front Right
        'FL': 0x37,  # Front Left
        'RR': 0x38,  # Rear Right (currently unused)
        'RL': 0x39   # Rear Left (currently unused)
    }
    
    # Control Signal Constants
    MOTOR_NEUTRAL = 127      # Neutral/stop value for motors
    MOTOR_MIN = 0           # Minimum motor value
    MOTOR_MAX = 255         # Maximum motor value
    STEERING_NEUTRAL = 90    # Neutral steering position (degrees)
    STEERING_MIN = 0        # Minimum steering angle (degrees)
    STEERING_MAX = 180      # Maximum steering angle (degrees)
    
    # Vehicle Physical Parameters (defaults)
    DEFAULT_WHEELBASE = 1.5      # meters
    DEFAULT_TRACK_WIDTH = 1.2    # meters  
    DEFAULT_MAX_STEERING_ANGLE = 0.6  # radians
    DEFAULT_MAX_SPEED = 0.1      # m/s
    DEFAULT_CONTROL_RATE = 10    # Hz


class I2CManager:
    """
    Manages I2C communication with motor drivers and steering controllers.
    Provides error handling and logging for all I2C operations.
    """
    
    def __init__(self, bus_number: int = HardwareConfiguration.I2C_BUS_NUMBER):
        """
        Initialize I2C manager with specified bus configuration.
        
        Args:
            bus_number: I2C bus number to use
            
        Raises:
            RuntimeError: If I2C bus initialization fails
        """
        self.bus_number = bus_number
        self.bus = None
        self._initialize_bus()
    
    def _initialize_bus(self) -> None:
        """
        Initialize the I2C bus with timeout and retry settings.
        
        Raises:
            RuntimeError: If bus initialization fails
        """
        try:
            rospy.loginfo(f"Initializing I2C bus {self.bus_number}")
            
            self.bus = SMBus(self.bus_number)
            self.bus.set_timeout(HardwareConfiguration.I2C_TIMEOUT)
            self.bus.set_retries(HardwareConfiguration.I2C_RETRIES)
            
            # Allow bus to stabilize
            time.sleep(1)
            
            rospy.loginfo("I2C bus initialized successfully")
            
        except Exception as e:
            error_msg = f"Failed to initialize I2C bus {self.bus_number}: {e}"
            rospy.logerr(error_msg)
            raise RuntimeError(error_msg)
    
    def send_float_format_data(self, address: int, value: int) -> bool:
        """
        Send data in float format (2 floats + 1 byte) to I2C device.
        This format is used for motor driver commands.
        
        Args:
            address: I2C device address (0x00-0xFF)
            value: Integer value to send (0-255)
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Validate inputs
            if not (0x00 <= address <= 0xFF):
                rospy.logwarn(f"Invalid I2C address: {hex(address)}")
                return False
                
            if not (0 <= value <= 255):
                rospy.logwarn(f"Value {value} out of range [0-255] for address {hex(address)}")
                value = max(0, min(255, value))  # Clamp to valid range
            
            # Pack data: two floats (0.0) + one byte (value)
            float1 = struct.pack('f', 0.0)
            float2 = struct.pack('f', 0.0)
            byte_value = struct.pack('B', value)
            data = float1 + float2 + byte_value
            
            # Send via I2C
            msg = i2c_msg.write(address, data)
            self.bus.i2c_rdwr(msg)
            
            rospy.logdebug(f"Sent float format data to {hex(address)}: {value}")
            return True
            
        except OSError as e:
            rospy.logwarn(f"I2C communication error at {hex(address)}: {e}")
            return False
        except Exception as e:
            rospy.logerr(f"Unexpected error sending data to {hex(address)}: {e}")
            return False
    
    def send_integer_data(self, address: int, value: int) -> bool:
        """
        Send integer data directly to I2C device.
        This format is used for steering controller commands.
        
        Args:
            address: I2C device address (0x00-0xFF)
            value: Integer value to send (0-255)
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Validate inputs
            if not (0x00 <= address <= 0xFF):
                rospy.logwarn(f"Invalid I2C address: {hex(address)}")
                return False
                
            if not (0 <= value <= 255):
                rospy.logwarn(f"Value {value} out of range [0-255] for address {hex(address)}")
                value = max(0, min(255, value))  # Clamp to valid range
            
            # Send integer data
            self.bus.i2c_wr(address, [value])
            
            rospy.logdebug(f"Sent integer data to {hex(address)}: {value}")
            return True
            
        except OSError as e:
            rospy.logwarn(f"I2C communication error at {hex(address)}: {e}")
            return False
        except Exception as e:
            rospy.logerr(f"Unexpected error sending integer data to {hex(address)}: {e}")
            return False
    
    def emergency_stop_all_motors(self) -> bool:
        """
        Send emergency stop command to all motor drivers.
        Sets all motors to neutral position (127).
        
        Returns:
            bool: True if all motors stopped successfully
        """
        rospy.logwarn("Emergency stop: Setting all motors to STOP")
        success = True
        
        for wheel, address in HardwareConfiguration.DRIVER_ADDRESSES.items():
            try:
                if not self.send_float_format_data(address, HardwareConfiguration.MOTOR_NEUTRAL):
                    rospy.logerr(f"Failed to stop motor {wheel} at {hex(address)}")
                    success = False
                else:
                    rospy.logdebug(f"Successfully stopped motor {wheel}")
                
                time.sleep(HardwareConfiguration.I2C_COMMAND_DELAY)
                
            except Exception as e:
                rospy.logerr(f"Exception stopping motor {wheel}: {e}")
                success = False
        
        return success
    
    def close(self) -> None:
        """
        Safely close the I2C bus connection.
        """
        if self.bus:
            try:
                self.bus.close()
                rospy.loginfo("I2C bus closed successfully")
            except Exception as e:
                rospy.logwarn(f"Error closing I2C bus: {e}")


class AckermannCalculator:
    """
    Handles Ackermann steering geometry calculations and motor speed computations.
    Converts twist commands to appropriate motor and steering values.
    """
    
    def __init__(self, wheelbase: float, max_steering_angle: float, max_speed: float):
        """
        Initialize Ackermann calculator with vehicle parameters.
        
        Args:
            wheelbase: Distance between front and rear axles (meters)
            max_steering_angle: Maximum steering angle (radians)
            max_speed: Maximum vehicle speed (m/s)
        """
        self.wheelbase = wheelbase
        self.max_steering_angle = max_steering_angle
        self.max_speed = max_speed
        
        rospy.loginfo(f"Ackermann calculator initialized - "
                     f"wheelbase: {wheelbase}m, "
                     f"max_steering: {math.degrees(max_steering_angle):.1f}°, "
                     f"max_speed: {max_speed}m/s")
    
    def calculate_steering_and_speed(self, linear_vel: float, angular_vel: float) -> Dict:
        """
        Calculate steering angle and motor speeds from twist velocities.
        
        Args:
            linear_vel: Forward/backward velocity (m/s)
            angular_vel: Rotational velocity (rad/s)
            
        Returns:
            Dict containing:
                - steering_angle_degrees: Steering angle in degrees (0-180)
                - motor_speeds: Dict of motor speeds for each wheel
                - steering_commands: Dict of steering commands for each wheel
        """
        try:
            # Handle stationary or straight-line motion
            if abs(angular_vel) < 1e-6 or abs(linear_vel) < 1e-6:
                return self._get_straight_motion_commands(linear_vel)
            
            # Calculate Ackermann steering geometry
            turning_radius = linear_vel / angular_vel
            steering_angle_rad = math.atan(self.wheelbase / turning_radius)
            
            # Clamp steering angle to physical limits
            steering_angle_rad = max(-self.max_steering_angle, 
                                   min(self.max_steering_angle, steering_angle_rad))
            
            # Convert to degrees with servo offset
            steering_angle_deg = int((steering_angle_rad * 180.0) / math.pi) + HardwareConfiguration.STEERING_NEUTRAL
            steering_angle_deg = max(HardwareConfiguration.STEERING_MIN, 
                                   min(HardwareConfiguration.STEERING_MAX, steering_angle_deg))
            
            # Calculate motor speeds
            motor_speeds = self._calculate_motor_speeds(linear_vel)
            
            # Prepare steering commands
            steering_commands = self._calculate_steering_commands(steering_angle_deg)
            
            rospy.logdebug(f"Calculated - Steering: {steering_angle_deg}°, "
                          f"Motor speeds: {motor_speeds}")
            
            return {
                'steering_angle_degrees': steering_angle_deg,
                'motor_speeds': motor_speeds,
                'steering_commands': steering_commands
            }
            
        except Exception as e:
            rospy.logerr(f"Error in steering calculation: {e}")
            return self._get_safe_default_commands()
    
    def _get_straight_motion_commands(self, linear_vel: float) -> Dict:
        """
        Generate commands for straight-line motion (no steering).
        
        Args:
            linear_vel: Forward/backward velocity
            
        Returns:
            Dict with straight-line motion commands
        """
        motor_speeds = self._calculate_motor_speeds(linear_vel)
        steering_commands = self._calculate_steering_commands(HardwareConfiguration.STEERING_NEUTRAL)
        
        return {
            'steering_angle_degrees': HardwareConfiguration.STEERING_NEUTRAL,
            'motor_speeds': motor_speeds,
            'steering_commands': steering_commands
        }
    
    def _calculate_motor_speeds(self, linear_vel: float) -> Dict[str, int]:
        """
        Calculate motor speed values for each wheel.
        
        Args:
            linear_vel: Linear velocity in m/s
            
        Returns:
            Dict mapping wheel names to motor speed values (0-255)
        """
        # Clamp velocity to limits
        clamped_vel = max(-self.max_speed, min(self.max_speed, linear_vel))
        
        # Convert to motor command value (127 = neutral, range 0-255)
        # Scaling: velocity range [-max_speed, +max_speed] -> [0, 255]
        speed_command = int((clamped_vel / 1.5) * 180) + HardwareConfiguration.MOTOR_NEUTRAL
        speed_command = max(HardwareConfiguration.MOTOR_MIN, 
                          min(HardwareConfiguration.MOTOR_MAX, speed_command))
        
        # Assign speeds to wheels (left wheels need inverted direction)
        motor_speeds = {}
        for wheel in HardwareConfiguration.DRIVER_ADDRESSES.keys():
            if wheel in ['RL', 'FL']:  # Left wheels - invert direction
                motor_speeds[wheel] = HardwareConfiguration.MOTOR_NEUTRAL - (speed_command - HardwareConfiguration.MOTOR_NEUTRAL)
                motor_speeds[wheel] = max(HardwareConfiguration.MOTOR_MIN,
                                        min(HardwareConfiguration.MOTOR_MAX, motor_speeds[wheel]))
            else:  # Right wheels - normal direction
                motor_speeds[wheel] = speed_command
        
        return motor_speeds
    
    def _calculate_steering_commands(self, steering_angle_deg: int) -> Dict[str, int]:
        """
        Calculate steering commands for each wheel.
        
        Args:
            steering_angle_deg: Steering angle in degrees (0-180)
            
        Returns:
            Dict mapping wheel names to steering commands
        """
        return {
            'FL': steering_angle_deg,
            'FR': steering_angle_deg,
            'RR': steering_angle_deg,
            'RL': 180 - steering_angle_deg  # Rear left wheel inverted
        }
    
    def _get_safe_default_commands(self) -> Dict:
        """
        Get safe default commands in case of calculation errors.
        
        Returns:
            Dict with safe default values (stopped, straight)
        """
        return {
            'steering_angle_degrees': HardwareConfiguration.STEERING_NEUTRAL,
            'motor_speeds': {wheel: HardwareConfiguration.MOTOR_NEUTRAL 
                           for wheel in HardwareConfiguration.DRIVER_ADDRESSES.keys()},
            'steering_commands': {wheel: HardwareConfiguration.STEERING_NEUTRAL 
                                for wheel in ['FL', 'FR', 'RR', 'RL']}
        }


class SimpleTwistToAckermann:
    """
    Main ROS node class for converting Twist messages to Ackermann control.
    Handles ROS communication, I2C hardware control, and coordinate everything.
    """
    
    def __init__(self):
        """
        Initialize the ROS node and all its components.
        
        Raises:
            SystemExit: If initialization fails critically
        """
        try:
            rospy.loginfo("Initializing Simple Twist to Ackermann Node...")
            
            # Initialize ROS node
            rospy.init_node('simple_twist_to_ackermann', anonymous=True)
            
            # Load configuration parameters
            self._load_parameters()
            
            # Initialize hardware manager
            self.i2c_manager = I2CManager()
            
            # Initialize Ackermann calculator
            self.ackermann_calc = AckermannCalculator(
                self.wheelbase, 
                self.max_steering_angle, 
                self.max_speed
            )
            
            # Setup ROS interface
            self._setup_ros_interface()
            
            # Register shutdown hook
            rospy.on_shutdown(self._shutdown_callback)
            atexit.register(self._emergency_shutdown)
            
            rospy.loginfo("Simple Twist to Ackermann Node initialized successfully")
            
        except Exception as e:
            rospy.logerr(f"Critical error during initialization: {e}")
            rospy.logerr(traceback.format_exc())
            sys.exit(1)
    
    def _load_parameters(self) -> None:
        """
        Load configuration parameters from ROS parameter server.
        Uses default values if parameters are not available.
        """
        try:
            self.wheelbase = rospy.get_param('~wheelbase', HardwareConfiguration.DEFAULT_WHEELBASE)
            self.track_width = rospy.get_param('~track_width', HardwareConfiguration.DEFAULT_TRACK_WIDTH)
            self.max_steering_angle = rospy.get_param('~max_steering_angle', HardwareConfiguration.DEFAULT_MAX_STEERING_ANGLE)
            self.max_speed = rospy.get_param('~max_speed', HardwareConfiguration.DEFAULT_MAX_SPEED)
            self.control_rate = rospy.get_param('~control_rate', HardwareConfiguration.DEFAULT_CONTROL_RATE)
            
            # Input validation
            if self.wheelbase <= 0:
                rospy.logwarn(f"Invalid wheelbase {self.wheelbase}, using default")
                self.wheelbase = HardwareConfiguration.DEFAULT_WHEELBASE
                
            if self.max_steering_angle <= 0:
                rospy.logwarn(f"Invalid max_steering_angle {self.max_steering_angle}, using default")
                self.max_steering_angle = HardwareConfiguration.DEFAULT_MAX_STEERING_ANGLE
            
            rospy.loginfo(f"Parameters loaded - wheelbase: {self.wheelbase}m, "
                         f"max_steering: {math.degrees(self.max_steering_angle):.1f}°, "
                         f"max_speed: {self.max_speed}m/s, "
                         f"rate: {self.control_rate}Hz")
                         
        except Exception as e:
            rospy.logwarn(f"Error loading parameters, using defaults: {e}")
            self.wheelbase = HardwareConfiguration.DEFAULT_WHEELBASE
            self.track_width = HardwareConfiguration.DEFAULT_TRACK_WIDTH
            self.max_steering_angle = HardwareConfiguration.DEFAULT_MAX_STEERING_ANGLE
            self.max_speed = HardwareConfiguration.DEFAULT_MAX_SPEED
            self.control_rate = HardwareConfiguration.DEFAULT_CONTROL_RATE
    
    def _setup_ros_interface(self) -> None:
        """
        Setup ROS publishers and subscribers.
        
        Raises:
            RuntimeError: If ROS interface setup fails
        """
        try:
            # Create rate controller
            self.rate = rospy.Rate(self.control_rate)
            
            # Create steering angle publishers
            self.steering_publishers = {
                'FL': rospy.Publisher('/steer_FL', UInt8, queue_size=1),
                'FR': rospy.Publisher('/steer_FR', UInt8, queue_size=1),
                'RR': rospy.Publisher('/steer_RR', UInt8, queue_size=1),
                'RL': rospy.Publisher('/steer_RL', UInt8, queue_size=1)
            }
            
            # Create twist command subscriber
            self.cmd_vel_sub = rospy.Subscriber(
                '/kb_teleop/cmd_vel', 
                Twist, 
                self._twist_callback,
                queue_size=1
            )
            
            rospy.loginfo("ROS interface setup completed")
            
        except Exception as e:
            error_msg = f"Failed to setup ROS interface: {e}"
            rospy.logerr(error_msg)
            raise RuntimeError(error_msg)
    
    def _twist_callback(self, msg: Twist) -> None:
        """
        Callback function for processing incoming Twist messages.
        Converts twist to Ackermann control and executes commands.
        
        Args:
            msg: Twist message with linear and angular velocities
        """
        try:
            # Extract velocities
            linear_vel = msg.linear.x
            angular_vel = msg.angular.z
            
            rospy.logdebug(f"Received twist - linear: {linear_vel:.3f}, angular: {angular_vel:.3f}")
            
            # Calculate control commands
            control_data = self.ackermann_calc.calculate_steering_and_speed(linear_vel, angular_vel)
            
            # Execute motor commands via I2C
            self._execute_motor_commands(control_data['motor_speeds'])
            
            # Publish steering commands via ROS
            self._publish_steering_commands(control_data['steering_commands'])
            
        except Exception as e:
            rospy.logerr(f"Error in twist callback: {e}")
            rospy.logerr(traceback.format_exc())
            # Emergency stop on error
            self._emergency_stop()
    
    def _execute_motor_commands(self, motor_speeds: Dict[str, int]) -> None:
        """
        Execute motor speed commands via I2C.
        
        Args:
            motor_speeds: Dictionary mapping wheel names to speed values
        """
        for wheel, address in HardwareConfiguration.DRIVER_ADDRESSES.items():
            try:
                speed = motor_speeds.get(wheel, HardwareConfiguration.MOTOR_NEUTRAL)
                
                if not self.i2c_manager.send_float_format_data(address, speed):
                    rospy.logwarn(f"Failed to send motor command to {wheel}")
                
                time.sleep(HardwareConfiguration.I2C_COMMAND_DELAY)
                
            except Exception as e:
                rospy.logerr(f"Exception sending motor command to {wheel}: {e}")
    
    def _publish_steering_commands(self, steering_commands: Dict[str, int]) -> None:
        """
        Publish steering angle commands to ROS topics.
        
        Args:
            steering_commands: Dictionary mapping wheel names to steering angles
        """
        try:
            for wheel, angle in steering_commands.items():
                if wheel in self.steering_publishers:
                    msg = UInt8()
                    msg.data = max(HardwareConfiguration.STEERING_MIN, 
                                 min(HardwareConfiguration.STEERING_MAX, angle))
                    self.steering_publishers[wheel].publish(msg)
                    
        except Exception as e:
            rospy.logerr(f"Error publishing steering commands: {e}")
    
    def _emergency_stop(self) -> None:
        """
        Execute emergency stop procedure.
        """
        rospy.logwarn("Executing emergency stop")
        try:
            self.i2c_manager.emergency_stop_all_motors()
        except Exception as e:
            rospy.logerr(f"Error during emergency stop: {e}")
    
    def _shutdown_callback(self) -> None:
        """
        Clean shutdown procedure called by ROS.
        """
        rospy.loginfo("Shutting down Simple Twist to Ackermann Node...")
        self._emergency_stop()
        time.sleep(0.1)  # Allow commands to complete
        self.i2c_manager.close()
        rospy.loginfo("Shutdown completed")
    
    def _emergency_shutdown(self) -> None:
        """
        Emergency shutdown called by atexit.
        """
        try:
            if hasattr(self, 'i2c_manager'):
                self.i2c_manager.emergency_stop_all_motors()
        except:
            pass  # Ignore errors during emergency shutdown
    
    def run(self) -> None:
        """
        Main execution loop for the ROS node.
        Keeps the node alive and responsive.
        """
        try:
            rospy.loginfo("Starting main execution loop...")
            
            while not rospy.is_shutdown():
                rospy.spin()
                
        except KeyboardInterrupt:
            rospy.loginfo("Keyboard interrupt received")
        except Exception as e:
            rospy.logerr(f"Error in main loop: {e}")
            rospy.logerr(traceback.format_exc())
        finally:
            self._shutdown_callback()


def main():
    """
    Main entry point for the Simple Twist to Ackermann Node.
    """
    try:
        # Create and run the node
        node = SimpleTwistToAckermann()
        node.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt exception")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt")
    except SystemExit:
        rospy.loginfo("System exit")
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
        rospy.logerr(traceback.format_exc())
    finally:
        rospy.loginfo("Node execution finished")


if __name__ == '__main__':
    main()