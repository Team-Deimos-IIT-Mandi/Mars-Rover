#!/usr/bin/env python3
import smbus2
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math
import struct
import sys

# Define I2C constants
ADD1 = 0x31
ADD2 = 0x32
ADD3 = 0x33
ADD4 = 0x34
I2C_BUS = 1

# Initialize the I2C bus object.
try:
    bus = smbus2.SMBus(I2C_BUS)
except FileNotFoundError:
    rospy.logerr(f"I2C bus not found at /dev/i2c-{I2C_BUS}. Please check your hardware configuration.")
    sys.exit(1)
except Exception as e:
    rospy.logerr(f"An error occurred while initializing the I2C bus: {e}")
    sys.exit(1)

def send_data(vf1, vf2, vi1, add):
    """
    Packs and sends data to a specific I2C address.

    Args:
        vf1 (float): First float value to send.
        vf2 (float): Second float value to send.
        vi1 (int): Integer value to send.
        add (int): The I2C address of the device.
    
    This function packs the data into a byte array and sends it over the I2C bus.
    It uses a try-except block to handle potential communication errors.
    """
    try:
        n1 = struct.pack('f', vf1)
        n2 = struct.pack('f', vf2)
        int1 = struct.pack('B', vi1)
        data = n1 + n2 + int1
        
        write_msg = smbus2.i2c_msg.write(add, data)
        bus.i2c_rdwr(write_msg)
        
    except IOError as e:
        rospy.logerr(f"I/O error communicating with I2C device at address {hex(add)}: {e}")
    except struct.error as e:
        rospy.logerr(f"Data packing error: {e}")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")

class DifferentialDriveController:
    """
    ROS node for controlling a differential drive robot via I2C.
    
    This class handles ROS topic subscriptions, performs kinematic calculations,
    and sends motor commands over the I2C bus to four different motor controllers.
    """
    def __init__(self):
        """
        Initializes the ROS node, publishers, subscribers, and robot parameters.
        """
        rospy.init_node('differential_drive_controller', anonymous=True)
        
        self.left_wheel_omega_pub = rospy.Publisher('left_omega', Float64, queue_size=10)
        self.right_wheel_omega_pub = rospy.Publisher('right_omega', Float64, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/kb_teleop/cmd_vel', Twist, self.cmd_vel_callback)
        
        self.a = 0.34
        self.b = 1.0
        self.r = 0.11

    def cmd_vel_callback(self, msg):
        """
        Callback function for the /kb_teleop/cmd_vel topic.

        Args:
            msg (Twist): The received ROS Twist message containing linear and
                         angular velocity commands.
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if angular_z != 0:
            try:
                left_wheel_vel = angular_z * math.sqrt(math.pow((linear_x / angular_z) + (self.b / 2), 2) + math.pow(self.a / 2, 2)) / math.cos(math.atan2(self.a / 2, linear_x / angular_z - self.b / 2))
                right_wheel_vel = angular_z * math.sqrt(math.pow((linear_x / angular_z) - (self.b / 2), 2) + math.pow(self.a / 2, 2)) / math.cos(math.atan2(self.a / 2, linear_x / angular_z + self.b / 2))
            except ZeroDivisionError:
                rospy.logwarn("ZeroDivisionError encountered in kinematic calculation. Defaulting to linear movement.")
                left_wheel_vel = linear_x
                right_wheel_vel = linear_x
        else:
            left_wheel_vel = linear_x
            right_wheel_vel = linear_x

        right_i2c = 127 + int(30 * (linear_x if angular_z >= 0 else -linear_x))
        left_i2c = 127 + int(30 * (linear_x if angular_z <= 0 else -linear_x))
        
        send_data(0.0, 0.0, right_i2c, ADD1)
        send_data(0.0, 0.0, right_i2c, ADD3)
        send_data(0.0, 0.0, left_i2c, ADD2)
        send_data(0.0, 0.0, left_i2c, ADD4)

        self.left_wheel_omega_pub.publish(left_i2c)
        self.right_wheel_omega_pub.publish(right_i2c)

if __name__ == '__main__':
    try:
        controller = DifferentialDriveController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted. Shutting down.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred in the main loop: {e}")