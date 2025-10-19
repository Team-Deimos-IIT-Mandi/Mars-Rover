#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math 

class DifferentialDriveController:
    def __init__(self):
        rospy.init_node('differential_drive_controller')
        self.left_wheel_omega_pub = rospy.Publisher('left_wheel_vel', Float64, queue_size=10)
        self.right_wheel_omega_pub = rospy.Publisher('right_wheel_vel', Float64, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        self.a = 0.34  # distance between front and rear wheels
        self.b = 0.60  # distance between left and right wheels
        self.r = 0.11  # radius of wheels
        
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate left and right wheel velocities
        if angular_z != 0:
            left_wheel_vel = angular_z * math.sqrt(math.pow((linear_x/angular_z)-(self.b/2), 2) + math.pow(self.a/2, 2)) / math.cos(math.atan2(self.a/2, linear_x/angular_z - self.b/2))
            right_wheel_vel = angular_z * math.sqrt(math.pow((linear_x/angular_z)+(self.b/2), 2) + math.pow(self.a/2, 2)) / math.cos(math.atan2(self.a/2, linear_x/angular_z + self.b/2))
        else:
            left_wheel_vel = linear_x
            right_wheel_vel = linear_x

        # Publish left and right wheel velocities
        self.left_wheel_omega_pub.publish(left_wheel_vel/self.r)
        self.right_wheel_omega_pub.publish(right_wheel_vel/self.r)

if __name__ == '__main__':
    try:
        controller = DifferentialDriveController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
