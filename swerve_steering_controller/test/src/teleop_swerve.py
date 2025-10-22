#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class TeleopSwerve:
    def __init__(self):
        rospy.init_node('teleop_swerve')
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher(
            '/swerve_steering_controller/cmd_vel', 
            Twist, 
            queue_size=1
        )
        
        # Speed parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.linear_increment = 0.1
        self.angular_increment = 0.1
        
        # Instructions
        self.msg = """
Reading from the keyboard and Publishing to /swerve_steering_controller/cmd_vel!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease linear speed by 10%
w/x : increase/decrease angular speed by 10%

i : forward
, : backward
j : turn left
l : turn right
u : forward + left
o : forward + right
m : backward + left
. : backward + right
k : stop

CTRL-C to quit
"""
        
        print(self.msg)
        print("Current speeds - Linear: %.2f m/s, Angular: %.2f rad/s" % 
              (self.linear_speed, self.angular_speed))

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                twist = Twist()
                
                if key == 'i':  # Forward
                    twist.linear.x = self.linear_speed
                elif key == ',':  # Backward
                    twist.linear.x = -self.linear_speed
                elif key == 'j':  # Turn left
                    twist.angular.z = self.angular_speed
                elif key == 'l':  # Turn right
                    twist.angular.z = -self.angular_speed
                elif key == 'u':  # Forward + left
                    twist.linear.x = self.linear_speed
                    twist.angular.z = self.angular_speed
                elif key == 'o':  # Forward + right
                    twist.linear.x = self.linear_speed
                    twist.angular.z = -self.angular_speed
                elif key == 'm':  # Backward + left
                    twist.linear.x = -self.linear_speed
                    twist.angular.z = self.angular_speed
                elif key == '.':  # Backward + right
                    twist.linear.x = -self.linear_speed
                    twist.angular.z = -self.angular_speed
                elif key == 'k':  # Stop
                    twist.linear.x = 0
                    twist.angular.z = 0
                elif key == 'q':  # Increase linear speed
                    self.linear_speed += self.linear_increment
                    print("Linear speed: %.2f m/s" % self.linear_speed)
                    continue
                elif key == 'z':  # Decrease linear speed
                    self.linear_speed = max(0, self.linear_speed - self.linear_increment)
                    print("Linear speed: %.2f m/s" % self.linear_speed)
                    continue
                elif key == 'w':  # Increase angular speed
                    self.angular_speed += self.angular_increment
                    print("Angular speed: %.2f rad/s" % self.angular_speed)
                    continue
                elif key == 'x':  # Decrease angular speed
                    self.angular_speed = max(0, self.angular_speed - self.angular_increment)
                    print("Angular speed: %.2f rad/s" % self.angular_speed)
                    continue
                elif key == '\x03':  # Ctrl+C
                    break
                
                self.cmd_vel_pub.publish(twist)
                
        except Exception as e:
            print(e)
        finally:
            # Send stop command
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    try:
        teleop = TeleopSwerve()
        teleop.run()
    except rospy.ROSInterruptException:
        pass

