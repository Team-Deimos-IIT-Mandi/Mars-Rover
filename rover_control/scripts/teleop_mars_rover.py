#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class TeleopMarsRover:
    def __init__(self):
        rospy.init_node('teleop_mars_rover', anonymous=False)
        
        # Get parameters
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/mars_rover/cmd_vel')
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher(
            self.cmd_vel_topic, 
            Twist, 
            queue_size=1
        )
        
        # Speed parameters
        self.linear_speed = 0.3      # m/s - conservative start
        self.angular_speed = 0.5     # rad/s
        self.linear_increment = 0.1
        self.angular_increment = 0.1
        self.max_linear_speed = 1.5
        self.max_angular_speed = 2.0
        
        # Current command
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Instructions
        self.msg = """
╔═══════════════════════════════════════════════════════════════╗
║           Mars Rover Swerve Drive Teleop Control             ║
╚═══════════════════════════════════════════════════════════════╝

Publishing to: {topic}

Movement Controls (Use keyboard):
   u    i    o
   j    k    l
   m    ,    .

Key Functions:
─────────────────────────────────────────────────────────────────
  i : Move Forward          u : Forward + Turn Left
  , : Move Backward         o : Forward + Turn Right
  j : Rotate Left           m : Backward + Turn Left  
  l : Rotate Right          . : Backward + Turn Right
  k : STOP (Emergency)
  
Speed Adjustment:
─────────────────────────────────────────────────────────────────
  q : Increase linear speed by 0.1 m/s
  z : Decrease linear speed by 0.1 m/s
  w : Increase angular speed by 0.1 rad/s
  x : Decrease angular speed by 0.1 rad/s
  
  SPACE : Emergency Stop
  CTRL-C : Quit

Current Settings:
  Linear Speed:  {linear:.2f} m/s (max: {max_linear:.2f})
  Angular Speed: {angular:.2f} rad/s (max: {max_angular:.2f})
─────────────────────────────────────────────────────────────────
""".format(
            topic=self.cmd_vel_topic,
            linear=self.linear_speed,
            angular=self.angular_speed,
            max_linear=self.max_linear_speed,
            max_angular=self.max_angular_speed
        )
        
        print(self.msg)
        
        # Setup for reading keyboard
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self, timeout=0.1):
        """Get keyboard input with timeout"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_status(self):
        """Print current speed settings"""
        print("\r[LINEAR: {:.2f} m/s | ANGULAR: {:.2f} rad/s] - Cmd: [{:.2f}, {:.2f}]".format(
            self.linear_speed, self.angular_speed, 
            self.current_linear, self.current_angular), end='')
        sys.stdout.flush()

    def run(self):
        """Main control loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        try:
            print("\nRover teleop ready! Use keys to control the rover.\n")
            
            while not rospy.is_shutdown():
                key = self.get_key()
                
                twist = Twist()
                
                # Movement commands
                if key == 'i':  # Forward
                    self.current_linear = self.linear_speed
                    self.current_angular = 0.0
                elif key == ',':  # Backward
                    self.current_linear = -self.linear_speed
                    self.current_angular = 0.0
                elif key == 'j':  # Turn left (in place)
                    self.current_linear = 0.0
                    self.current_angular = self.angular_speed
                elif key == 'l':  # Turn right (in place)
                    self.current_linear = 0.0
                    self.current_angular = -self.angular_speed
                elif key == 'u':  # Forward + left
                    self.current_linear = self.linear_speed
                    self.current_angular = self.angular_speed
                elif key == 'o':  # Forward + right
                    self.current_linear = self.linear_speed
                    self.current_angular = -self.angular_speed
                elif key == 'm':  # Backward + left
                    self.current_linear = -self.linear_speed
                    self.current_angular = self.angular_speed
                elif key == '.':  # Backward + right
                    self.current_linear = -self.linear_speed
                    self.current_angular = -self.angular_speed
                elif key == 'k' or key == ' ':  # Stop
                    self.current_linear = 0.0
                    self.current_angular = 0.0
                    print("\n[EMERGENCY STOP]")
                
                # Speed adjustment
                elif key == 'q':  # Increase linear speed
                    self.linear_speed = min(self.max_linear_speed, 
                                           self.linear_speed + self.linear_increment)
                    print("\n[Linear speed: {:.2f} m/s]".format(self.linear_speed))
                elif key == 'z':  # Decrease linear speed
                    self.linear_speed = max(0.1, 
                                           self.linear_speed - self.linear_increment)
                    print("\n[Linear speed: {:.2f} m/s]".format(self.linear_speed))
                elif key == 'w':  # Increase angular speed
                    self.angular_speed = min(self.max_angular_speed,
                                            self.angular_speed + self.angular_increment)
                    print("\n[Angular speed: {:.2f} rad/s]".format(self.angular_speed))
                elif key == 'x':  # Decrease angular speed
                    self.angular_speed = max(0.1,
                                            self.angular_speed - self.angular_increment)
                    print("\n[Angular speed: {:.2f} rad/s]".format(self.angular_speed))
                elif key == '\x03':  # Ctrl+C
                    break
                
                # Publish command
                twist.linear.x = self.current_linear
                twist.angular.z = self.current_angular
                self.cmd_vel_pub.publish(twist)
                
                self.print_status()
                rate.sleep()
                
        except Exception as e:
            print("\n[ERROR] {}".format(e))
        finally:
            # Send stop command
            print("\n\n[Stopping rover...]")
            twist = Twist()
            for _ in range(5):
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("[Teleop shutdown complete]")

if __name__ == '__main__':
    try:
        teleop = TeleopMarsRover()
        teleop.run()
    except rospy.ROSInterruptException:
        print("\n[ROS Interrupt]")
    except KeyboardInterrupt:
        print("\n[Keyboard Interrupt]")