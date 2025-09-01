import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import smbus
import sys

class I2CMotorController:
    """
    A class to handle ROS communication and I2C motor control.
    """
    def __init__(self, stm_address, i2c_bus, i2c_speed_khz):
        """
        Initializes the ROS node, I2C bus, and robot parameters.

        Args:
            stm_address (int): The I2C address of the STM board.
            i2c_bus (int): The I2C bus number on the system.
            i2c_speed_khz (int): The desired I2C clock speed in kHz.
        """
        self.STM_ADDRESS = stm_address
        self.I2C_BUS = i2c_bus
        self.desired_i2c_speed_khz = i2c_speed_khz

        self.L = 0.5
        self.R = 0.1615

        try:
            self.bus = smbus.SMBus(self.I2C_BUS)
            self._initialize_i2c_speed()
        except FileNotFoundError:
            rospy.logerr(f"I2C bus not found at /dev/i2c-{self.I2C_BUS}. Please check your hardware configuration.")
            sys.exit(1)
        except Exception as e:
            rospy.logerr(f"An error occurred while initializing the I2C bus: {e}")
            sys.exit(1)

        rospy.init_node('cmd_vel_to_i2c', anonymous=True)
        rospy.Subscriber('/cmd_vel', Twist, self.on_twist)
        self.chatter_pub = rospy.Publisher('debug', String, queue_size=10)

    def _initialize_i2c_speed(self):
        """
        Sets the I2C clock speed on the STM board.
        """
        try:
            self.bus.write_byte_data(self.STM_ADDRESS, 0x00, self.desired_i2c_speed_khz)
            rospy.loginfo(f"I2C clock speed set to {self.desired_i2c_speed_khz} kHz.")
        except IOError as e:
            rospy.logerr(f"Failed to set I2C clock speed: {e}")

    def _send_motor_commands(self, r_pwm):
        """
        Sends the PWM data for the right motor over I2C.

        Args:
            r_pwm (int): The PWM value for the right motor.
        """
        try:
            self.bus.write_byte_data(self.STM_ADDRESS, 0, r_pwm)
        except IOError as e:
            rospy.logerr(f"Failed to write to I2C device at address {hex(self.STM_ADDRESS)}: {e}")
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred while sending I2C data: {e}")

    def on_twist(self, msg):
        """
        Callback function for the /cmd_vel topic.

        Args:
            msg (Twist): The received ROS Twist message with velocity commands.
        """
        l = (1 / self.R) * (msg.linear.x - msg.angular.z * self.L / 2)
        r = (1 / self.R) * (msg.linear.x + msg.angular.z * self.L / 2)

        if l == 0 and r == 0:
            l_pwm = 0
            r_pwm = 0
        else:
            l_pwm = 10 + (10.84599 * l) if l > 0 else -10 + (10.84599 * l)
            r_pwm = 10 + (10.84599 * r) if r > 0 else -10 + (10.84599 * r)

        self._send_motor_commands(r_pwm)

    def run(self):
        """
        Starts the ROS event loop.
        """
        rospy.spin()

if __name__ == '__main__':
    try:
        STM_ADDRESS = 0x12 << 1
        I2C_BUS = 1
        desired_i2c_speed_khz = 100
        
        controller = I2CMotorController(STM_ADDRESS, I2C_BUS, desired_i2c_speed_khz)
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted. Shutting down.")
    except Exception as e:
        rospy.logerr(f"An unhandled exception occurred: {e}")