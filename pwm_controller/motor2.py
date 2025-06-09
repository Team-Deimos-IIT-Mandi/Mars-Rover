import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import smbus

L = 0.5
R = 0.1615
I2C_BUS = 1  # Adjust based on your hardware
STM_ADDRESS = 0x12<<1  # Adjust based on your STM board's I2C address

# Set the desired I2C clock speed (frequency) in kHz
desired_i2c_speed_khz = 100

# Set I2C clock speed during initialization
def initialize_i2c_bus(bus, speed_khz):
    try:
        bus.write_byte_data(STM_ADDRESS, 0x00, speed_khz)  # Set the clock speed
        print(f"I2C clock speed set to {speed_khz} kHz")
    except IOError:
        rospy.logerr("Failed to set I2C clock speed.")

# Initialize the I2C bus with the desired speed


def on_twist(msg):
    L = 0.5
    R = 0.1615
    #define L 0.5
#define R 0.1615
    l = (1/R) * (msg.linear.x - msg.angular.z * L / 2)
    r = (1/R) * (msg.linear.x + msg.angular.z * L / 2)

    if l == 0 and r == 0:
        l_pwm = 0
        r_pwm = 0
    else:
        l_pwm = 10 + (10.84599 * l) if l > 0 else -10 + (10.84599 * l)
        r_pwm = 10 + (10.84599 * r) if r > 0 else -10 + (10.84599 * r)


    # Send data over I2C
    try:
        bus = smbus.SMBus(I2C_BUS)
        bus.write_byte_data(STM_ADDRESS, 0, r_pwm)
        bus.close()
    except IOError:
        rospy.logerr("Failed to write to I2C device.")

def main():
    rospy.init_node('cmd_vel_to_i2c', anonymous=True)
    try:
        bus = smbus.SMBus(I2C_BUS)
        initialize_i2c_bus(bus, desired_i2c_speed_khz)
    except IOError:
        rospy.logerr("Failed to initialize I2C bus.")

    rospy.Subscriber('/cmd_vel', Twist, on_twist)
    chatter_pub = rospy.Publisher('debug', String, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # You can publish debug messages here if needed
        rospy.spinOnce()
        rate.sleep()

if __name__ == '_main_':
    try:
        main()
    except rospy.ROSInterruptException:
        pass