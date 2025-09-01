#!/usr/bin/env python3
import struct
import rospy
import smbus2
from std_msgs.msg import Float32MultiArray

class I2CCommunicator:
    """
    A ROS node to subscribe to velocity and position data and send it over I2C.

    This class handles the entire workflow:
    - Initializing the ROS node and I2C bus.
    - Subscribing to the relevant ROS topic.
    - Processing incoming data in a callback.
    - Converting the data for I2C transmission.
    - Sending the data to I2C slave devices.
    - Handling potential I2C communication errors.
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('vp_listener_node', anonymous=True)
        
        # I2C parameters
        self.I2C_BUS = 1
        self.SLAVE_ADD_FRONT = 0x52
        self.SLAVE_ADD_REAR = 0x53
        self.bus = None

        # Initialize I2C bus with error handling
        try:
            self.bus = smbus2.SMBus(self.I2C_BUS)
            rospy.loginfo("Successfully initialized I2C bus.")
        except FileNotFoundError:
            rospy.logerr("Could not open I2C bus. Please check the bus number and file permissions.")
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred while initializing I2C bus: {e}")

        # ROS subscriber
        self.vp_sub = rospy.Subscriber('/mars_rover/velo_and_pos', Float32MultiArray, self.vp_callback)
        self.vp_data = None
        rospy.loginfo("Subscribed to /mars_rover/velo_and_pos topic.")

    def vp_callback(self, msg):
        """
        Callback function to store the latest received data from the ROS topic.
        
        This method is called automatically by the ROS subscriber when a new message arrives.
        It updates the internal `vp_data` attribute with the message's data.
        """
        self.vp_data = msg.data

    def convert_and_send(self, data, address):
        """
        Converts a list of floating-point numbers to bytes and sends them over I2C.

        Args:
            data (list): A list of float values to be sent.
            address (int): The I2C address of the slave device.
        """
        if self.bus is None:
            rospy.logwarn("I2C bus not initialized. Skipping data send.")
            return

        try:
            # Pack float data into a byte string using big-endian format (>)
            data_bytes = struct.pack(">" + "f" * len(data), *data)
            
            # Send data byte by byte to the slave device.
            for i, bite in enumerate(data_bytes):
                self.bus.write_byte_data(address, i, bite)
            
            rospy.loginfo(f"Sent {len(data)} floats to slave at 0x{address:02X}.")

        except IOError as e:
            rospy.logerr(f"I/O Error during I2C communication with 0x{address:02X}: {e}")
        except struct.error as e:
            rospy.logerr(f"Structuring error when packing data: {e}")
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred during I2C communication: {e}")

    def run(self):
        """
        The main loop of the ROS node.
        
        This loop checks for new data, processes it, and sends it over I2C at a specified rate.
        It runs until the ROS node is shut down.
        """
        rate = rospy.Rate(10)  # Loop at 10 Hz
        while not rospy.is_shutdown():
            if self.vp_data:
                vp_data = list(self.vp_data)
                
                # Split the data into front and rear axle commands
                vp_data_front = [vp_data[0], vp_data[1], vp_data[4], vp_data[5]]
                vp_data_rear = [vp_data[2], vp_data[3], vp_data[6], vp_data[7]]

                rospy.loginfo(f"Processing Front Data: {vp_data_front}")
                rospy.loginfo(f"Processing Rear Data: {vp_data_rear}")
                
                # Uncomment the following lines to enable I2C communication
                # self.convert_and_send(vp_data_front, self.SLAVE_ADD_FRONT)
                # self.convert_and_send(vp_data_rear, self.SLAVE_ADD_REAR)

                # Clear the data after processing to ensure only new messages are sent
                self.vp_data = None
            
            rate.sleep()

if __name__ == '__main__':
    try:
        comm = I2CCommunicator()
        comm.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node was shut down.")