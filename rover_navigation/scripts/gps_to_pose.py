#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class OdometryToPoseStamped:
    """
    A ROS node that converts Odometry messages into PoseStamped messages
    and republishes them as navigation goals for move_base.
    """

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gps_waypoint', anonymous=True)

        # Subscriber: listens to Odometry data (e.g., from GPS or localization)
        self.odom_sub = rospy.Subscriber(
            '/gps_point',                # Input topic
            Odometry,                    # Message type
            self.odom_callback           # Callback function
        )

        # Publisher: publishes PoseStamped messages to Move Base
        self.pose_pub = rospy.Publisher(
            '/move_base_simple/goal',    # Target topic for goals
            PoseStamped,                 # Message type
            queue_size=10
        )

    def odom_callback(self, msg: Odometry):
        """
        Callback function that processes incoming Odometry messages
        and converts them into PoseStamped messages.
        """
        # Extract position and orientation from Odometry
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Create a PoseStamped message
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header = msg.header  # Use same timestamp & frame

        # Assign position (force z=0 for 2D navigation)
        pose_stamped_msg.pose.position.x = position.x
        pose_stamped_msg.pose.position.y = position.y
        pose_stamped_msg.pose.position.z = 0.0

        # Assign orientation (use from Odometry; can override if unreliable)
        pose_stamped_msg.pose.orientation = orientation

        # Publish the PoseStamped goal
        self.pose_pub.publish(pose_stamped_msg)

    def run(self):
        """
        Keeps the node alive and responsive to callbacks.
        """
        rospy.loginfo("Odometry to PoseStamped node started. Listening on /gps_point...")
        rospy.spin()


if __name__ == '__main__':
    try:
        # Instantiate the converter node
        odom_to_posestamped = OdometryToPoseStamped()

        # Run the node
        odom_to_posestamped.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Odometry to PoseStamped node.")
