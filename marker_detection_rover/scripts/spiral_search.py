#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import math
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool  # Added for AR topic

class SpiralWaypointGenerator:
    def __init__(self):
        rospy.init_node('spiral_waypoint_generator', anonymous=True)

        self.threshold_distance = 3
        self.initial_pose = None
        self.spiral_radius = 30
        self.step_increment = 0.1
        self.waypoints = []
        
        self.current_position = None
        self.ar_active = False  # Add AR active status flag
        
        # Subscribers and publishers
        self.odom_subscriber = rospy.Subscriber('/odometry/filtered/local', Odometry, self.odom_callback)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.marker_publisher = rospy.Publisher('/waypoints_marker', Marker, queue_size=10)
        self.AR = rospy.Subscriber('/AR', Bool, self.ar_callback)  # Add AR callback

    def ar_callback(self, msg):
        """Callback for the AR topic to stop the spiral search."""
        self.ar_active = msg.data  # Update the status of AR
        
        if self.ar_active:
            rospy.loginfo("AR detected, stopping the spiral search.")
            # You can stop the spiral generation here if needed
            self.waypoints = []  # Clear the waypoints list to stop publishing goals
            self.initial_pose = None  # Reset the initial pose as well
        
    def generate_spiral_waypoints(self):
        waypoints = []
        t = 0
        while t <= self.spiral_radius:
            x = self.initial_pose.x + math.cos(t) * t / 3
            y = self.initial_pose.y + math.sin(t) * t / 3
            
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'odom'
            waypoint.pose.position = Point(x, y, 0.0)
            
            tangent_angle = math.atan2(y + t * math.cos(t), x - t * math.sin(t))
            quaternion = quaternion_from_euler(0, 0, tangent_angle)
            waypoint.pose.orientation.x = quaternion[0]
            waypoint.pose.orientation.y = quaternion[1]
            waypoint.pose.orientation.z = quaternion[2]
            waypoint.pose.orientation.w = quaternion[3]

            waypoints.append(waypoint)

            t += self.step_increment 
        
        return waypoints

    def publish_waypoints_markers(self):
        """Publishes the waypoints as markers for RViz visualization."""
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'spiral_waypoints'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5  # Size of the points
        marker.scale.y = 0.5  # Size of the points
        marker.color.a = 1.0  # Full opacity
        marker.color.r = 1.0  # Red color

        # Add the positions of the waypoints to the marker
        for waypoint in self.waypoints:
            point = Point()
            point.x = waypoint.pose.position.x
            point.y = waypoint.pose.position.y
            point.z = waypoint.pose.position.z
            marker.points.append(point)

        # Publish the marker
        self.marker_publisher.publish(marker)

    def odom_callback(self, odom_msg):
        if self.ar_active:  # Check if AR is active (True)
            rospy.loginfo("AR is active, stopping the odometry callback.")
            return  # Exit if AR is active
        
        if odom_msg.header.stamp.secs != rospy.Time.now().secs:
            return None
        
        self.current_position = odom_msg.pose.pose.position
        rate = rospy.Rate(2)
        
        if not self.initial_pose:
            self.initial_pose = self.current_position
            print(self.initial_pose.x, self.initial_pose.y)
            self.waypoints = self.generate_spiral_waypoints()
            print(self.waypoints[0].pose.position.x, self.waypoints[0].pose.position.y)
        
        self.publish_waypoints_markers()
        rate.sleep()
        
        if len(self.waypoints) > 0:
            next_waypoint = self.waypoints[0].pose.position
            distance = math.sqrt((self.current_position.x - next_waypoint.x)**2 +
                                 (self.current_position.y - next_waypoint.y)**2)
            
            if len(self.waypoints) > 0 and distance < self.threshold_distance:
                goal_msg = self.waypoints[0]
                self.waypoints.pop(0)
                goal_msg.header.stamp = rospy.Time.now()
                self.goal_publisher.publish(goal_msg)
        else:
            print("Spiral Search Finished")

    def run(self):
        while not rospy.is_shutdown():
            if self.ar_active:
                rospy.loginfo("AR is active, stopping the spiral search.")
                break  # Exit if AR is active
            rospy.sleep(0.1)  # Adjust the sleep duration as needed

if __name__ == '__main__':
    print("Spiral Search Started")
    spiral_generator = SpiralWaypointGenerator()
    spiral_generator.run()
