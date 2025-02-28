import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

class GoalAchiever:
    def __init__(self):
        rospy.init_node('goal_achiever', anonymous=True)
        
        # Subscribers
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.odom_sub = rospy.Subscriber("/odometry/filtered/global", Odometry, self.odom_callback)
        
        # Publisher
        self.goal_achieved_pub = rospy.Publisher("/goal_achieved", Bool, queue_size=10)
        
        self.goal = None
        self.tolerance = 0.2  # Distance threshold for considering goal achieved
        
    def goal_callback(self, msg):
        """Callback function to receive goal coordinates."""
        self.goal = msg.pose.position
        rospy.loginfo(f"Received new goal: {self.goal.x}, {self.goal.y}")
    
    def odom_callback(self, msg):
        """Callback function to receive current position and check goal achievement."""
        if self.goal is None:
            return
        
        current_position = msg.pose.pose.position
        distance = self.euclidean_distance(current_position, self.goal)
        
        if distance <= self.tolerance:
            rospy.loginfo("Goal reached!")
            self.goal_achieved_pub.publish(Bool(data=True))
        else:
            self.goal_achieved_pub.publish(Bool(data=False))
    
    def euclidean_distance(self, pos1, pos2):
        """Compute Euclidean distance between two positions."""
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        goal_achiever = GoalAchiever()
        goal_achiever.run()
    except rospy.ROSInterruptException:
        pass
