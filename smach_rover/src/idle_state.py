#!/usr/bin/env python3
import rospy
import smach

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['teleop', 'autonomous', 'spiral_search', 'shutdown'])
        self.process = None  # Make sure self.process is initialized

    def terminate_process(self):
        """Terminate any running process if active."""
        if self.process:
            self.process.terminate()
            rospy.loginfo("Terminated the previous process.")
            self.process = None
        else:
            rospy.loginfo("No process running to terminate.")

    def execute(self, userdata):
        rospy.loginfo('Rover is in idle mode, ready for new commands')

        while not rospy.is_shutdown():
            # Prompt user to type a command to switch state
            rospy.loginfo("Type 'teleop', 'spiral_search', 'autonomous', or 'shutdown' to switch state:")
            user_input = input().lower()  # Blocking call (could be replaced with asynchronous method)

            if user_input == 'teleop':
                self.terminate_process()  # Terminate previous terminal if any
                rospy.loginfo('Switching to Teleoperation mode...')
                return 'teleop'

            elif user_input == 'spiral_search':
                self.terminate_process()  # Terminate previous terminal if any
                rospy.loginfo('Switching to Spiral Search mode...')
                return 'spiral_search'  # Return correct outcome 'spiral', not 'spiral_search'

            elif user_input == 'autonomous':
                self.terminate_process()  # Terminate previous terminal if any
                rospy.loginfo('Switching to Autonomous mode...')
                return 'autonomous'

            elif user_input == 'shutdown':
                rospy.loginfo('Shutting down the rover...')
                return 'shutdown'

            else:
                rospy.logwarn(f"Invalid input: {user_input}. Please type 'teleop', 'spiral_search', 'autonomous', or 'shutdown'.")

            rospy.sleep(1)  # Sleep to avoid constant logging and input checks

        # If ROS is shutting down, return shutdown outcome
        return 'shutdown'
