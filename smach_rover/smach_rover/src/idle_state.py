#!/usr/bin/env python3
import rospy
import smach
import subprocess
from std_msgs.msg import String
import os

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['teleop', 'autonomous', 'spiral_search', 'shutdown'])
        self.process = None
        self.current_state = "idle"  # Set default state
        rospy.Subscriber('/smach_state', String, self.callback)

    def callback(self, data):
        """ Callback function to handle state changes from /smach_state topic """
        new_state = data.data.strip().lower()

        if new_state in ['teleop', 'autonomous', 'spiral_search', 'shutdown']:
            rospy.loginfo(f"Received new state: {new_state}")
            self.current_state = new_state  # Immediately store new state
        else:
            rospy.logwarn(f"Invalid state received: {new_state}")

    def execute(self, userdata):
        rospy.loginfo('Rover is in IDLE mode. Waiting for state transitions...')

        while not rospy.is_shutdown():
            if self.current_state != "idle":  # Check if state changed
                        rospy.loginfo(f"Transitioning to state: {self.current_state}")
                        if self.current_state == 'autonomous':
                            return 'autonomous'
                        elif self.current_state == 'teleop':
                            return 'teleop'
                        elif self.current_state == 'spiral_search':
                            return 'spiral_search'
                        elif self.current_state == 'shutdown':
                            return 'shutdown'
                            self.terminate_process()
                        else :    
                            rospy.logwarn(f"Invalid input: {self.received_state}.")

            rospy.sleep(2.0)

    def terminate_process(self):
        """Kills all gnome-terminal processes."""
        try:
            result = subprocess.run(
                ["pgrep", "-f", "gnome-terminal"], stdout=subprocess.PIPE, text=True
            )
            pids = result.stdout.strip().split("\n")

            for pid in pids:
                if pid:  # Ensure valid PID
                    subprocess.run(["kill", "-9", pid])
                    rospy.loginfo(f"Killed process {pid}")

        except Exception as e:
            rospy.logwarn(f"Error killing processes: {e}")


if __name__ == "__main__":
    rospy.init_node('idle')
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    
    with sm:
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'autonomous': 'success', 
                                            'teleop': 'success', 
                                            'spiral_search': 'success',
                                            'shutdown': 'failure'})

    outcome = sm.execute()            
