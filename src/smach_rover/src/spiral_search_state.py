#!/usr/bin/env python3
import rospy
import smach
import subprocess  
import os
import rospkg
import signal
from std_msgs.msg import String


class SpiralSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['teleop', 'autonomous', 'shutdown', 'idle'])
        self.process = None  # Handle for the new terminal process
        self.current_state = "spiral_search"  # Set default state
        rospy.Subscriber('/smach_state', String, self.callback)

    def callback(self, data):
        """ Callback function to update state based on /smach_state topic """
        new_state = data.data.strip().lower()

        if new_state in ['teleop', 'autonomous', 'idle', 'shutdown']:
            rospy.loginfo(f"Received new state: {new_state}")
            self.current_state = new_state  # Immediately store new state
        else:
            rospy.logwarn(f"Invalid state received: {new_state}")

    def execute(self, userdata):
        rospy.loginfo('Launching Spiral Search mode...')

        try:

            if self.process is None : 
                launch_file_path = "/home/ashutosh/catkin_ws/src/Mars-Rover/marker_detection_rover/scripts/spiral_search.py"
                command = [
                 "gnome-terminal",
                 "--",
                 "bash",
                 "-c",
                 f"python3 {launch_file_path}; exec bash"
                  ] 
                self.process = subprocess.Popen(command, preexec_fn=os.setpgrp)
                rospy.loginfo("Spiral search started in a new terminal.")

            while not rospy.is_shutdown():
                rospy.sleep(2.0)

                if self.current_state != "spiral_search":  # Check if state changed
                    rospy.loginfo(f"Transitioning to state: {self.current_state}")

                    if self.current_state == 'autonomous':
                        return 'autonomous'
                    elif self.current_state == 'teleop':
                        return 'teleop'
                    elif self.current_state == 'idle':
                        return 'idle'
                    elif self.current_state == 'shutdown':
                        self.terminate_process()
                        return 'shutdown'

        except Exception as e:
            rospy.logerr(f'Spiral Search failed: {e}')
            self.terminate_process()
            return 'idle'

        # **Ensure a valid return even if rospy shuts down**
        rospy.logwarn("Unexpected exit from SpiralSearch execute()! Defaulting to 'idle'.")
        return 'idle'

    def terminate_process(self):
        """Terminates the subprocess running the launch file."""
        if self.process:
            rospy.loginfo("Terminating Spiral Search process...")
            self.process.terminate()
            rospy.sleep(3)  # Allow time for process to terminate

            # Properly find and kill the process
            result = subprocess.run(["pgrep", "-f", "gnome-terminal"], stdout=subprocess.PIPE, text=True)
            pids = result.stdout.strip().split("\n")

            for pid in pids:
                if pid:
                    subprocess.run(["kill", "-9", pid])
                    rospy.loginfo(f"Killed process {pid}")


if __name__ == "__main__":
    rospy.init_node('spiral_search')
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    
    with sm:
        smach.StateMachine.add('SPIRAL', SpiralSearch(),  
                               transitions={'autonomous': 'success', 
                                            'teleop': 'success', 
                                            'idle': 'success',
                                            'shutdown': 'failure'})

    outcome = sm.execute()
    rospy.loginfo(f"State Machine exited with outcome: {outcome}")  # Debug logging
