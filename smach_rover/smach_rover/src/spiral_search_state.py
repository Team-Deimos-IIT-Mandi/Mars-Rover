#!/usr/bin/env python3
import rospy
import smach
import subprocess  
import os
import rospkg
import signal


class SpiralSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['teleop', 'autonomous', 'shutdown', 'idle'])
        self.process = None  # Handle for the new terminal process


    def execute(self, userdata):
        rospy.loginfo('Launching spiral search mode...')

        try:
        
            launch_file_path = "/home/ashutosh/catkin_ws/src/Mars-Rover/marker_detection_rover/scripts/spiral_search.py"

           
            command = [
                  "gnome-terminal",
                  "--",
                  "bash",
                  "-c",
                  f"python3 {launch_file_path}; exec bash"
                    ]

            # Launch the new terminal and return the process object
            self.process = subprocess.Popen(command, preexec_fn=os.setpgrp)

            while not rospy.is_shutdown():
                # Prompt user to type a command to switch state
                rospy.loginfo("Spiral Search: Type 'teleop', 'autonomous', 'idle', or 'shutdown' to switch state:")
                user_input = input().lower()

                if user_input == 'teleop':
                    rospy.loginfo('Switching to Teleoperation mode...')
                    return 'teleop'

                elif user_input == 'autonomous':
                    rospy.loginfo('Switching to Autonomous mode...')
                    return 'autonomous'

                elif user_input == 'idle':
                    rospy.loginfo('Switching to Idle mode...')
                    return 'idle'

                elif user_input == 'shutdown':
                    rospy.loginfo('Shutting down the rover...')
                    rospy.sleep(6.0)
                    result = subprocess.run(
                              ["pgrep", "-f", "gnome-terminal"], 
                              stdout=subprocess.PIPE , 
                              text=True
                                )
                    # Get the list of process IDs
                    pids = result.stdout.strip().split("\n")
                    # Kill each process
                    for pid in pids:
                        subprocess.run(["kill", "-9", pid])
            
                else:
                    rospy.logwarn(f"Invalid input: {user_input}. Please type 'teleop', 'autonomous', 'idle', or 'shutdown'.")

                rospy.sleep(1)

        except Exception as e:
            rospy.logerr(f'Spiral Search failed: {e}')
            if self.process:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            return 'idle'