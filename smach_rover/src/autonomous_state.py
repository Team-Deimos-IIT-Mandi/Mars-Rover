#!/usr/bin/env python3
import rospy
import smach
import subprocess 
import rospkg
import os
import signal

class Autonomous(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['teleop', 'spiral_search', 'idle', 'shutdown'])
        self.process = None  # To store the subprocess


    def execute(self, userdata):
        rospy.loginfo('Launching autonomous navigation mode in a new terminal...')

        try:
            # Get path to the move_base_mapless_demo.launch file in rover_navigation package
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('rover_navigation')
            launch_file_path = os.path.join(package_path, 'launch', 'move_base_mapless_demo.launch')

            # Launch the autonomous navigation task in a new terminal
            command = f"gnome-terminal -- bash -c 'roslaunch {package_path}/launch/move_base_mapless_demo.launch; exec bash'"
            self.process = subprocess.Popen(command, shell=True)
            rospy.loginfo('Autonomous navigation mode started in a new terminal.')

            # Stay in autonomous state until the user types a valid command for switching state
            while not rospy.is_shutdown():
                # Prompt user to type a command to switch state
                rospy.loginfo("Autonomous: Type 'teleop', 'spiral_search', 'shutdown' or 'idle' to switch state:")
                user_input = input().lower()
                try : 
                   if user_input == 'teleop':
                       rospy.loginfo('Switching to Teleoperation mode...')
                       return 'teleop'
                
                   elif user_input == 'spiral_search':
                       rospy.loginfo('Switching to Spiral Search mode...')
                       return 'spiral_search'
                
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
                        rospy.logwarn(f"Invalid input: {user_input}. Please type 'teleop', 'spiral_search', 'shutdown' or 'idle'.")

                except EOFError as e:
                    rospy.logerr(f"Error while getting user input: {e}")
                    self.terminate_process()  # Ensure the process is terminated on failure
                    return 'failed'

                rospy.sleep(1)  # Sleep to avoid constant logging and input checks

        except Exception as e:
            rospy.logerr(f'Autonomous navigation failed: {e}')
            if self.process:
                self.process.terminate()  # Ensure the subprocess is terminated if an error occurs
            return 'failed'