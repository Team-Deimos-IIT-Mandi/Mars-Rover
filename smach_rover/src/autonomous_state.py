#!/usr/bin/env python3
import rospy
import smach
import subprocess  # To open a new terminal
import rospkg
import os

class Autonomous(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['teleop', 'spiral_search', 'idle', 'failed'])
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
                rospy.loginfo("Autonomous: Type 'teleop', 'spiral_search', or 'idle' to switch state:")
                user_input = input().lower()

                if user_input == 'teleop':
                    self.process.terminate()  # Terminate autonomous terminal
                    rospy.loginfo('Switching to Teleoperation mode...')
                    return 'teleop'
                
                elif user_input == 'spiral_search':
                    self.process.terminate()  # Terminate autonomous terminal
                    rospy.loginfo('Switching to Spiral Search mode...')
                    return 'spiral_search'
                
                elif user_input == 'idle':
                    self.process.terminate()  # Terminate autonomous terminal
                    rospy.loginfo('Switching to Idle mode...')
                    return 'idle'
                
                else:
                    rospy.logwarn(f"Invalid input: {user_input}. Please type 'teleop', 'spiral_search', or 'idle'.")

                rospy.sleep(1)  # Sleep to avoid constant logging and input checks

        except Exception as e:
            rospy.logerr(f'Autonomous navigation failed: {e}')
            if self.process:
                self.process.terminate()  # Ensure the subprocess is terminated if an error occurs
            return 'failed'
