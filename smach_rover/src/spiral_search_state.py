#!/usr/bin/env python3
import rospy
import smach
import subprocess  # Import subprocess to launch in new terminal
import os
import rospkg

class SpiralSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['teleop', 'autonomous', 'shutdown', 'idle'])
        self.process = None  # Handle for the new terminal process

    def execute(self, userdata):
        rospy.loginfo('Launching spiral search mode...')

        try:
            # Get the path to the 'arucoSpawner.launch' file
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('aruco_sim_rover')
            launch_file_path = os.path.join(package_path, 'launch', 'arucoSpawner.launch')

            # Command to launch in a new terminal using gnome-terminal
            command = f"gnome-terminal -- bash -c 'roslaunch {package_path} {launch_file_path}; exec bash'"

            # Start the launch file in a new terminal
            self.process = subprocess.Popen(command, shell=True)
            rospy.loginfo('Spiral search mode started in new terminal.')

            while not rospy.is_shutdown():
                # Prompt user to type a command to switch state
                rospy.loginfo("Spiral Search: Type 'teleop', 'autonomous', 'idle', or 'shutdown' to switch state:")
                user_input = input().lower()

                if user_input == 'teleop':
                    self.process.terminate()  # Terminate spiral search terminal
                    rospy.loginfo('Switching to Teleoperation mode...')
                    return 'teleop'

                elif user_input == 'autonomous':
                    self.process.terminate()  # Terminate spiral search terminal
                    rospy.loginfo('Switching to Autonomous mode...')
                    return 'autonomous'

                elif user_input == 'idle':
                    self.process.terminate()  # Terminate spiral search terminal
                    rospy.loginfo('Switching to Idle mode...')
                    return 'idle'

                elif user_input == 'shutdown':
                    self.process.terminate()  # Terminate spiral search terminal
                    rospy.loginfo('Shutting down the rover...')
                    return 'shutdown'

                else:
                    rospy.logwarn(f"Invalid input: {user_input}. Please type 'teleop', 'autonomous', 'idle', or 'shutdown'.")

                rospy.sleep(1)

        except Exception as e:
            rospy.logerr(f'Spiral Search failed: {e}')
            if self.process:
                self.process.terminate()  # Ensure the subprocess is terminated if an error occurs
            return 'failed'
