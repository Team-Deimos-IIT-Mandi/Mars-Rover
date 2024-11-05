#!/usr/bin/env python3
import rospy
import smach
import subprocess  # To open new terminal
import rospkg
import os

class Teleop(smach.State):


    def terminate_process(self):
        """Safely terminate the teleoperation process."""
        if self.process:
            rospy.loginfo('Terminating teleop process...')
            self.process.terminate()
            self.process.wait()  # Ensure the process terminates before continuing
            rospy.loginfo('Teleop process terminated.')



    def __init__(self):
        smach.State.__init__(self, outcomes=['autonomous', 'spiral_search', 'idle', 'shutdown'])
        self.process = None  # To store the subprocess

    def execute(self, userdata):
        rospy.loginfo('Launching teleoperation mode in a new terminal...')

        try:
            # Get path to the teleop.launch file in rover_control package
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('rover_control')
            launch_file_path = os.path.join(package_path, 'launch', 'teleop.launch')

            # Check if the launch file exists before executing
            if not os.path.exists(launch_file_path):
                rospy.logerr(f"Launch file {launch_file_path} does not exist!")
                return 'failed'

            # Launch the teleoperation task in a new terminal
            command = f"gnome-terminal -- bash -c 'roslaunch {package_path}/launch/teleop.launch; exec bash'"
            rospy.loginfo(f"Executing command: {command}")

            # Try to start the subprocess
            self.process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            rospy.loginfo('Teleoperation mode started in a new terminal.')

            # Stay in teleop state until the user types a valid command for switching state
            while not rospy.is_shutdown():
                rospy.loginfo("Teleop: Type 'autonomous', 'spiral_search', or 'idle' to switch state:")
                
                # Use Python 3 compatible input method
                try:
                    user_input = input().strip().lower()

                    if user_input == 'autonomous':
                        rospy.loginfo('Switching to Autonomous mode...')
                        self.terminate_process()  # Terminate teleop process
                        return 'autonomous'

                    elif user_input == 'spiral_search':
                        rospy.loginfo('Switching to Spiral Search mode...')
                        self.terminate_process()  # Terminate teleop process
                        return 'spiral_search'

                    elif user_input == 'idle':
                        rospy.loginfo('Switching to Idle mode...')
                        self.terminate_process()  # Terminate teleop process
                        return 'idle'

                    elif user_input == 'shutdown':
                         rospy.loginfo('Shutting down the rover...')
                         return 'shutdown'

    
                    else:
                        rospy.logwarn(f"Invalid input: {user_input}. Please type 'autonomous', 'spiral_search', 'idle' or 'shutdown'.")
                
                except EOFError as e:
                    rospy.logerr(f"Error while getting user input: {e}")
                    self.terminate_process()  # Ensure the process is terminated on failure
                    return 'failed'

                rospy.sleep(1)  # Sleep to avoid constant logging and input checks

        except Exception as e:
            rospy.logerr(f'Teleoperation failed: {e}')
            self.terminate_process()  # Ensure the subprocess is terminated if an error occurs
            return 'failed'