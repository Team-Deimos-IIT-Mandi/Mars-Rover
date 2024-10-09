#!/usr/bin/env python3
import rospy
import smach
import subprocess  # Import subprocess to launch in new terminal
import os
import rospkg

class SpiralSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'failed'])
        self.process = None  # Handle for the new terminal process

    def execute(self, userdata):
        rospy.loginfo('Launching spiral search mode...')

        try:
            # Get the path to the 'arucoSpawner.launch' file
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('aruco_sim_rover')
            launch_file_path = os.path.join(package_path, 'launch', 'arucoSpawner.launch')

            # Command to launch in a new terminal using xterm
            command = f"xterm -e 'roslaunch {package_path} {launch_file_path}'"
            
            # Start the launch file in a new terminal
            self.process = subprocess.Popen(command, shell=True)
            rospy.loginfo('Spiral search mode started in new terminal.')

            # Simulate the spiral search task (replace with actual search logic)
            while self.process.poll() is None:  # Check if the process is still running
                rospy.sleep(1)  # Replace with actual spiral search logic if necessary

            # Process completed successfully
            if self.process.returncode == 0:
                rospy.loginfo('Spiral search mode completed successfully.')
                return 'completed'
            else:
                rospy.logerr('Spiral search mode failed.')
                return 'failed'

        except Exception as e:
            rospy.logerr(f'Spiral search failed: {e}')
            if self.process:
                self.process.terminate()
            return 'failed'

    def terminate(self):
        # Ensure the process is properly terminated when the state exits
        if self.process:
            self.process.terminate()
            rospy.loginfo('Spiral search process terminated.')
