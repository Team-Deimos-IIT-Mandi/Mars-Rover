#!/usr/bin/env python3
import rospy
import smach
import subprocess
import rospkg
import os
from std_msgs.msg import String


class Autonomous(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['teleop', 'spiral_search', 'idle', 'shutdown'])
        self.process = None  
        self.current_state = "autonomous"  # Set default state
        rospy.Subscriber('/smach_state', String, self.callback)

    def callback(self, data):
        """ Callback function to update state based on /smach_state topic """
        new_state = data.data.strip().lower()

        if new_state in ['teleop', 'spiral_search', 'idle', 'shutdown']:
            rospy.loginfo(f"Received new state: {new_state}")
            self.current_state = new_state  # Immediately store new state
        else:
            rospy.logwarn(f"Invalid state received: {new_state}")

    def execute(self, userdata):
        rospy.loginfo('Launching Autonomous Navigation mode in a new terminal...')

        try:
            if self.process is None:
                rospack = rospkg.RosPack()
                package_path = rospack.get_path('rover_navigation')
                launch_file_path = os.path.join(package_path, 'launch', 'move_base_mapless_demo.launch')

                command = f"gnome-terminal -- bash -c 'roslaunch {launch_file_path}; exec bash'"
                self.process = subprocess.Popen(command, shell=True)
                rospy.loginfo('Autonomous Navigation started.')

            while not rospy.is_shutdown():
                if self.current_state != "autonomous":  # Check if state changed
                    rospy.loginfo(f"Transitioning to state: {self.current_state}")
                    if self.current_state == 'teleop':
                        return 'teleop'
                    elif self.current_state == 'spiral_search':
                        return 'spiral_search'
                    elif self.current_state == 'idle':
                        return 'idle'
                    elif self.current_state == 'shutdown':
                        self.terminate_process()  # Ensure process is terminated
                        return 'shutdown'
                    else :
                        rospy.logwarn(f"Invalid input : {self.current_state}")    
                        return 'idle'

                rospy.sleep(2.0)  # Prevent high CPU usage

        except Exception as e:
            rospy.logerr(f'Autonomous navigation failed: {e}')
            self.terminate_process()  # Ensure process is cleaned up on error
            return 'idle'  # Return a valid fallback state

        rospy.logwarn("Unexpected loop exit! Defaulting to 'idle'.")
        return 'idle'  # Ensure a valid outcome is always returned

    def terminate_process(self):
        """Terminates the subprocess running the launch file."""
        if self.process:
            rospy.loginfo("Terminating Autonomous Navigation...")
            self.process.terminate()
            rospy.sleep(3)  # Allow process to terminate
            
            result = subprocess.run(["pgrep", "-f", "gnome-terminal"], stdout=subprocess.PIPE, text=True)
            pids = result.stdout.strip().split("\n")

            for pid in pids:
                if pid:
                    subprocess.run(["kill", "-9", pid])
                    rospy.loginfo(f"Killed process {pid}")


if __name__ == "__main__":
    rospy.init_node('autonomous_state')
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    
    with sm:
        smach.StateMachine.add('AUTONOMOUS', Autonomous(), 
                               transitions={'teleop': 'success', 
                                            'spiral_search': 'success', 
                                            'idle': 'success',
                                            'shutdown': 'failure'})

    outcome = sm.execute()
