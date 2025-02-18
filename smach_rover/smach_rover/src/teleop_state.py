#!/usr/bin/env python3
import rospy
import smach
import subprocess
import rospkg
import os
import signal
from std_msgs.msg import String

class Teleop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['autonomous', 'spiral_search', 'idle', 'shutdown'])
        self.process = None  # To store the subprocess
        self.current_state = "teleop"  # Set default state
        rospy.Subscriber('/smach_state', String, self.callback)

    def callback(self, data):
        """Callback function for subscriber to update the state."""
        new_state = data.data.strip().lower()

        if new_state in ['autonomous', 'spiral_search', 'idle', 'shutdown']:
            rospy.loginfo(f"Received new state: {new_state}")
            self.current_state = new_state  # Immediately store new state
        else:
            rospy.logwarn(f"Invalid state received: {new_state}")

    def execute(self, userdata):
        rospy.loginfo('Launching teleoperation mode in a new terminal...')

        try:
            if self.process is None :
                 rospack = rospkg.RosPack()
                 package_path = rospack.get_path('rover_control')
                 launch_file_path = os.path.join(package_path, 'launch', 'teleop.launch')
                 command = f"gnome-terminal -- bash -c 'roslaunch {package_path}/launch/teleop.launch; exec bash'"
                 self.process = subprocess.Popen(command, shell=True)
                 rospy.loginfo('Teleoperation mode started.')

            while not rospy.is_shutdown():
                
                if self.current_state != "teleop":  # Check if state changed
                    rospy.loginfo(f"Transitioning to state: {self.current_state}")

                    if self.current_state == 'autonomous':
                        return 'autonomous'
                    elif self.current_state == 'spiral_search':
                        return 'spiral_search'
                    elif self.current_state == 'idle':
                        return 'idle'
                    elif self.current_state == 'shutdown':
                        self.terminate_process()
                        return 'shutdown'
                    else :    
                        rospy.logwarn(f"Invalid input: {self.current_state}.")
                        return 'idle'
                        
                rospy.sleep(2.0)        

        except Exception as e:
            rospy.logerr(f'Teleoperation failed: {e}')
            self.terminate_process()
            return 'idle'

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
    rospy.init_node('teleop_state')
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    
    with sm:
        smach.StateMachine.add('TELEOP', Teleop(), 
                               transitions={'autonomous': 'success', 
                                            'spiral_search': 'success', 
                                            'idle': 'success',
                                            'shutdown': 'failure'})

    outcome = sm.execute()
