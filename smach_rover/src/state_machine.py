#!/usr/bin/env python3
import rospy
import smach
from idle_state import Idle
from teleop_state import Teleop
from autonomous_state import Autonomous
from spiral_search_state import SpiralSearch
import os
import rospkg
import subprocess

def launch_gazebo_and_rviz():
    """Launch Gazebo and RViz in new terminals."""
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('rover_description')
    gazebo_launch_file = os.path.join(package_path, 'launch', 'main.launch')
    rviz_launch_file = os.path.join(package_path, 'launch', 'display.launch')  # Ensure you have a RViz launch file

    # Debug information
    rospy.loginfo(f"Package path: {package_path}")
    rospy.loginfo(f"Gazebo launch file: {gazebo_launch_file}")

    try:
        # Launch Gazebo in a new terminal
        subprocess.Popen(['gnome-terminal', '--', 'roslaunch', 'rover_description', 'main.launch'])
        rospy.loginfo("Gazebo launched successfully in a new terminal.")

        # Launch RViz in a new terminal
        subprocess.Popen(['gnome-terminal', '--', 'roslaunch', 'rover_description', 'display.launch'])
        rospy.loginfo("RViz launched successfully in a new terminal.")

        return True  # Return True indicating successful launch

    except Exception as e:
        rospy.logerr(f"Failed to launch Gazebo and RViz: {e}")
        return None


def main():
    rospy.init_node('rover_state_machine')

    # Launch both Gazebo and RViz
    gazebo_rviz_launch = launch_gazebo_and_rviz()
    if gazebo_rviz_launch is None:
        rospy.logerr("Failed to launch Gazebo and RViz. Shutting down.")
        return

    # Create the state machine
    sm = smach.StateMachine(outcomes=['shutdown'])
    
    with sm:
        # Add the Idle state as the default starting state
        smach.StateMachine.add('IDLE', Idle(),
                               transitions={
                                   'teleop': 'TELEOPERATION',
                                   'autonomous': 'AUTONOMOUS_NAVIGATION',
                                   'spiral_search': 'SPIRAL_SEARCH',
                                   'shutdown': 'shutdown'
                               })

        # Autonomous navigation state
        smach.StateMachine.add('AUTONOMOUS_NAVIGATION', Autonomous(),
                        transitions={
                            'spiral_search': 'SPIRAL_SEARCH',
                            'teleop': 'TELEOPERATION',
                            'shutdown': 'shutdown',
                            'idle': 'IDLE'
                        })

        # Teleoperation state
        smach.StateMachine.add('TELEOPERATION', Teleop(),
                       transitions={
                           'spiral_search': 'SPIRAL_SEARCH',
                           'autonomous': 'AUTONOMOUS_NAVIGATION',
                           'shutdown': 'shutdown',
                           'idle': 'IDLE'
                       })

        # Spiral search state
        smach.StateMachine.add('SPIRAL_SEARCH', SpiralSearch(),
                               transitions={
                                'teleop': 'TELEOPERATION',
                                'autonomous': 'AUTONOMOUS_NAVIGATION',
                                'shutdown': 'shutdown',
                                'idle': 'IDLE'
                               })

    try:
        # Execute the state machine starting from Idle
        outcome = sm.execute()
        rospy.loginfo(f"State machine executed with outcome: {outcome}")

    except Exception as e:
        rospy.logerr(f"Error occurred during state machine execution: {e}")

if __name__ == '__main__':
    main()