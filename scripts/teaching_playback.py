#! /usr/bin/env python

from ast import mod
import keyboard
import rospy
import moveit_commander
import json
import os
from tf.transformations import quaternion_from_euler

class TeachingPlayback:
    def __init__(self):
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.arm.set_max_velocity_scaling_factor(0.2)
        self.arm.set_max_acceleration_scaling_factor(0.2)

        self.gripper.set_joint_value_target([0.9, 0.9])
        self.gripper.go()

        self.arm.set_named_target("home")
        self.arm.go()

        self.pose = self.arm.get_current_pose()
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]
        self.last_modified = None
        self.hand = self.gripper.get_current_joint_values()
    
    def update(self, event):
        command = [0,0,0,0]
        try:
            if os.path.exists('key_command.json'):
                modified = os.path.getmtime('key_command.json')
                if self.last_modified is None or modified > self.last_modified:
                    self.last_modified = modified
                    with open('key_command.json', 'r') as f:
                        command_dict = json.load(f)
                        command = command_dict['command']
        except Exception as e:
            print("Error occured: {e}")

        print(command)
        self.pose.pose.position.x += command[0]*0.01
        self.pose.pose.position.y += command[1]*0.01
        self.pose.pose.position.z += command[2]*0.01
        self.arm.set_pose_target(self.pose)
        self.arm.go()
        self.hand[0] += command[3]*0.1
        self.hand[1] += command[3]*0.1
        self.gripper.set_joint_value_target(self.hand)
        self.gripper.go()

if __name__ == '__main__':
    rospy.init_node("crane_x7_teaching_playback")
    teaching_playback = TeachingPlayback()
    timer = rospy.Timer(rospy.Duration(0.1), teaching_playback.update)
    rospy.spin()

