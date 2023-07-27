#! /usr/bin/env python

import rospy
import moveit_commander
import json
import os
import math
import time

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
        self.pose.pose.orientation.x = -math.sqrt(0.5)
        self.pose.pose.orientation.y = math.sqrt(0.5)
        self.pose.pose.orientation.z = 0
        self.pose.pose.orientation.w = 0
        self.last_modified = None
        self.hand = self.gripper.get_current_joint_values()
        self.positions = []
        self.hands = []
    
    def update(self, event):
        command = [0,0,0,0,0,0]
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

        #print(command)
        if command[0] != 0 or command[1] != 0 or command[2] != 0:
            self.pose.pose.position.x += command[0]*0.01
            self.pose.pose.position.y += command[1]*0.01
            self.pose.pose.position.z += command[2]*0.01
            self.arm.set_pose_target(self.pose)
            self.arm.go()
        
        if command[3] != 0:
            self.hand[0] += command[3]*0.1
            self.hand[1] += command[3]*0.1
            self.gripper.set_joint_value_target(self.hand)
            self.gripper.go()
        
        if command[4] == 1:
            self.positions.append(self.arm.get_current_pose())
            self.hands.append(self.gripper.get_current_joint_values())
            print("SAVE: "+str(len(self.positions)))
        elif command[4] == -1:
            self.positions.pop()
            self.hands.pop()
            self.pose = self.positions[-1]
            self.hand = self.hands[-1]
            self.arm.set_pose_target(self.pose)
            self.arm.go()
            self.gripper.set_joint_value_target(self.hand)
            self.gripper.go()
            print("DELETE: "+str(len(self.positions)))
        elif command[5] == 1:
            for position, hand in zip(self.positions, self.hands):
                print(position)
                self.arm.set_pose_target(position)
                self.arm.go()
                self.gripper.set_joint_value_target(hand)
                self.gripper.go()
            self.pose = self.positions[-1]
            self.hand = self.hands[-1]

if __name__ == '__main__':
    rospy.init_node("crane_x7_teaching_playback")
    teaching_playback = TeachingPlayback()
    timer = rospy.Timer(rospy.Duration(0.1), teaching_playback.update)
    rospy.spin()
