#!/usr/bin/env python3

import rospy
import moveit_commander
import json
import os
import math
import copy
from moveit_msgs.msg import JointConstraint
from tf.transformations import quaternion_from_euler

def limit_value(value, min_value, max_value):
    if value < min_value:
        print("Out of range")
        return min_value
    elif value > max_value:
        print("Out of range")
        return max_value
    return value

class TeachingPlayback:
    def __init__(self):
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.arm.set_max_velocity_scaling_factor(0.2)
        self.arm.set_max_acceleration_scaling_factor(0.2)

        self.reset_waiting = False
        self.last_modified = None
        self.positions = []
        self.hands = []

        self.set_home_pose()
        self.pose = self.arm.get_current_pose()
        self.set_downward_orientation()
        self.hand = self.gripper.get_current_joint_values()

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "crane_x7_upper_arm_revolute_part_twist_joint"
        joint_constraint.position = 0
        joint_constraint.tolerance_above = 0.1
        joint_constraint.tolerance_below = 0.1
        joint_constraint.weight = 0.5
        constraints = moveit_commander.Constraints()
        constraints.joint_constraints.append(joint_constraint)
        self.arm.set_path_constraints(constraints)

    def set_downward_orientation(self):
        q = quaternion_from_euler(0, math.pi, 0)
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]

    def set_home_pose(self):
        self.gripper.set_joint_value_target([0.9, 0.9])
        if not self.gripper.go(wait=True):
            rospy.logwarn("Failed to move gripper to home")
        self.arm.set_named_target("home")
        if not self.arm.go(wait=True):
            rospy.logwarn("Failed to move arm to home")

    def reset_to_vertical(self):
        rospy.loginfo("Resetting to vertical pose...")
        self.arm.set_named_target("vertical")
        if not self.arm.go(wait=True):
            rospy.logwarn("Failed to move arm to vertical")
        self.reset_waiting = True

    def update(self, event):
        command = [0, 0, 0, 0, 0, 0]
        try:
            if os.path.exists('key_command.json'):
                modified = os.path.getmtime('key_command.json')
                if self.last_modified is None or modified > self.last_modified:
                    self.last_modified = modified
                    with open('key_command.json', 'r') as f:
                        content = f.read()
                        if not content.strip():
                            print("Empty command file.")
                            return
                        command_dict = json.loads(content)
                        command = command_dict['command']
        except Exception as e:
            print("Error occurred: {}".format(e))
            return

        if self.reset_waiting:
            if any(c != 0 for c in command):
                rospy.loginfo("Input detected. Returning to home.")
                self.set_home_pose()
                self.pose = self.arm.get_current_pose()
                self.set_downward_orientation()
                self.hand = self.gripper.get_current_joint_values()
                self.reset_waiting = False
            else:
                return

        if command[0] != 0 or command[1] != 0 or command[2] != 0:
            pos = self.pose.pose.position
            pos.x = limit_value(pos.x + command[0] * 0.01, 0.10, 0.27)
            pos.y = limit_value(pos.y + command[1] * 0.01, -0.25, 0.25)
            pos.z = limit_value(pos.z + command[2] * 0.01, 0.10, 0.31)
            self.arm.set_pose_target(self.pose)
            success, plan, _, _ = self.arm.plan()
            if success:
                self.arm.execute(plan, wait=True)
            else:
                rospy.logwarn("Failed to plan for pose target")

        if command[3] != 0:
            self.hand[0] = limit_value(self.hand[0] + command[3] * 0.1, 0.5, 0.9)
            self.hand[1] = self.hand[0]
            self.gripper.set_joint_value_target(self.hand)
            if not self.gripper.go(wait=True):
                rospy.logwarn("Failed to move gripper")

        if command[4] == 1:
            self.positions.append(copy.deepcopy(self.pose))
            self.hands.append(copy.deepcopy(self.hand))
            print("SAVE: {}".format(len(self.positions)))

        elif command[4] == -1:
            if self.positions:
                if len(self.positions) > 1:
                    target_pose = copy.deepcopy(self.positions[-2])
                    target_hand = copy.deepcopy(self.hands[-2])
                    self.positions.pop()
                    self.hands.pop()
                    self.arm.set_pose_target(target_pose)
                    success, plan, _, _ = self.arm.plan()
                    if success:
                        self.arm.execute(plan, wait=True)
                    else:
                        rospy.logwarn("Failed to plan to pose after delete")
                    self.gripper.set_joint_value_target(target_hand)
                    if not self.gripper.go(wait=True):
                        rospy.logwarn("Failed to move gripper after delete")
                    self.pose = target_pose
                    self.hand = target_hand
                else:
                    self.positions.pop()
                    self.hands.pop()
                    self.set_home_pose()
                    self.pose = self.arm.get_current_pose()
                    self.set_downward_orientation()
                    self.hand = self.gripper.get_current_joint_values()
                print("DELETE: {}".format(len(self.positions)))
            else:
                print("Nothing to delete")

        elif command[5] == 1:
            if self.positions:
                for i, (position, hand) in enumerate(zip(self.positions, self.hands), start=1):
                    print("INDEX: {}".format(i))
                    self.arm.set_pose_target(position)
                    success, plan, _, _ = self.arm.plan()
                    if success:
                        self.arm.execute(plan, wait=True)
                    else:
                        rospy.logwarn("Failed to plan during playback")
                    self.gripper.set_joint_value_target(hand)
                    if not self.gripper.go(wait=True):
                        rospy.logwarn("Failed to move gripper during playback")
                self.pose = copy.deepcopy(self.positions[-1])
                self.hand = copy.deepcopy(self.hands[-1])
            else:
                print("Nothing to playback")

        elif command[5] == -1:
            self.positions = []
            self.hands = []
            self.reset_to_vertical()

if __name__ == '__main__':
    rospy.init_node("crane_x7_teaching_playback")
    moveit_commander.roscpp_initialize([])
    teaching_playback = TeachingPlayback()
    timer = rospy.Timer(rospy.Duration(0.1), teaching_playback.update)
    try:
        os.system('stty -echo')
        rospy.spin()
    finally:
        os.system('stty echo')
        moveit_commander.roscpp_shutdown()
