#!/usr/bin/env python3
# coding: utf-8

import moveit_commander
import rospy
import math
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose

gazebo_model_states = ModelStates()

def subcb(msg):
    global gazebo_model_states
    gazebo_model_states = msg

def main():
    global gazebo_model_states

    sub_model_states = rospy.Subscriber("gazebo/model_states", ModelStates, subcb, queue_size = 1)

    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.4)
    arm.set_max_acceleration_scaling_factor(1.0)
    rospy.sleep(1.0)

    arm_joints = arm.get_current_joint_values()
    print("arm_joints = " + str(arm_joints))
    # for link_index, arm_joint in enumerate(arm_joints):
        # print("arm_joint[" + str(link_index) + "] = " + str(arm_joint))

if __name__ == '__main__':
    # ROS ノードを初期化する
    rospy.init_node('end_effector_mover')

    # エンドエフェクタを右に移動する
    main()

