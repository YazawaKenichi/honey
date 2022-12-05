#!/usr/bin/env python3
# coding: utf-8

import moveit_commander
import rospy
import math
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose

gazebo_model_states = ModelStates()
arm_joints = []

def init():
    global arm_joints
    arm_joints = [1.8315570317207914,
            1.117091769121374,
            -0.09001392494161564,
            -2.1952402647913614,
            -1.805752043496053,
            1.4472519351793007,
            -2.616679847729068
            ]

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

    # 基準姿勢にする
    arm.set_named_target("vertical")
    arm.go()
    rospy.sleep(1.0)

    # arm_goal = arm.get_current_joint_values()
    # arm.set_joint_value_target(arm_goal)
    arm.go(arm_joints)
    arm.stop()

if __name__ == '__main__':
    # ROS ノードを初期化する
    rospy.init_node('end_effector_mover')

    init()
    # エンドエフェクタを右に移動する
    main()

