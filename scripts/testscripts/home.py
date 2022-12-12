#!/usr/bin/env python3
# coding: utf-8

import moveit_commander
import rospy
import math
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Int16MultiArray

class Aimer:
    def __init__(self):
        gazebo_model_states = ModelStates()
        self.search_pose = [1.8315570317207914,
                1.117091769121374,
                -0.09001392494161564,
                -2.1952402647913614,
                -1.805752043496053,
                1.4472519351793007,
                -2.616679847729068
                ]
        self._sub = rospy.Subscriber("/MouthCoordinate", Int16MultiArray, self.mouth_coordinate_callback)
        self.model_states = rospy.Subscriber("gazebo/model_states", ModelStates, self.model_state_callback, queue_size = 1)

        # 腕グループの取得
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm.set_max_velocity_scaling_factor(0.4)
        self.arm.set_max_acceleration_scaling_factor(1.0)

        # Pose クラスのインスタンス化
        self.target_pose = Pose()

    # MouthCoordinate を Subscribe したら
    def mouth_coordinate_callback(self, _coordinate):
        self.coordinate = _coordinate

    # ModelState を Subscribe したら
    def model_state_callback(self, msg):
        self.gazebo_model_states = msg

    # 基準姿勢にする
    def standard_pose(self):
        self.arm.set_named_target("vertical")
        self.go()

    # go と stop をひとまとめにした
    def go(self):
        self.arm.go()
        self.arm.stop()

    def main(self):
        # self.arm_goal = arm.get_current_joint_values()
        # self.arm.set_joint_value_target(arm_goal)
        self.standard_pose()

if __name__ == '__main__':
    # ROS ノードを初期化する
    rospy.init_node('end_effector_mover')

    # エイムするクラス
    aimer = Aimer()

    # main() メソッドの実行
    aimer.main()

