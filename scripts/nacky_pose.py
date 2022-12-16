#!/usr/bin/env python3
# coding: utf-8

# Image Size : [960, 540]
# Image Center : [480, 270]
# Target Coordinate : [347.5, 294]

import moveit_commander
import rospy
import math
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
# クォータニオンからオイラー角への変換に必要
import tf
from geometry_msgs.msg import Vector3, Quaternion

cont = True

# Quaternion to Euler
def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

# Euler to Quaternion
def euler_to_quaternion(euler):
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def yaw_of(object_orientation):
    # クウォータニオンをオイラー角に変換し yaw 角度を返す
    euler = euler_from_quaternion((object_orientation.x, object_orientation.y, object_orientation.z, object_orientation.w))
    return euler[2]

class Aimer:
    def __init__(self):
        # ModelStates クラスのインスタンス化
        self.gazebo_model_states = ModelStates()
        # オブジェクトの位置姿勢を取得
        # self._sub = rospy.Subscriber("/MouthCoordinate", Float32MultiArray, self.mouth_coordinate_callback)
        self.sub_model_states = rospy.Subscriber('gazebo/model_states', ModelStates, self.model_state_callback, queue_size = 1)

        # 腕グループの取得
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm.set_max_velocity_scaling_factor(0.4)
        self.arm.set_max_acceleration_scaling_factor(1.0)

        # Pose クラスのインスタンス化
        # self.target_pose = Pose()
        self.target_pose = []

    # オブジェクトの位置姿勢を取得
    def get_crane_pose(self):
        OBJECT_NAME = 'crane_x7'
        # オブジェクト名と一致する場所のインデックス番号を 'object_index' に格納する
        self.object_index = self.gazebo_model_states.name.index(OBJECT_NAME)
        # オブジェクトの位置を取得
        self.object_position = self.gazebo_model_states.pose[self.object_index].position
        # オブジェクトの姿勢を取得
        self.object_orientation = self.gazebo_model_states.pose[self.object_index].orientation

    # MouthCoordinate を Subscribe したら
    def mouth_coordinate_callback(self):
        self.get_crane_pose()
        # 目標値を設定する
        self.target_pose = [
                0,
                0,
                0.25,
                1.64,
                -0.03,
                -1.57]
        # 目標値を渡す
        self.arm.set_pose_target(self.target_pose)
        # エンドエフェクタを移動する
        self.go()

    # ModelState を Subscribe したら
    def model_state_callback(self, msg):
        self.gazebo_model_states = msg
        self.get_crane_pose()
        self.mouth_coordinate_callback()

    # go と stop をひとまとめにした
    def go(self):
        self.arm.go()
        self.arm.stop()

    def main(self):
        print("Aiming ...")
        rospy.spin()

if __name__ == '__main__':
    # ROS ノードを初期化する
    rospy.init_node('end_effector_mover')

    # エイムするクラス
    aimer = Aimer()

    try:
        if not rospy.is_shutdown():
            # main() メソッドの実行
            aimer.main()
    except rospy.ROSInterruptException:
        pass

