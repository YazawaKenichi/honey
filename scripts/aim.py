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

cont = True

def yaw_of(object_orientation):
    # クウォータニオンをオイラー角に変換し yaw 角度を返す
    euler = euler_from_quaternion((object_orientation.x, object_orientation.y, object_orientation.z, object_orientation.w))
    return euler[2]

class Aimer:
    def __init__(self):
        # ModelStates クラスのインスタンス化
        self.gazebo_model_states = ModelStates()
        # オブジェクトの位置姿勢を取得
        self._sub = rospy.Subscriber("/MouthCoordinate", Float32MultiArray, self.mouth_coordinate_callback)
        self.sub_model_states = rospy.Subscriber('gazebo/model_states', ModelStates, self.model_state_callback, queue_size = 1)

        # 腕グループの取得
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm.set_max_velocity_scaling_factor(0.4)
        self.arm.set_max_acceleration_scaling_factor(1.0)

        # Pose クラスのインスタンス化
        self.target_pose = Pose()

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
    def mouth_coordinate_callback(self, _coordinate):
        self.get_crane_pose()
        self.coordinate = _coordinate.data
        # 関節座標系 x 軸正方向はカメラ画面右向き
        # print("  position.x : " + str(self.object_position.x) + " -> " + str(self.object_position.x - self.coordinate[0]))
        self.target_pose.position.x = self.object_position.x - self.coordinate[0]
        print("  position.x : " + str(self.object_position.x - self.target_pose.position.x))
        # 関節座標系 y 軸正方向はカメラ画面上向き
        # print("  position.y : " + str(self.object_position.y) + " -> " + str(self.object_position.y + self.coordinate[1]))
        self.target_pose.position.y = self.object_position.y + self.coordinate[1]
        print("  position.y : " + str(self.object_position.x - self.target_pose.position.x))
        # 関節座標系位置 xy 以外を揃える
        # print("  position.z : " + str(self.object_position.z) + " -> " + str(self.object_position.z))
        print("  position.z : " + str(self.object_position.x - self.target_pose.position.x))
        self.target_pose.position.z = self.object_position.z
        # 関節座標系姿勢 xyzw を揃える
        object_yaw = yaw_of(self.object_orientation)
        q = quaternion_from_euler(- math.pi, 0.0, object_yaw)
        # print("quaternion.x : " + str(self.object_orientation.x) + " -> " + str(q[0]))
        print("  position.x : " + str(self.object_position.x - self.target_pose.position.x))
        # print("quaternion.y : " + str(self.object_orientation.y) + " -> " + str(q[1]))
        print("  position.y : " + str(self.object_position.x - self.target_pose.position.x))
        # print("quaternion.z : " + str(self.object_orientation.z) + " -> " + str(q[2]))
        print("  position.z : " + str(self.object_position.x - self.target_pose.position.x))
        # print("quaternion.w : " + str(self.object_orientation.w) + " -> " + str(q[3]))
        print("  position.w : " + str(self.object_position.x - self.target_pose.position.x))
        self.target_pose.orientation.x = q[0]
        self.target_pose.orientation.y = q[1]
        self.target_pose.orientation.z = q[2]
        self.target_pose.orientation.w = q[3]
        # 目標値を渡す
        self.arm.set_pose_target(self.target_pose)
        # エンドエフェクタを移動する
        self.go()

    # ModelState を Subscribe したら
    def model_state_callback(self, msg):
        self.gazebo_model_states = msg
        self.get_crane_pose()

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

