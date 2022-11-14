#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import math
import moveit_commander


def main():
    arm = moveit_commander.MoveGroupCommander("arm")
    # 駆動速度を調整する
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_max_acceleration_scaling_factor(1.0)

    # SRDFに定義されている"vertical"の姿勢にする
    # すべてのジョイントの目標角度が0度になる
    arm.set_named_target("vertical")
    arm.go()

    # 目標角度と実際の角度を確認
    print("joint_value_target (radians):")
    print(arm.get_joint_value_target())
    print("current_joint_values (radians):")
    print(arm.get_current_joint_values())

    # 現在角度をベースに、目標角度を作成する
    # 現在の関節角度をリストとして取得する。
    target_joint_values = arm.get_current_joint_values()
    # 各ジョイントの角度を１つずつ変更する
    # -45 deg = -0.79 rad
    joint_angle = math.radians(-45)
    # 0 <= i < 7
    for i in range(7):
        # target_joint_values は行列なのに、
        # 代入する値はスカラー（-0.79 rad）なので
        # 行列の全要素がこのスカラー値になる。
        # -0.79 rad
        target_joint_values[i] = joint_angle
        # target_joint_values を渡しているのでこれはリスト全体を渡している
        # 関節角度全てが -0.79 rad になる。
        arm.set_joint_value_target(target_joint_values)
        # go(joints=None, wait=True)
        # 目標を設定し、グループを指定された目標に移動します。
        arm.go()
        print(str(i) + "-> joint_value_target (degrees):"
              + str(math.degrees(arm.get_joint_value_target()[i]))
              + ", current_joint_values (degrees):"
              + str(math.degrees(arm.get_current_joint_values()[i]))
        )

    rospy.sleep(3)
    # 垂直に戻す
    arm.set_named_target("vertical")
    arm.go()


if __name__ == '__main__':
    rospy.init_node("joint_values_example")

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
