#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
import actionlib
import geometry_msgs.msg
import rosnode
from tf import transformations
from tf.transformations import quaternion_from_euler
import math
from geometry_msgs.msg import Point
from robotdesign3_2021_1.msg import CustomArray
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)

class Home(object):
    def __init__(self):
        rospy.init_node("Pose_MediaPipe")
        rospy.Subscriber('/hand_topic', CustomArray, self.callback, queue_size=1)
        self._client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self._goal = GripperCommandGoal()
        self.robot = moveit_commander.RobotCommander()
        self.array_points = CustomArray()
        
        self._client.wait_for_server(rospy.Duration(10.0))
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("txiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        self.clear()
        
        self.hand1_x = 0
        self.hand1_y = 0
        self.hand1_z = 0

        self.hand2_x = 0
        self.hand2_y = 0
        self.hand2_z = 0
        
        self.hand3_x = 0
        self.hand3_y = 0
        self.hand3_z = 0

        self.hand4_x = 0
        self.hand4_y = 0
        self.hand4_z = 0

    def command(self, position, effort):
        # self._goal = GripperCommandGoal() で定義されている
        # GripperCommandGoal().position で姿勢を決定する
        # GripperCommandGoal().max_effort でトルクを決定する
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal,feedback_cb=self.feedback)

    def callback(self, msg):
        
        self.hand1_x = msg.points[0].x
        self.hand1_y = msg.points[0].y
        self.hand1_z = msg.points[0].z

        self.hand2_x = msg.points[1].x
        self.hand2_y = msg.points[1].y
        self.hand2_z = msg.points[1].z

        self.hand3_x = msg.points[2].x
        self.hand3_y = msg.points[2].y
        self.hand3_z = msg.points[2].z

        self.hand4_x = msg.points[3].x
        self.hand4_y = msg.points[3].y
        self.hand4_z = msg.points[3].z

    def feedback(self,msg):
        print("feedback callback")
        print(msg)

    def stop(self):
        self._client.cancel_goal()
        
    def wait(self, timeout=0.1 ):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def clear(self):
        # GripperCommandGoal() は control_msgs.msg からインポートしている
        # GripperCommandGoal 型の self._goal を定義初期化している
        # すなわち呼び出すたびにゴールの情報が初期化できる
        self._goal = GripperCommandGoal()

    def manipulation(self):
        # 自分クラスをインスタンス化して 
        gc = Home()
        # crane_x7_moveit_config/config/crane_x7.srdf 内で定義されたリンクのグループ "arm" を指定してそれらをインスタンス化
        arm = moveit_commander.MoveGroupCommander("arm")
        gripper = moveit_commander.MoveGroupCommander("gripper")
        #安全性のため速度を1/2ほどに設定
        arm.set_max_velocity_scaling_factor(0.25)
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        arm.set_max_acceleration_scaling_factor(1.0)
        # 今のジョイントの角度を取得
        target_joint_values = arm.get_current_joint_values()
        print(target_joint_values)
        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)
        # raw_input() は Python2 で使用する標準入力
        # Python3 なら input() で問題ない
        key = raw_input()

        if key == "o":
            print("Open Gripper.")
            # if 内スコープで gripper を作成して 45.0 度を選択
            gripper = 45.0
            # class Home(): command(position, effort): グリッパの姿勢とトルクを決定する
            gc.command(math.radians(gripper),1.0)
            # gc = Home クラス自身
            result = gc.wait(2.0)
            print(result)
            rospy.sleep(1.0)
            print("")

        if key == "c":
            print("Close Gripper.")
            gripper = 0.0 # 0.0
            gc.command(math.radians(gripper),1.0)
            result = gc.wait(2.0)
            print(result)
            rospy.sleep(1.0)

        if key == "h":   
            print("="*30)
            print("home_mode")
            #ホームポジションの初期姿勢を決め打ちでセット
            target_joint_values[0] = math.radians(0)
            target_joint_values[1] = math.radians(0)
            target_joint_values[2] = math.radians(0)
            target_joint_values[3] = math.radians(-90)
            target_joint_values[4] = math.radians(0)
            target_joint_values[5] = math.radians(-85)
            target_joint_values[6] = math.radians(-90)
            arm.set_joint_value_target(target_joint_values) #初期姿勢をマニピュレータにセット
            arm.go()#初期位置へ移動開始

            print("-"*20)
            print("move to home")
            arm_goal_pose = arm.get_current_pose().pose
            print("Arm goal pose:")
            print(arm_goal_pose)
            print("-"*20)

            rospy.sleep(0.5)
            target_joint_values[6] = math.radians(-90)
            arm.set_joint_value_target(target_joint_values)
            arm_goal_pose = arm.get_current_pose().pose
            arm.go()
            print(arm_goal_pose)


        if key == "q":
            hand1_x_buff = 0.26 + self.hand1_y - 0.045
            hand1_y_buff = (self.hand1_x + 0.05)
            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = hand1_x_buff
            target_pose.position.y = hand1_y_buff
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.32
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm.set_pose_target( target_pose )
            arm.go()
            rospy.sleep(1.0)

            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = hand1_x_buff
            target_pose.position.y = hand1_y_buff
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.25
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            
            arm.set_pose_target( target_pose )
            arm.go()

            rospy.sleep(1.0)

            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = hand1_x_buff
            target_pose.position.y = hand1_y_buff
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.32
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            
            arm.set_pose_target( target_pose )
            arm.go()

        if key == "l":
            while True:
                hand1_x_buff = 0.26 + self.hand1_y - 0.040
                hand1_y_buff = (self.hand1_x + 0.05)
                target_pose = geometry_msgs.msg.Pose()
                target_joint_values = arm.get_current_joint_values()
                target_pose.position.x = hand1_x_buff
                target_pose.position.y = hand1_y_buff
                print(target_pose.position.x, target_pose.position.y)
                target_pose.position.z = 0.32
                q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
                target_pose.orientation.x = q[0]
                target_pose.orientation.y = q[1]
                target_pose.orientation.z = q[2]
                target_pose.orientation.w = q[3]
                arm.set_pose_target( target_pose )
                arm.go()
                rospy.sleep(1.0)


        if key == "w":
            hand2_x_buff = 0.26 + self.hand2_y - 0.040
            hand2_y_buff = (self.hand2_x + 0.062)
            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = hand2_x_buff
            target_pose.position.y = hand2_y_buff
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.32
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm.set_pose_target( target_pose )
            arm.go()

            rospy.sleep(1.0)

            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = hand2_x_buff
            target_pose.position.y = hand2_y_buff
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.25
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            
            arm.set_pose_target( target_pose )
            arm.go()

            rospy.sleep(1.0)

            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = hand2_x_buff
            target_pose.position.y = hand2_y_buff
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.32
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            
            arm.set_pose_target( target_pose )
            arm.go()

        if key == "e":
            hand3_x_buff = 0.26 + self.hand3_y - 0.048
            hand3_y_buff = (self.hand3_x + 0.075)
            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = hand3_x_buff
            target_pose.position.y = hand3_y_buff
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.32
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm.set_pose_target( target_pose )
            arm.go()

            rospy.sleep(1.0)

            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = hand3_x_buff
            target_pose.position.y = hand3_y_buff
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.25
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            
            arm.set_pose_target( target_pose )
            arm.go()

            rospy.sleep(1.0)

            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = hand3_x_buff
            target_pose.position.y = hand3_y_buff
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.32
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            
            arm.set_pose_target( target_pose )
            arm.go()

        if key == "r":
            hand4_x_buff = 0.26 + self.hand4_y - 0.033
            hand4_y_buff = (self.hand4_x + 0.08)
            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = hand4_x_buff
            target_pose.position.y = hand4_y_buff
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.32
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm.set_pose_target( target_pose )
            arm.go()

            rospy.sleep(1.0)

            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = hand4_x_buff
            target_pose.position.y = hand4_y_buff
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.25
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            
            arm.set_pose_target( target_pose )
            arm.go()

            rospy.sleep(1.0)

            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = hand4_x_buff
            target_pose.position.y = hand4_y_buff
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.32
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            
            arm.set_pose_target( target_pose )
            arm.go()

        if key == "f":
            print("vertical")
            arm.set_named_target("vertical")
            arm.go()
            
    def run(self):
        try:
            while not rospy.is_shutdown():
                self.manipulation()
        except rospy.ROSInterruptException:
            pass
if __name__ == '__main__':
    while not rospy.is_shutdown():
            Home().run()
