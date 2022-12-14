#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# このプログラムは crane_x7_example の gripper_action_example.py を一部改変したものです。

# Copyright 2018 RT Corporation
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

# このサンプルは実機動作のみに対応しています
# fake_execution:=trueにすると、GripperCommandActionのサーバが立ち上がりません

import sys
import rospy
import time
import actionlib
import math

# eddited
import random

from std_msgs.msg import Float64
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)

class GripperClient(object):
    def __init__(self):
        # 命令を送信して返信をもらう通信を実現する
        self._client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self._goal = GripperCommandGoal()

        # Wait 10 Seconds for the gripper action server to start or exit
        self._client.wait_for_server(rospy.Duration(10.0))
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        self.clear()

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal,feedback_cb=self.feedback)

    def feedback(self,msg):
        print("feedback callback")
        print(msg)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=0.1 ):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()

def main():
    gc = GripperClient()
    # eddited
    t = 0
    f = 2   # Hz
    w = 2 * math.pi * f
    dt = 1 / 10

    while True: 
        t = t + dt
        gripper = 90 * (math.sin(w * t) + 1) / 2
        print("gripper = " + str(gripper))

        print("Open Gripper.")
        gc.command(math.radians(gripper),1.0)
        result = gc.wait(2.0)
        print(result)
        time.sleep(dt)
        print("")

if __name__ == "__main__":
    rospy.init_node("hoge")
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

