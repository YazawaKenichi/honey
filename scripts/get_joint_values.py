#! /usr/bin/evn python
# coding: utf-8

import rospy
import moveit_commander
from geometry_

def main():
    SLEEP_TIME = 3.0
    print('Wait ' + str(SLEEP_TIME) + ' secs.')
    rospy.sleep(SLEEP_TIME)
    print('Start')
    while True:

if __name__ == '__main__':
    rospy.init_node('get_joint_values')

    try:
        if not rospy.is_shutdown():
            main()
        except rospy.ROSInterruptException:
            pass


