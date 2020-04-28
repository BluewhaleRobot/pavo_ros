#!/usr/bin/env python
# encoding=utf-8

"""
rplidar节点管理程序。在rplidar节点运行不正常的时候重启rpliar节点
"""

import rospy
import roslaunch
import time
from galileo_serial_server.msg import GalileoStatus

LAST_UPDATE_TIME = 0
VISUAL_FLAG = False


def get_scan(scan):
    global LAST_UPDATE_TIME
    LAST_UPDATE_TIME = int(time.time())


def get_galileo_status(status):
    global VISUAL_FLAG
    if status.visualStatus == -1:
        VISUAL_FLAG = False
    else:
        VISUAL_FLAG = True


if __name__ == "__main__":
    rospy.init_node("rplidar_manager")
    rospy.Subscriber("/scan", rospy.AnyMsg, get_scan)
    rospy.Subscriber("/galileo/status", GalileoStatus, get_galileo_status)
    rospy.set_param("~keep_running", True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    rplidar_launch = roslaunch.parent.ROSLaunchParent(
        uuid, ["/home/xiaoqiang/Documents/ros/src/pavo_ros/launch/xiaoqiang.launch"])
    rplidar_launch.start()
    rospy.loginfo("rplidar started")
    while not rospy.is_shutdown():
        time.sleep(5)
        keep_running_flag = rospy.get_param("~keep_running", True)
        # 处于导航状态下且无法收到雷达数据
        if int(time.time()) - LAST_UPDATE_TIME > 5 and VISUAL_FLAG and keep_running_flag:
            rospy.logerr("restart rplidar node")
            rplidar_launch.shutdown()
            roslaunch.configure_logging(uuid)
            rplidar_launch = roslaunch.parent.ROSLaunchParent(
                uuid, ["/home/xiaoqiang/Documents/ros/src/pavo_ros/launch/xiaoqiang.launch"])
            rplidar_launch.start()
        # 在停用状态下却有雷达数据
        if int(time.time()) - LAST_UPDATE_TIME < 5 and not keep_running_flag:
            rplidar_launch.shutdown()
