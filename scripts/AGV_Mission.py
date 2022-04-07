#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
#roslib.load_manifest('simple_navigation_goals_tutorial')
import rospy
import actionlib
import time

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Int64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from threading import *
import paho.mqtt.client as mqtt
from collections import OrderedDict
import json
import random
import time
import threading
import ast


class AGV_Mqtt():
    def __init__(self):
        self.MQTTHOST = "192.168.10.124"  # 服务器ip
        self.MQTTPORT = 50002  # 服务器端口
        self.mqttClient = mqtt.Client()
        self.AGV_Mission_Msg = 0
        self.AGV_Move_State = "front"
        #self.start_mqtt()

    def mqtt_conncet(self):
        self.mqttClient.connect(self.MQTTHOST, self.MQTTPORT, 60)

    # publish 消息
    def on_publish(self, payload, topic="/HG_DEV/ZK_ALL_REQ", qos=2):
        print("pub msg: %s to %s" % (payload, topic))
        self.mqttClient.publish(topic, payload, qos)
        #time.sleep(2.0)

    # 对mqtt订阅消息处理函数
    def on_message_come(self, lient, userdata, msg):
        # python3 bytes转str
        self.sub_msg = str(msg.payload, 'utf-8')
        print("sub msg: %s" % self.sub_msg)
        self.sub_msg = ast.literal_eval(self.sub_msg)
        #   去岸吊上货
        if self.sub_msg["name"] == "ZK" and self.sub_msg["dir"] == "AGV" and self.sub_msg["mission"] == 1:
            self.AGV_Mission_Msg = 1
        #   退出岸吊
        if self.sub_msg["name"] == "ZK" and self.sub_msg["dir"] == "AGV" and self.sub_msg["mission"] == 2:
            self.AGV_Mission_Msg = 2
        #   去门吊卸货
        if self.sub_msg["name"] == "ZK" and self.sub_msg["dir"] == "AGV" and self.sub_msg["mission"] == 3:
            self.AGV_Mission_Msg = 3
        #   回起始点
        if self.sub_msg["name"] == "ZK" and self.sub_msg["dir"] == "AGV" and self.sub_msg["mission"] == 4:
            self.AGV_Mission_Msg = 4
        #   中控控制AGV运动
        if self.sub_msg["name"] == "ZK" and self.sub_msg["dir"] == "AGV" and self.sub_msg["mission"] == 9:
            self.AGV_Mission_Msg = 9
            self.AGV_Move_State = self.sub_msg["move"]

    # mqtt subscribe 消息
    def on_subscribe(self):
        self.mqttClient.subscribe("/HG_DEV/ZK_ALL", 2)
        self.mqttClient.subscribe("/HG_DEV/AGV_MOVE", 2)
        self.mqttClient.on_message = self.on_message_come  # 消息到来处理函数

    def start_mqtt(self):
        self.mqtt_conncet()
        self.on_subscribe()
        self.mqttClient.loop_start()


class AGV_Mission():
    def __init__(self):
        rospy.init_node('send_goals_node', anonymous=False)
        self.host_ip = rospy.get_param("~host_ip", "192.168.3.6")
        self.host_port = rospy.get_param("~host_port", "50001")

        self.agv_req_msg = OrderedDict()
        self.agv_req_msg["name"] = "AGV"
        self.agv_req_msg["dir"] = "ZK"
        self.agv_req_msg["mission"] = 1
        self.agv_req_msg["state"] = "working"
        self.agv_req_json = json.dumps(self.agv_req_msg)

        self.agv_move_msg = Twist()
        self.mission_nub = 0
        self.agv_move_target = 0
        self.agv_target_state = False
        self.odom_x, self.odom_y = None, None
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('xqserial_server/Odom', Odometry, self.agv_odom)

        self.AGV_Mqtt = AGV_Mqtt()
        self.AGV_Mqtt.mqtt_host = self.host_ip
        self.AGV_Mqtt.mqtt_port = self.host_port
        self.AGV_Mqtt.start_mqtt()

        #   开启任务线程
        self.th = Thread(target=self.get_mission)
        self.th.start()

    def get_mission(self):
        while True:
            # 去岸吊
            if self.AGV_Mqtt.AGV_Mission_Msg == 1:
                self.AGV_Mqtt.AGV_Mission_Msg = 0
                self.mission_state_req(1, "working")
                self.mission_nub = 1
                self.and_goal(self.agv_move_target)
                self.and_goal(self.agv_move_target+1)
                self.and_goal(self.agv_move_target+2)
                self.odom_and = self.odom_y
                self.mission_state_req(self.mission_nub, "finish")
            #   退出岸吊
            if self.AGV_Mqtt.AGV_Mission_Msg == 2:
                self.AGV_Mqtt.AGV_Mission_Msg = 0
                self.mission_state_req(2, "working")
                self.mission_nub = 2
                self.and_back_goal()
                self.mission_state_req(self.mission_nub, "finish")
            #   去门吊
            if self.AGV_Mqtt.AGV_Mission_Msg == 3:
                self.AGV_Mqtt.AGV_Mission_Msg = 0
                self.mission_state_req(3, "working")
                self.mission_nub = 3
                self.md_goal(0)
                self.md_goal(1)
                self.mission_state_req(self.mission_nub, "finish")
            #   回起始点
            if self.AGV_Mqtt.AGV_Mission_Msg == 4:
                self.AGV_Mqtt.AGV_Mission_Msg = 0
                self.mission_state_req(4, "working")
                self.mission_nub = 4
                self.md_back_goal(0)
                self.goHome_goal(0)
                self.goHome_goal(1)
                self.mission_state_req(self.mission_nub, "finish")
            #   中控系统控制AGV
            if self.AGV_Mqtt.AGV_Mission_Msg == 9:
                self.AGV_Mqtt.AGV_Mission_Msg = 0
                self.AGV_Move_CMD(self.AGV_Mqtt.AGV_Move_State)

    #----- 更新任务状态 -----#
    def mission_state_req(self, mission_nub, state):
        self.agv_req_msg["mission"] = mission_nub
        self.agv_req_msg["state"] = state
        self.agv_req_json = json.dumps(self.agv_req_msg)
        self.AGV_Mqtt.on_publish(self.agv_req_json, "/HG_DEV/ZK_ALL_REQ", 2)

    # ----- AGV cmd_vel控制 -----#
    def AGV_Move_CMD(self, move_state):
        #   前进
        if move_state == "front":
            self.agv_move_msg.linear.x = 0.1
            self.agv_move_msg.angular.z = 0.0
        #   后退
        if move_state == "back":
            self.agv_move_msg.linear.x = -0.1
            self.agv_move_msg.angular.z = 0.0
        #   左转
        if move_state == "left":
            self.agv_move_msg.linear.x = 0.0
            self.agv_move_msg.angular.z = 0.1
        #   右转
        if move_state == "right":
            self.agv_move_msg.linear.x = 0.0
            self.agv_move_msg.angular.z = -0.1
        #   停止
        if move_state == "stop":
            self.agv_move_msg.linear.x = 0.0
            self.agv_move_msg.angular.z = 0.0
        self.cmd_vel.publish(self.agv_move_msg)


    # -----导航相关 ------#
    def agv_odom(self, pose):
        self.odom_x = pose.pose.pose.position.x
        self.odom_y = pose.pose.pose.position.y
        #print(self.odom_x, self.odom_y)

    def nav_id(self, data):
        self.nav_id_result = data.data

    #   去岸吊导航函数
    def and_goal(self, i):
        waypointsx = list([-0.89467215538, -2.82806181908, -2.79664230347])
        waypointsy = list([-4.17809247971, -4.48643016815, -5.96632671356])

        waypointsaw = list([0.999998178138, -0.709198683441, -0.703590820781])
        waypointsw = list([-0.00190885348296, 0.705008671865, 0.710605345401])
        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        self.ac.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server");

        # 初始化goal为MoveBaseGoal类型
        goal = MoveBaseGoal()

        # 使用map的frame定义goal的frame id
        goal.target_pose.header.frame_id = 'map'

        # 设置时间戳
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置目标位置
        goal.target_pose.pose.position.x = waypointsx[i]
        goal.target_pose.pose.position.y = waypointsy[i]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = waypointsaw[i]
        goal.target_pose.pose.orientation.w = waypointsw[i]
        rospy.loginfo("Sending goal")
        # 机器人移动
        self.move(goal)

    #   后退导航函数
    def and_back_goal(self):
        move_cmd = Twist()
        while (True):
            if ((self.odom_and - self.odom_y) < 1.4 and (self.odom_and - self.odom_y) > 0) or (
                    (self.odom_and - self.odom_y) > -1.4 and (self.odom_and - self.odom_y) < 0):
                move_cmd.linear.x = -0.2
                self.cmd_vel.publish(move_cmd)
            else:
                move_cmd.linear.x = 0.0
                self.cmd_vel.publish(move_cmd)
                break
        #self.mission_state_req(self.mission_nub, "finish")


    #   门吊（塔吊）导航函数
    def md_goal(self, i):
        waypointsx = list([-2.82806181908, -2.46146583557])
        waypointsy = list([-4.48643016815, -2.83102750778])

        waypointsaw = list([0.709198683441, 0.712895062793])
        waypointsw = list([0.705008671865, 0.701270724789])

        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        self.ac.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server");

        # 初始化goal为MoveBaseGoal类型
        goal = MoveBaseGoal()

        # 使用map的frame定义goal的frame id
        goal.target_pose.header.frame_id = 'map'

        # 设置时间戳
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置目标位置
        goal.target_pose.pose.position.x = waypointsx[i]
        goal.target_pose.pose.position.y = waypointsy[i]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = waypointsaw[i]
        goal.target_pose.pose.orientation.w = waypointsw[i]
        rospy.loginfo("Sending goal")
        # 机器人移动
        self.move(goal)

    #   后退导航函数
    def md_back_goal(self, i):
        move_cmd = Twist()
        while (True):
            if ((((self.odom_y - self.odom_md) < 1.3) and ((self.odom_y - self.odom_md) > 0)) or (
                    ((self.odom_y - self.odom_md) > -1.3) and ((self.odom_y - self.odom_md) < 0))):
                move_cmd.linear.x = -0.2
                self.cmd_vel.publish(move_cmd)
            else:
                move_cmd.linear.x = 0.0
                self.i = 1
                self.cmd_vel.publish(move_cmd)
                break

    #   起始点导航函数
    def goHome_goal(self, i):
        waypointsx = list([-1.15250444412, -0.998461604118])
        waypointsy = list([-4.45599317551, -3.05611181259])

        waypointsaw = list([0.721919542861, 0.72580535784])
        waypointsw = list([0.691977003689, 0.687900125404])


        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        self.ac.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server");

        # 初始化goal为MoveBaseGoal类型
        goal = MoveBaseGoal()

        # 使用map的frame定义goal的frame id
        goal.target_pose.header.frame_id = 'map'

        # 设置时间戳
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置目标位置
        goal.target_pose.pose.position.x = waypointsx[i]
        goal.target_pose.pose.position.y = waypointsy[i]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = waypointsaw[i]
        goal.target_pose.pose.orientation.w = waypointsw[i]
        rospy.loginfo("Sending goal")
        # 机器人移动
        self.move(goal)

    def move(self, goal):
        self.ac.send_goal(goal)

        # 设定5分钟的时间限制
        finished_within_time = self.ac.wait_for_result(rospy.Duration(300))

        # 如果5分钟之内没有到达，放弃目标
        if not finished_within_time:
            self.ac.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.ac.get_state()
            if state == GoalStatus.SUCCEEDED:
                self.agv_target_state = True
                
                self.odom_md = self.odom_y
                #   发布导航成功的flag
                rospy.loginfo("You have reached the goal!")

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        # Cancel any active goals
        self.ac.cancel_goal()


if __name__ == '__main__':
    try:
        AGV_Mission()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
