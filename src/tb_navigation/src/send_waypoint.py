#! /usr/bin/env python

import rospy
import csv
import actionlib
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

received = ""


def send_wp(x, y, z, ori):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    goal.target_pose.pose.orientation.z = ori
    goal.target_pose.pose.orientation.w = 1

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def callback(msg):
    global received
    received = msg.data
    print received


if __name__ == '__main__':
    try:
        image_sub = rospy.Subscriber("/turtlekin/qr", String, callback)
        pub = rospy.Publisher("/turtlekin/scan", String, queue_size=10)
        with open('/home/mishi9/turtlekin_ws/src/tb_navigation/src/poses.csv') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            rospy.init_node('send_waypoint_py')

            for row in csv_reader:
                print(row)
                result = send_wp(float(row[1]), float(row[2]), float(row[3]), float(row[4]))
                if row[0] == "10":
                    continue
                pub.publish("scan")
                while not received == row[0]:
                    pass
                rospy.loginfo("scan successful: " + received)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
