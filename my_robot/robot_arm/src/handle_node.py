#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped   
from geometry_msgs.msg import Quaternion

goal_pub = rospy.Publisher('goal_pos', Quaternion, queue_size=10)
decision_pub = rospy.Publisher('fruit', PointStamped, queue_size=10)
flag = 0

def callback(obj_pos):
    global flag

    goal_pos_msg = Quaternion()
    goal_pos_msg.x = obj_pos.z
    goal_pos_msg.y = -obj_pos.x - 35
    goal_pos_msg.z = -obj_pos.y + 75
    goal_pos_msg.w = obj_pos.w

    if goal_pos_msg.w == 3 and goal_pos_msg.x < 400:
        goal_pub.publish(goal_pos_msg)
        print(10000)

        if flag == 0:
            flag = 1
    

    
        

if __name__ == '__main__':
    rospy.init_node('handle_node', anonymous=True)

    rospy.Subscriber("obj_pos_tp", Quaternion, callback)
    n = 0
    while not rospy.is_shutdown():
        if n == 1:
            flag = 1
        flag_msg = PointStamped()
        flag_msg.header = Header()
        # print(flag)
        flag_msg.point = Point(flag,0,0)
        decision_pub.publish(flag_msg)
        if n == 1:
            flag = 0
            n = 0

        if flag == 1:
            n = 1

        rospy.sleep(0.01)
