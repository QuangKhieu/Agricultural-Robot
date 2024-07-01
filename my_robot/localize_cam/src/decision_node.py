#!/usr/bin/python3
import rospy
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped

flag_stop = 0
flag_pub = rospy.Publisher("flag_pub", Point, queue_size=10)

def callback(fruit_flag, init_flag):
    global flag_stop
    if fruit_flag.point.x == 1:
        flag_stop == 1
    elif fruit_flag.point.x == 0 and init_flag.point.x == 1:
        flag_stop == 0

if __name__=="__main__":
    rospy.init_node("flag_publisher", anonymous=True)
    
    fruit_subscriber = message_filters.Subscriber("fruit", PointStamped)
    arm_subscriber = message_filters.Subscriber("init", PointStamped)
    ts = message_filters.TimeSynchronizer([fruit_subscriber, arm_subscriber], 10)
    ts.registerCallback(callback)

    while not rospy.is_shutdown():
        flag_msg = Point(flag_stop,0,0)
        flag_pub.publish(flag_msg)

        rospy.sleep(0.01)