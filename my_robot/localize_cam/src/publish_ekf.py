#!/usr/bin/python3
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

flag = 0
raw_ekf = Point(0, 0, 0)
pose_pub = rospy.Publisher("ekf_pose", Point, queue_size=10)

def callback(ekf_point):
    global flag
    global raw_ekf

    if flag == 0:
        flag = 1
        raw_ekf = ekf_point
    return flag

if __name__=="__main__":
    rospy.init_node("ekf_flag_publisher", anonymous=True)
    print("EKF pose with flag is being published to the topic /ekf_pose ...")
    
    pose_sub = rospy.Subscriber("no_flag_ekf_pose", Point, callback)  # Subscribe outside the loop
    n = 0
    while not rospy.is_shutdown():
        if n == 1:
            flag = 1
        pose_with_flag = PointStamped()
        pose_with_flag.header = Header()
        #print(flag)
        pose_with_flag.point = Point(raw_ekf.x, raw_ekf.y, flag)
        pose_pub.publish(pose_with_flag.point)
        if n == 1:
            flag = 0
            n = 0

        if flag == 1:
            n = 1

        rospy.sleep(0.01)
