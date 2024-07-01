#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header

def publish_virtual_point():
    pub = rospy.Publisher("test", Point, queue_size=10)
    while not rospy.is_shutdown():
        try:
            point_msg = PointStamped()
            point_msg.header = Header()
            point_msg.point = Point(x=0.0, y=-1.0, z=90.0)
            pub.publish(point_msg.point)

        except rospy.ROSInterruptException as error:
            print(error)

if __name__=="__main__":
    rospy.init_node("virtual_point_publisher", anonymous=True)
    print("virtual pose to check things published to the topic /odometry_pose ...")
    print("x=0.0, y=-1.0, z=90.0")
    publish_virtual_point()
