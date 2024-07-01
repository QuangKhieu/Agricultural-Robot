#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped, Point, Vector3
from std_msgs.msg import Header

prev_odometry = np.array([0, 0, 0])
curr_odometry = np.array([0, 0, 0])
rate = 10   # Hz
pose_pub = rospy.Publisher("odometry_velocity", TwistStamped, queue_size=10)

def publish_odo_velo(event):
    global prev_odometry, curr_odometry, rate
    displacement = curr_odometry - prev_odometry
    velocity = displacement * rate
    v = np.linalg.norm(velocity[0:2])
    w = velocity[2]

    odo_velo = TwistStamped()
    odo_velo.header = Header()
    odo_velo.twist.linear = Vector3(curr_odometry[0], curr_odometry[1], curr_odometry[2])
    odo_velo.twist.angular = Vector3(v, w, 1/rate)
    
    pose_pub.publish(odo_velo)
    prev_odometry = curr_odometry

def sub_callback(odometry_msg:Point):
    global curr_odometry
    curr_odometry = np.array([odometry_msg.x, odometry_msg.y, odometry_msg.z])

if __name__=="__main__":
    rospy.init_node("odometry_pose_velo", anonymous=True)
    print("Subscribe to \odometry_pose and publish to ekf...")
    
    pose_sub = rospy.Subscriber("odometry_pose", Point, sub_callback)  # Subscribe outside the loop
    rospy.Timer(rospy.Duration(1/rate), publish_odo_velo)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown!")
