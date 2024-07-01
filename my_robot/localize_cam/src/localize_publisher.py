#!/usr/bin/python3
import rospy
#import pyrealsense2 as rs
import cv2
import numpy as np
# from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def cv2_to_imgmsg(cvim, encoding="passthrough", header=None):
    if not isinstance(cvim, (np.ndarray, np.generic)):
        raise TypeError('Your input type is not a numpy array')
    # prepare msg
    img_msg = Image()
    img_msg.height = cvim.shape[0]
    img_msg.width = cvim.shape[1]
    if header is not None:
        img_msg.header = header
    # encoding handling
    numpy_type_to_cvtype = {'uint8': '8U', 'int8': '8S', 'uint16': '16U',
                            'int16': '16S', 'int32': '32S', 'float32': '32F',
                            'float64': '64F'}
    numpy_type_to_cvtype.update(dict((v, k) for (k, v) in numpy_type_to_cvtype.items()))
    if len(cvim.shape) < 3:
        cv_type = '{}C{}'.format(numpy_type_to_cvtype[cvim.dtype.name], 1)
    else:
        cv_type = '{}C{}'.format(numpy_type_to_cvtype[cvim.dtype.name], cvim.shape[2])
    if encoding == "passthrough":
        img_msg.encoding = cv_type
    else:
        img_msg.encoding = encoding
    if cvim.dtype.byteorder == '>':
        img_msg.is_bigendian = True
    # img data to msg data
    img_msg.data = cvim.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height

    return img_msg


def imgmsg_to_cv2(img_msg, dtype=np.uint8):
    # it should be possible to determine dtype from img_msg.encoding but there is many different cases to take into account
    # original function args: imgmsg_to_cv2(img_msg, desired_encoding = "passthrough")
    return np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width, -1)

class ImagePublisher():
    def __init__(self) -> None:
        self.image_pub = rospy.Publisher("localize_img", Image, queue_size=10)
        # self.bridge = CvBridge()

        '''logitech'''
        self.capture = cv2.VideoCapture('/dev/video0')
        
        imgsz = (960, 544)

        if type(imgsz) is tuple:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, imgsz[0])
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, imgsz[1])

        # '''realsense'''
        # self.pipe = rs.pipeline()
        # self.cfg = rs.config()

        # self.cfg.enable_stream(rs.stream.infrared)
        # self.cfg.enable_stream(rs.stream.color, rs.format.bgr8, 30)
        # self.cfg.enable_stream(rs.stream.depth, rs.format.z16, 30)

        # self.pipe.start(self.cfg)

    def logitech_publish_callback(self, event):
        ret, img = self.capture.read()
        if not ret:
            rospy.ERROR("Could not grab a frame!")
            return
        try:
            self.img_msg = cv2_to_imgmsg(img)
            self.image_pub.publish(self.img_msg)
        except CvBridgeError as error:
            print(error)

def publish_image():
    ip = ImagePublisher()
    
    rate = 30.0    # Hz
    
    rospy.Timer(rospy.Duration(1.0/rate), ip.logitech_publish_callback)
    # rospy.Timer(rospy.Duration(1.0/100.0), ip.realsense_callback)
    rospy.spin()
        

if __name__=="__main__":
    rospy.init_node("localize_cam_publisher", anonymous=True)
    print("Image is being published to the topic /localize_img ...")
    publish_image()
