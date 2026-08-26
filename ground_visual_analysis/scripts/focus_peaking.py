#!/usr/bin/env python
import rospy
import rospkg
import cv2
import sys

from sensor_msgs.msg import Image
from visualanalysis_msgs.msg import TargetPositionROI2DArray
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError


class FocusPeaking():
    def __init__(self):
        rospack = rospkg.RosPack()
        self.bridge = CvBridge()
        self.img = None
        self.low_w = 0
        self.low_h = 0
        self.high_w = None
        self.high_h = None
        self.first = 1
        self.roi_time = None
        self.drone_id = rospy.get_param("~drone_id", 1)
        self.topic = "/drone_{}/shooting_camera".format(self.drone_id)
        self.img_sub_topic = rospy.get_param("~camera_topic", self.topic)
        self.image_sub = rospy.Subscriber(self.img_sub_topic, Image, self.image_callback,
                                          queue_size=1,
                                          buff_size=2 ** 24)
        self.tracking_ROI_sub = rospy.Subscriber(
            "/drone_{}/visual_analysis/target_position_roi_2d_array/".format(self.drone_id),
            TargetPositionROI2DArray, self.tracking_roi_callback, queue_size=1)
        self.image_pub = rospy.Publisher("/drone_{}/visual_analysis/focus_peak_image/".format(self.drone_id),
                                         Image, queue_size=1)
        self.focus_pub_image = rospy.Publisher("/drone_{}/visual_analysis/focus_value_image/".format(self.drone_id),
                                               Int32, queue_size=1)
        self.focus_pub_target = rospy.Publisher("/drone_{}/visual_analysis/focus_value_target/".format(self.drone_id),
                                                Int32, queue_size=1)
        rospy.loginfo(self.img_sub_topic)

    def tracking_roi_callback(self, data):
        self.check = True
        self.roi_time = rospy.Time.now()
        target = data.targets[0]
        self.low_w = target.x
        self.low_h = target.y
        self.high_w = self.low_w + target.w
        self.high_h = self.low_h + target.h

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img = cv_image
            self.img_g = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
            if self.roi_time is not None:
                if rospy.Time.now() - self.roi_time > rospy.Duration(2):
                    rospy.loginfo('WARNING! Last ROI is from over 2 seconds ago')
                    self.init_bounds()

            rospy.loginfo("img")
        except CvBridgeError as e:
            print e
            self.img = None
            return
        self.createPeaks()
        return

    def init_bounds(self):
        self.high_h, self.high_w = self.img.shape[:2]
        self.low_w = int(0.2 * self.high_w)
        self.low_h = int(0.2 * self.high_h)
        self.high_h = int(0.8 * self.high_h)
        self.high_w = int(0.8 * self.high_w)
        self.check = False

    def createPeaks(self):
        s = 0
        s2 = 0
        l1 = int(self.img.shape[0] * 0.2)
        l2 = int(self.img.shape[1] * 0.2)
        h1 = int(self.img.shape[0] * 0.8)
        h2 = int(self.img.shape[1] * 0.8)
        edges = cv2.Canny(self.img_g[l1:h1, l2:h2], 280, 350, 3)
        edges2 = cv2.Canny(self.img_g[self.low_h:self.high_h, self.low_w:self.high_w], 280, 350, 3)
        tmp_img = self.img[l1:h1, l2:h2, :]
        if edges is not None:
            s = sum(sum(edges2))
            s2 = sum(sum(edges))
            tmp_img[edges > 0, 1] = 255
        self.focus_pub_target.publish(s)
        self.focus_pub_image.publish(s2)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(tmp_img, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    rospy.init_node('focusPeaking', anonymous=True, log_level=rospy.DEBUG)
    tr = FocusPeaking()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
