#!/usr/bin/env python

import message_filters
import rospy
import rospkg
from sensor_msgs.msg import Image
from visualanalysis_msgs.msg import TargetPositionROI2DArray, VisualControlError, TargetPositionROI2D
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge

import cv2


class Visualizer(object):
    def __init__(self):
        rospack = rospkg.RosPack()
        self.drone_id = rospy.get_param('~drone_id', 1)
        self.path = rospack.get_path('drone_visual_analysis')
        self.pub = rospy.Publisher('/drone_{}/visualization'.format(self.drone_id),
                                   Image, queue_size=10)
        self.bridge = CvBridge()

        # subscriptions
        # roi
        roi_topic = '/drone_{}/visual_analysis/target_position_roi_2d_array'.format(self.drone_id)
        # rospy.Subscriber(roi_topic, TargetPositionROI2DArray, self.roi_callback)
        roi_sub = message_filters.Subscriber(roi_topic, TargetPositionROI2DArray)
        self.roi_time = None
        self.roi = None

        # detections
        det_topic = '/drone_{}/detection_rois'.format(self.drone_id)
        rospy.Subscriber(det_topic, TargetPositionROI2DArray, self.det_callback)
        self.det_time = None
        self.dets = None

        # pixel position
        gps_topic = '/drone_{}/pixel_position'.format(self.drone_id)
        rospy.Subscriber(gps_topic, Vector3, self.gps_callback)
        self.gps_time = None
        self.gps_msg = None
        self.gps = None

        # visual control error
        vce_topic = '/drone_{}/visual_analysis/visual_control_error'.format(self.drone_id)
        rospy.Subscriber(vce_topic, VisualControlError, self.vce_callback)
        vce_target_topic = '/drone_{}/visual_analysis/visual_control_target'.format(self.drone_id)
        rospy.Subscriber(vce_target_topic, Vector3, self.vce_target_callback)
        vce_goal_topic = '/drone_{}/visual_analysis/visual_control_goal'.format(self.drone_id)
        rospy.Subscriber(vce_goal_topic, Vector3, self.vce_goal_callback)
        self.vce = None

        # verification step
        ver_topic = '/drone_{}/verification_rois'.format(self.drone_id)
        rospy.Subscriber(ver_topic, TargetPositionROI2DArray, self.ver_callback)
        area_topic = '/drone_{}/detection_area'.format(self.drone_id)
        rospy.Subscriber(area_topic, TargetPositionROI2D, self.area_callback)
        self.area_time = None
        self.area = None
        self.ver_time = None
        self.vers = None

        # image
        self.image_topic = rospy.get_param('~camera_topic'.format(self.drone_id),
                                           '/drone_{}/shooting_camera'.format(self.drone_id))
        sa_image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback,
                                        queue_size=1,
                                        buff_size=2 ** 24)
        image_sub = message_filters.Subscriber(self.image_topic, Image)
        self.img = None
        self.img_msg = None

        self.timer = rospy.Timer(rospy.Duration(1 / 25.), self.timed_callback)
        self.info_timer = rospy.Timer(rospy.Duration(1), self.info_timer_callback)

        self.ts = message_filters.TimeSynchronizer([image_sub, roi_sub], 10)
        self.ts.registerCallback(self.main_callback)
        self.has_img_and_roi = False
        self.time_limit = 1
        self.img_roi_time = rospy.Time.now()

    def info_timer_callback(self, timer):
        if self.gps_msg is not None:
            rospy.loginfo('GPS: {},{},{}'.format(self.gps_msg.x, self.gps_msg.y, self.gps_msg.z))

    def main_callback(self, img_msg, roi_msg):
        rospy.loginfo('Image and ROI [{}]'.format(img_msg.header.stamp))
        # synchronized image and roi
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        self.img = cv_image

        self.roi_time = rospy.Time.now()
        self.roi = [roi_msg.targets[0].x, roi_msg.targets[0].y,
                    roi_msg.targets[0].x + roi_msg.targets[0].w,
                    roi_msg.targets[0].y + roi_msg.targets[0].h,
                    roi_msg.targets[0].trk_score]
        self.has_img_and_roi = True
        self.img_roi_time = rospy.Time.now()

    def timed_callback(self, timer):
        if rospy.Time.now() - self.img_roi_time > rospy.Duration(self.time_limit):
            self.has_img_and_roi = False
        if self.img is not None:
            if self.roi is not None:
                cv2.rectangle(self.img, (self.roi[0], self.roi[1]), (self.roi[2], self.roi[3]), (255, 144, 30), 3)
                cv2.putText(self.img, '{},{} {:1.5f}'.format(self.roi[0], self.roi[1], self.roi[4]),
                            (self.roi[0], self.roi[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 255), 2)
                if self.vce is not None:
                    roi_w = self.roi[2] - self.roi[0]
                    roi_h = self.roi[3] - self.roi[1]

                    cv2.circle(self.img, self.vce_target, 10, (255, 0, 0), -1)

                    cv2.circle(self.img, self.vce_goal, 10, (0, 255, 0), -1)

                    cv2.line(self.img, (self.img.shape[1] / 2, self.img.shape[0] / 2),
                             (self.roi[0] + roi_w / 2, self.roi[1] + roi_h / 2), (0, 0, 0), 1)
                    cv2.putText(self.img, '{:.1f},{:.1f}'.format(self.vce[0], self.vce[1]),
                                (self.img.shape[1] / 2, self.img.shape[0] / 2 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (60, 20, 220), 2)
                if rospy.Time.now() - self.roi_time > rospy.Duration(2):
                    rospy.loginfo('WARNING! Last ROI is from over 2 seconds ago')
                    self.roi = None
            if self.area is not None:
                cv2.rectangle(self.img, (self.area[0], self.area[1]), (self.area[2], self.area[3]), (0, 255, 0), 10)
                if rospy.Time.now() - self.area_time > rospy.Duration(1):
                    self.area = None
            if self.gps is not None:
                cv2.circle(self.img, self.gps, 10, (0, 0, 255), -1)
                if rospy.Time.now() - self.gps_time > rospy.Duration(2):
                    rospy.loginfo('WARNING! Last GPS signal is from over 2 seconds ago')
                    self.gps = None
                    self.gps_msg = None
            if self.dets is not None:
                for det in self.dets:
                    cv2.rectangle(self.img, (det[0], det[1]), (det[2], det[3]), (50, 205, 50), 3)
                    cv2.putText(self.img, '{:1.5f}'.format(det[4]), (det[0], det[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (255, 255, 255), 2)
                    if rospy.Time.now() - self.det_time > rospy.Duration(0.2):
                        self.dets = None
            if self.vers is not None:
                for ver in self.vers:
                    cv2.rectangle(self.img, (ver[0], ver[1]), (ver[2], ver[3]), (238, 104, 123), 3)
                    if rospy.Time.now() - self.ver_time > rospy.Duration(0.5):
                        self.vers = None

            self.img_msg = self.bridge.cv2_to_imgmsg(self.img, encoding='bgr8')
            self.pub.publish(self.img_msg)

    def gps_callback(self, msg):
        self.gps_time = rospy.Time.now()
        self.gps_msg = msg
        self.gps = (int(msg.x), int(msg.y))

    def vce_callback(self, msg):
        self.vce = [msg.x_error, msg.y_error]

    def vce_target_callback(self, msg):
        self.vce_target = (int(msg.x), int(msg.y))

    def vce_goal_callback(self, msg):
        self.vce_goal = (int(msg.x), int(msg.y))

    def roi_callback(self, msg):
        self.roi_time = rospy.Time.now()
        self.roi = [msg.targets[0].x, msg.targets[0].y,
                    msg.targets[0].x + msg.targets[0].w,
                    msg.targets[0].y + msg.targets[0].h,
                    msg.targets[0].trk_score]

    def area_callback(self, msg):
        self.area_time = rospy.Time.now()
        self.area = [msg.x, msg.y,
                     msg.x + msg.w,
                     msg.y + msg.h]

    def det_callback(self, msg):
        self.det_time = rospy.Time.now()
        self.dets = [[roi.x, roi.y, roi.x + roi.w, roi.y + roi.h, roi.det_score] for roi in msg.targets]

    def ver_callback(self, msg):
        self.ver_time = rospy.Time.now()
        self.vers = [[roi.x, roi.y, roi.x + roi.w, roi.y + roi.h] for roi in msg.targets]

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        if not self.has_img_and_roi:
            self.img = cv_image


if __name__ == '__main__':
    rospy.init_node('Visualizer', anonymous=True)
    node = Visualizer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
