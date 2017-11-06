#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

from dvs_msgs.msg import SpecialEvent, Event, EventArray
from bluefox_ros.msg import CaptureInfo

import message_filters



class TestForwarder:
    def __init__(self):

        rospy.init_node('forwarder', anonymous=True)
        rospy.loginfo("starting test forwarder")

        images_sub = message_filters.Subscriber('/camera/image_raw', Image)
        images_sub.registerCallback(self.callback)

        self.image_pub = rospy.Publisher("/foobar", Image, queue_size=2)

    def run(self):
        rospy.spin()

    def callback(self, image):
        image.header.stamp = image.header.stamp + rospy.Duration(secs=2)
        self.image_pub.publish(image)


class Synchronizator:
    def __init__(self):
        rospy.init_node('synchronizator', anonymous=True)
        rospy.loginfo("starting synchronizator")

        self.last_stamp = rospy.Time.now()

        event_sub  = message_filters.Subscriber('/dvs/special_events', SpecialEvent)
        info_sub   = message_filters.Subscriber('/camera/capture_info', CaptureInfo)
        images_sub = message_filters.Subscriber('/camera/image_raw', Image)

        ts = message_filters.ApproximateTimeSynchronizer([event_sub, info_sub, images_sub], 10, 0.1)
        ts.registerCallback(self.callback)

        self.image_pub = rospy.Publisher("/synchronized/camera/image_raw", Image, queue_size=10)

    def run(self):
        rospy.spin()

    def callback(self, event, camera_info, image):
        #rospy.loginfo("got camera frame with matching special event: event: {} image: {}".format(event.header.stamp, camera_info.header.stamp))
        deltaT = event.header.stamp - camera_info.header.stamp
        fps    = rospy.Duration(1)/(event.header.stamp-self.last_stamp)
        #rospy.loginfo("fps: {}, deltaT: {}s".format(fps, deltaT.to_sec()))
        self.last_stamp = event.header.stamp

        # set new timestamp for images based on special events
        # timestamps of images refer to middle of exposure
        # falling edges represent start of exposure (polarity = false)
        image.header.stamp = event.header.stamp + rospy.Duration(nsecs=1000*camera_info.exp_time_us)/2
        self.image_pub.publish(image)



if __name__ == '__main__':
    s = Synchronizator()
    #s = TestForwarder()
    s.run()

