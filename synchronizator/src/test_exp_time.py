#!/usr/bin/env python

import datetime

import rospy
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from dvs_msgs.msg import SpecialEvent, Event, EventArray
from bluefox_ros.msg import CaptureInfo
from std_srvs.srv import Trigger, TriggerResponse







class ExpTimeTest:

    TOLERANCE_US = 50 # difference between reported/configured exposure time and that measured by DAVIS is quite significant

    LOW_EXP_TIME  = 8000
    HIGH_EXP_TIME = 9000

    def __init__(self):
        rospy.init_node('event_visualizer', anonymous=True)

        self.configclient = dynamic_reconfigure.client.Client('/camera')

        rospy.Subscriber("/dvs/special_events", SpecialEvent, self.special_event_callback)
        rospy.Subscriber("/camera/capture_info", CaptureInfo, self.image_callback)

        rospy.Service('change_exp_time', Trigger, self.change_trigger)

        self.last_falling = rospy.Time.now()

        self.currently_set_exp_time = 0

        # timestamp of first image received that matches new exposure time
        self.first_good_syncframe = None
        self.first_good_cameraframe = None

    def matches_expected_exposure_time(self, exp_time_us):
        return self.currently_set_exp_time - ExpTimeTest.TOLERANCE_US <= exp_time_us <= self.currently_set_exp_time + ExpTimeTest.TOLERANCE_US

    def special_event_callback(self, event):
        if event.polarity == False:
            self.last_falling = event.ts
        else:
            #print "got sync frame (exp_time:", (event.ts - self.last_falling).to_sec()*1000*1000, ", ts:", event.ts.to_sec(), ")"
            if self.first_good_syncframe is None and self.matches_expected_exposure_time((event.ts - self.last_falling).nsecs/1000):
                #print("MATCH!")

                # todo: which point in time to take? middle of exposure?
                #self.first_good_syncframe = self.last_falling
                self.first_good_syncframe = event.ts
                self.check_first_good()

    def image_callback(self, img):
        #print "got camera frame (exp_time:", img.exp_time_us, ", ts:", img.header.stamp.to_sec(), ")"
        if self.first_good_cameraframe is None and self.matches_expected_exposure_time(img.exp_time_us):
            #print("MATCH!")
            self.first_good_cameraframe = img.header.stamp
            self.check_first_good()


    def check_first_good(self):
        if self.first_good_syncframe and self.first_good_cameraframe:
            print "found matching pair: time difference:", (self.first_good_cameraframe - self.first_good_syncframe).to_sec()*1000, "ms"

            # now keep track of this match-up...


    def change_trigger(self, reg):

        # toggle between two different exposure times
        if self.currently_set_exp_time == ExpTimeTest.LOW_EXP_TIME:
            t = ExpTimeTest.HIGH_EXP_TIME
        else:
            t = ExpTimeTest.LOW_EXP_TIME

        self.config = self.configclient.update_configuration({'exp_time': t})
        print "changed exposure time to", self.config['exp_time']/1000, "ms"
        self.currently_set_exp_time = self.config['exp_time']

        self.first_good_syncframe = None
        self.first_good_cameraframe = None

        response = TriggerResponse()
        response.success = True
        response.message = ""
        return response



#rospy.Subscriber("/camera/image_raw", Image, tl_camera_images.header_callback)


if __name__ == "__main__":
    e = ExpTimeTest()
    rospy.spin()
