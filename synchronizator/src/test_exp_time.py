#!/usr/bin/env python

import datetime
import math

import rospy
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from dvs_msgs.msg import SpecialEvent, Event, EventArray
from bluefox_ros.msg import CaptureInfo
from std_srvs.srv import Trigger, TriggerResponse
import message_filters



# TODO: prevent changes to exposure time while waiting for previous change
# TODO: timeout wait for exp time change
# TODO: detect significant changes in timing offset (for example due to dropped frame)




class ExpTimeTest:

    TOLERANCE_US = 50 # difference between reported/configured exposure time and that measured by DAVIS is quite significant

    LOW_EXP_TIME  = 8000
    HIGH_EXP_TIME = 9000

    def __init__(self):
        rospy.init_node('event_visualizer', anonymous=True)

        self.configclient = dynamic_reconfigure.client.Client('/camera')

        rospy.Subscriber("/dvs/special_events", SpecialEvent, self.special_event_callback)

        #rospy.Subscriber("/camera/capture_info", CaptureInfo, self.image_callback)

        capture_info = message_filters.Subscriber("/camera/capture_info", CaptureInfo)
        camera_image = message_filters.Subscriber("/camera/image_raw", Image)

        self.synced_cam_info = message_filters.TimeSynchronizer([capture_info, camera_image], 10)
        self.synced_cam_info.registerCallback(self.image_callback)


        self.capt_info_pub = rospy.Publisher("/synchronized/camera/capture_info", CaptureInfo, queue_size=2)
        self.image_pub     = rospy.Publisher("/synchronized/camera/image_raw", Image, queue_size=2)



        rospy.Service('change_exp_time', Trigger, self.change_trigger)

        self.last_falling = rospy.Time.now()

        self.currently_set_exp_time = 0

        # timestamp of first image received that matches new exposure time
        self.first_good_syncframe   = None
        self.first_good_cameraframe = None

        self.first_good_sync_seq    = None
        self.first_good_image_seq   = None
        self.image_event_seq_offset = None

        self.sync_timestamps = {}
        self.image_queue = []

    def matches_expected_exposure_time(self, exp_time_us):
        return self.currently_set_exp_time - ExpTimeTest.TOLERANCE_US <= exp_time_us <= self.currently_set_exp_time + ExpTimeTest.TOLERANCE_US

    def special_event_callback(self, event):
        if event.polarity == False:
            self.last_falling = event.ts
            return

        #print "got sync frame (exp_time:", (event.ts - self.last_falling).to_sec()*1000*1000, ", ts:", event.ts.to_sec(), ")"
        if self.first_good_syncframe is None and self.matches_expected_exposure_time((event.ts - self.last_falling).nsecs/1000):
            #print("MATCH!")

            # todo: which point in time to take? middle of exposure?
            #self.first_good_syncframe = self.last_falling
            self.first_good_syncframe = event.ts
            self.first_good_sync_seq  = event.header.seq
            self.check_first_good()

        if self.image_event_seq_offset is not None:
            # middle of exposure
            correct_ts = self.last_falling + (event.ts-self.last_falling)/2
            self.sync_timestamps[int(math.floor(event.header.seq/2))] = correct_ts

            self.check_pending_buffers()

    def image_callback(self, info, img):

        #print "got camera frame (exp_time:", img.exp_time_us, ", ts:", img.header.stamp.to_sec(), ")"
        if self.first_good_cameraframe is None and self.matches_expected_exposure_time(info.exp_time_us):
            #print("MATCH!")
            self.first_good_cameraframe = info.header.stamp
            self.first_good_image_seq   = img.header.seq
            self.check_first_good()

        if self.image_event_seq_offset is not None:
            self.image_queue.append((info, img))
            self.check_pending_buffers()


    def check_first_good(self):
        if self.first_good_syncframe and self.first_good_cameraframe:
            # now keep track of this match-up...
            self.image_event_seq_offset = int(math.floor(self.first_good_sync_seq/2)) - self.first_good_image_seq

            print "found matching pair: time difference:", (self.first_good_cameraframe - self.first_good_syncframe).to_sec()*1000, "ms"
            print "seq offset:", self.image_event_seq_offset


    def check_pending_buffers(self):
        if len(self.image_queue) == 0 or len(self.sync_timestamps) == 0:
            return

        i = 0
        while i < len(self.image_queue):
            info, img = self.image_queue[i]
            expected_seq = img.header.seq + self.image_event_seq_offset

            if expected_seq in self.sync_timestamps:

                print "published synced image pair. offset:", (info.header.stamp-self.sync_timestamps[expected_seq]).to_sec()*1000, "ms"

                info.header.stamp = self.sync_timestamps[expected_seq]
                img .header.stamp = self.sync_timestamps[expected_seq]

                self.image_pub.publish(img)
                self.capt_info_pub.publish(info)

                del self.image_queue[i]
                del self.sync_timestamps[expected_seq]
            else:
                i += 1

        if len(self.image_queue) > 2 and len(self.sync_timestamps) > 2:
            print "WARNING: Matchup between images and sync events failed!"
            for info, img in self.image_queue:
                print "> IMAGE:", img.header.seq, img.header.seq+self.image_event_seq_offset

            for seq, ts in self.sync_timestamps.iteritems():
                print "> SYNC:", seq


    def change_trigger(self, reg):

        # toggle between two different exposure times
        if self.currently_set_exp_time == ExpTimeTest.LOW_EXP_TIME:
            t = ExpTimeTest.HIGH_EXP_TIME
        else:
            t = ExpTimeTest.LOW_EXP_TIME

        self.config = self.configclient.update_configuration({'exp_time': t})
        print "changed exposure time to", self.config['exp_time']/1000, "ms"
        self.currently_set_exp_time = self.config['exp_time']

        self.first_good_syncframe   = None
        self.first_good_cameraframe = None

        self.first_good_sync_seq    = None
        self.first_good_image_seq   = None
        self.image_event_seq_offset = None

        self.sync_timestamps = {}
        self.image_queue = []

        response = TriggerResponse()
        response.success = True
        response.message = ""
        return response



#rospy.Subscriber("/camera/image_raw", Image, tl_camera_images.header_callback)


if __name__ == "__main__":
    e = ExpTimeTest()
    rospy.spin()
