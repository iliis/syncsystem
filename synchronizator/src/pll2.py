#!/usr/bin/env python

import datetime
import math
import threading
import numpy as np

import rospy
from dvs_msgs.msg import SpecialEvent

################################################################################
# Helper Functions

def prevnext(l):
    it = iter(l)
    try:
        last_obj = it.next()
        while True:
            next_obj = it.next()
            yield last_obj, next_obj
            last_obj = next_obj
    except StopIteration:
        pass

################################################################################

class SpecialEventsPLL:

    # PARAMETERS
    #############################################

    TIMEOUT = rospy.Duration.from_sec(2) # timeout for waiting for data/synchronized frames etc.

    MISSING_EVENT_TIMEOUT = rospy.Duration.from_sec(0.02) # wait at least this long before publishing estimated events

    MIN_QUEUE_LENGTH =  20
    MAX_QUEUE_LENGTH = 100

    #---------------------------------------------------------------------------

    def __init__(self):

        self.mutex = threading.Lock()
        self.got_new_data = threading.Event()

        rospy.init_node('special_event_filter', anonymous=True)

        rospy.Subscriber("/dvs/special_events", SpecialEvent, self.special_event_callback, queue_size=None)

        self.event_publisher = rospy.Publisher("/synchronized/special_events", SpecialEvent, queue_size=0)

        self.invert_special_events = rospy.get_param('~invert_special_events', False)

        self.timeout_timer = None

        self.special_events = []

        self.published_event_seq = 0
        self.last_published_frame_t = None

    #---------------------------------------------------------------------------

    def special_event_callback(self, event):
        with self.mutex:

            if self.invert_special_events:
                event.polarity = not event.polarity

            self.special_events.append(event)

            self.got_new_data.set()


    #---------------------------------------------------------------------------

    def main(self):
        while not rospy.is_shutdown():
            self.got_new_data.wait(timeout=self.TIMEOUT.to_sec()) # todo: lower this, to quickly react to lost events
            if not self.got_new_data.is_set():
                rospy.logwarn("Timeout in main thread while waiting for data.")
                continue

            with self.mutex:
                self.process()
                self.got_new_data.clear()

    # just to keep indentation to a minimum ;)
    def process(self):

        # wait for PLL to settle
        if len(self.special_events) < self.MIN_QUEUE_LENGTH:
            return

        # remove old events
        if len(self.special_events) > self.MAX_QUEUE_LENGTH:
            del self.special_events[:len(self.special_events)-self.MAX_QUEUE_LENGTH]

        assert len(self.special_events) <= self.MAX_QUEUE_LENGTH

        # estimate parameters
        ref_t = rospy.Time.now()



        # only use complete frames for estimation (TODO: also incorporate single events?)
        event_pairs = [(A,B) for A, B in prevnext(self.special_events) if A.polarity and not B.polarity]
        frame_ts    = [A.ts + (B.ts-A.ts)/2 for A, B in event_pairs]
        exp_times   = [B.ts - A.ts for A, B in event_pairs]

        inter_frame_ts = [B-A for A, B in prevnext(frame_ts)]

        inter_frame_time_est = np.median(inter_frame_ts)
        exp_time_est         = np.median(exp_times)

        # initialize last published frame so that we start publishing first received event
        if self.last_published_frame_t is None:
            self.last_published_frame_t = frame_ts[0] - inter_frame_time_est

        # TODO: measure stddev and abort/error/reset when it is too high (e.g. due to changed framerate)

        #phase_offsets = [ref_t - (frame_t+(len(frame_ts)-i)*inter_frame_time_est) for i, frame_t in enumerate(frame_ts)]
        #print "FPS:", inter_frame_time_est.to_sec(), "exp_time:", exp_time_est, "phase offset:", np.median(phase_offsets).to_sec()

        # advance to next frame
        while ref_t - self.last_published_frame_t >= inter_frame_time_est + self.MISSING_EVENT_TIMEOUT:
            # it's time for the next frame!

            # estimate time of new frame
            new_frame_t = self.last_published_frame_t + inter_frame_time_est

            # look for a real frame that matches estimate
            found = False
            for frame_t, exp_time in reversed(zip(frame_ts, exp_times)):
                if abs(frame_t - new_frame_t) <= inter_frame_time_est/2:
                    #rospy.loginfo("publishing event pair. phase offset: {:7.3f}ms, exp_time offset: {:7.3f}ms".format(
                    #    (frame_t - new_frame_t).to_sec()*1000, (exp_time - exp_time_est).to_sec()*1000))
                    self.publish_event_pair(frame_t, exp_time)
                    found = True
                    return # give special event thread some time to catch up
                    #break

                # optimize search by aborting early
                if frame_t <= self.last_published_frame_t:
                    break

            if found:
                continue

            # nothing found
            rospy.logwarn("missing events detected, publishing estimated event pair t= {} / {} seq = {}"
                    .format(new_frame_t-exp_time_est/2, new_frame_t+exp_time_est/2, self.published_event_seq))
            self.publish_event_pair(new_frame_t, exp_time_est)


    def publish_event_pair(self, frame_t, exp_time):
        #rospy.loginfo(" ---> published frame_t = {}, exp_time = {:7.3f}ms".format(frame_t.to_sec()*1000 % 10000, exp_time.to_sec()*1000))

        # check that we don't publish out of order or otherwise impossible data
        if not self.last_published_frame_t < frame_t - exp_time/2:
            rospy.logerr("invalid call to publish_event_pair(): last published a frame at t={}, new at t={}, seq={}".format(
                self.last_published_frame_t, frame_t, self.published_event_seq))

        self.publish_event(frame_t - exp_time/2, not self.invert_special_events)
        self.publish_event(frame_t + exp_time/2,     self.invert_special_events)
        self.last_published_frame_t = frame_t


    def publish_event(self, ts, polarity):
        event = SpecialEvent()
        event.polarity = polarity
        event.ts = ts
        event.header.seq = self.published_event_seq
        event.header.stamp = event.ts

        self.event_publisher.publish(event)

        self.published_event_seq += 1





################################################################################

if __name__ == "__main__":
    pll = SpecialEventsPLL()
    pll.main()
