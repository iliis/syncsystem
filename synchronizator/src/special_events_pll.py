#!/usr/bin/env python

import datetime
import math
import threading
import numpy as np

import rospy
from dvs_msgs.msg import SpecialEvent



class SpecialEventsPLL:

    # PARAMETERS
    #############################################

    TIMEOUT = rospy.Duration.from_sec(2) # timeout for waiting for data/synchronized frames etc.

    GAIN = 0.1

    #---------------------------------------------------------------------------

    def __init__(self):

        self.mutex = threading.Lock()
        self.got_new_data = threading.Event()

        rospy.init_node('event_visualizer', anonymous=True)

        rospy.Subscriber("/dvs/special_events", SpecialEvent, self.special_event_callback, queue_size=None)

        self.event_publisher = rospy.Publisher("/synchronized/special_events", SpecialEvent, queue_size=0)

        self.invert_special_events = rospy.get_param('~invert_special_events', False)

        self.timeout_timer = None

        self.last_rising_edge = None
        self.last_measured_frame_t = None

        self.last_published_frame_t = None

        self.est_exposure_time = None # how long a single frame is
        self.est_frame_period  = None # time between two frames (middle to middle)

        self.published_event_seq = 0

        self.has_settled = False

        # if PLL has locked output actual events if there are any that are
        # close enough to the ones predicted by the PLL
        self.measured_frames = [] # type: List[Tuple[rospy.Time, rospy.Duration]]

    #---------------------------------------------------------------------------

    def special_event_callback(self, event):
        with self.mutex:

            if self.invert_special_events:
                event.polarity = not event.polarity

            if event.polarity == True: # begin of a frame
                if self.last_rising_edge is not None:
                    # error, two rising edges after each other
                    rospy.logwarn("invalid rising edge")
                    pass # can't do anything here

                self.last_rising_edge = event

            else:

                self.has_settled = False

                if self.last_rising_edge is None:
                    # error, this can't happen in proper operation (but sometimes does, eg. if ROS is dropping messages)
                    rospy.logwarn("ignoring invalid falling edge")
                    return # ignore this event

                current_exp_time = event.ts - self.last_rising_edge.ts
                current_frame_t  = event.ts - current_exp_time/2

                self.last_rising_edge = None

                if current_exp_time < rospy.Duration.from_sec(0.0005):
                    rospy.logwarn("ignoring very short frame with {} us exposure time.".format(current_exp_time.nsecs/1000))
                    return

                # initialize PLL
                if self.est_exposure_time is None:
                    self.est_exposure_time     = current_exp_time
                    self.last_measured_frame_t = current_frame_t
                    return

                # initialization at second frame
                if self.est_frame_period is None:
                    self.est_frame_period = current_frame_t - self.last_measured_frame_t
                    self.last_published_frame_t = self.last_measured_frame_t
                    return

                current_frame_period = current_frame_t - self.last_measured_frame_t

                # actual PLL
                exp_time_err     = current_exp_time - self.est_exposure_time
                frame_period_err = (current_frame_t - self.last_measured_frame_t) - self.est_frame_period
                phase_err        = current_frame_t - self.last_published_frame_t

                # phase error modulo +/-period/2
                while abs(phase_err) >= self.est_frame_period / 2:
                    #rospy.loginfo("{} {}".format(phase_err, self.est_frame_period))
                    if phase_err > rospy.Duration(0):
                        phase_err -= self.est_frame_period
                    else:
                        phase_err += self.est_frame_period

                self.est_exposure_time += exp_time_err * self.GAIN
                self.est_frame_period  += frame_period_err * self.GAIN
                self.last_published_frame_t += phase_err * self.GAIN

                errstr = "Estimation errors: exposure time: {:6.3f} ms, frame period: {:6.3f} ms, phase: {:6.3f} ms" \
                        .format(exp_time_err.to_sec()*1000, frame_period_err.to_sec()*1000, phase_err.to_sec()*1000)

                if abs(phase_err) < rospy.Duration.from_sec(0.0001):
                    self.has_settled = True
                    rospy.loginfo(errstr)

                    self.measured_frames.append( (current_frame_t, current_exp_time) )
                else:
                    self.has_settled = False
                    rospy.logwarn(errstr)


                self.last_measured_frame_t = current_frame_t
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

    # just to keep indentation to a minimum ;)
    def process(self):

        # wait for PLL to settle
        if self.last_published_frame_t is None:
            return

        # don't publish events into the future
        if self.last_published_frame_t + self.est_frame_period > rospy.Time.now():
            return

        # but don't publish anything while PLL hasn't locked in yet
        if not self.has_settled:
            return

        # look for a close match of actual data
        # otherwise take predicted data
        current_frame_t  = self.last_published_frame_t
        current_exp_time = self.est_exposure_time

        for i in reversed(range(len(self.measured_frames))):

            frame_t  = self.measured_frames[i][0]
            exp_time = self.measured_frames[i][1]
            if abs(frame_t - current_frame_t) < rospy.Duration.from_sec(0.001):
                rospy.loginfo("found match, publishing real data. Frame offset: {} us".format((frame_t - current_frame_t).to_sec()*1000000))
                current_frame_t  = frame_t
                current_exp_time = exp_time
                del self.measured_frames[:i+1] # remove everything up to and including this frame
                # advance to next frame
                self.last_published_frame_t += self.est_frame_period
                break
            else:
                rospy.logwarn("measured data off by too much, publishing estimated data. Error: {:6.3f}ms".format((frame_t-current_frame_t).to_sec()*1000))
                # delete every frame in the past
                # TODO: ???
                del self.measured_frames[:]


        synced_event_begin = SpecialEvent()
        synced_event_begin.polarity = not self.invert_special_events
        synced_event_begin.ts = current_frame_t - current_exp_time/2
        synced_event_begin.header.seq = self.published_event_seq
        synced_event_begin.header.stamp = synced_event_begin.ts

        self.published_event_seq += 1

        synced_event_end = SpecialEvent()
        synced_event_end.polarity = self.invert_special_events
        synced_event_end.ts = current_frame_t + current_exp_time/2
        synced_event_end.header.seq = self.published_event_seq
        synced_event_end.header.stamp = synced_event_end.ts

        self.published_event_seq += 1

        self.event_publisher.publish(synced_event_begin)
        self.event_publisher.publish(synced_event_end)

        # advance to next frame
        self.last_published_frame_t += self.est_frame_period


        #rospy.loginfo("published frame: duration = {}, fps = {}".format(self.est_exposure_time, 1/self.est_frame_period.to_sec()))




################################################################################

if __name__ == "__main__":
    pll = SpecialEventsPLL()
    pll.main()
