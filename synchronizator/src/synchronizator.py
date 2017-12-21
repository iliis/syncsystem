#!/usr/bin/env python

import random

import datetime
import math
import threading
from enum import Enum
import numpy as np

import rospy
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from dvs_msgs.msg import SpecialEvent, Event, EventArray
from bluefox_ros.msg import CaptureInfo
from std_srvs.srv import Trigger, TriggerResponse
import message_filters

# Note: this code only ever works with special events (rising and falling
# edges) and never with normal events (brighter/darker). Thus, special events
# are just called 'events' throughout this code!


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

#-------------------------------------------------------------------------------

def floor_rest(f):
    i = int(math.floor(f))
    return i, f-i


################################################################################

class FrameMeasurement:
    def __init__(self, ts, exp_time, is_estimated=False, seq=None):
        self.seq      = seq
        self.ts       = ts
        self.exp_time = exp_time
        self.is_estimated = is_estimated

################################################################################

class ExpTimeSynchronizator:

    # PARAMETERS
    #############################################

    TOLERANCE = rospy.Duration.from_sec(0.0001) # difference between reported/configured exposure time and that measured by DAVIS is quite significant

    EXP_TIME_DELTA_US = 1000 # [us] change exposure time by this amount to identify frames

    TIMEOUT = rospy.Duration.from_sec(2) # timeout for waiting for data/synchronized frames etc.

    MISSING_EVENT_TIMEOUT = rospy.Duration.from_sec(0.2) # wait at least this long before publishing estimated events

    OFFSET_HISTORY_SIZE = 30 # keep offset of last N image-frames to detect dropped frames and such

    PLL_QUEUE_LEN = 30 # keep timing of last N event-frames to estimate parameters of missing events

    #---------------------------------------------------------------------------

    def __init__(self):

        # store measured frames (measured from special events) in here
        # these may contain holes tough, when events were lost!
        self.raw_frame_queue = []
        self.raw_frame_queue_mutex   = threading.Lock()
        self.raw_frame_queue_changed = threading.Event()
        self.last_event_received = None

        # raw frames from the raw_frame_queue are transfered into this queue
        # and any holes are filled by estimating the parameters of the missing
        # frame
        self.frame_queue = []
        self.last_frame_seq = 0 # frames are numbered incrementally, matching images to special events happens through this

        self.mutex = threading.Lock()
        self.got_new_image = threading.Event()

        rospy.init_node('event_visualizer', anonymous=True)


        self.skip_initial_image_count = rospy.get_param('~skip_first_frames', 0)
        if self.skip_initial_image_count > 0:
            rospy.loginfo("skipping initial {} frames".format(self.skip_initial_image_count))
        else:
            self.skip_initial_image_count = 0

        self.invert_special_events = rospy.get_param('~invert_special_events', False)


        self.configclient = dynamic_reconfigure.client.Client('/camera')

        rospy.Subscriber("/dvs/special_events", SpecialEvent, self.special_event_callback, queue_size=None)

        self.capture_info_sub = message_filters.Subscriber("/camera/capture_info", CaptureInfo, queue_size=None)
        self.camera_image_sub = message_filters.Subscriber("/camera/image_raw", Image, queue_size=None)

        self.synced_cam_info = message_filters.TimeSynchronizer([self.capture_info_sub, self.camera_image_sub], 10)
        #self.synced_cam_info.registerCallback(self.image_callback)


        self.capt_info_pub = rospy.Publisher("/synchronized/camera/capture_info", CaptureInfo, queue_size=0)
        self.image_pub     = rospy.Publisher("/synchronized/camera/image_raw", Image, queue_size=0)


        self.original_exp_time = None

        rospy.Service('resynchronize', Trigger, self.start_synchronization)

        self.timeout_timer = None

        self.last_img_seq = None
        self.last_sevent_seq = None


        self.reset_sync()

    ############################################################################
    # CALLBACK THREADS
    # The following functions all run in their own thread!

    def timeout(self, event):
        with self.mutex:
            rospy.logerr("TIMEOUT")

            self.reset_sync() # TODO: handle this in main loop?

    #---------------------------------------------------------------------------
    # a frame starts with a rising event and ends with a falling event
    # in the case of a DVS or normal DAVIS this is inverted by the sync circuitry
    # TODO: how to handle missing events? incorporate pll!

    def special_event_callback(self, event):
        with self.raw_frame_queue_mutex:
            if self.invert_special_events:
                event.polarity = not event.polarity

            good = True
            if self.last_event is not None:
                if event.header.seq != self.last_event.header.seq+1:
                    rospy.logwarn("Non-consecutive special event messages received!")
                    good = False
                if event.polarity == self.last_event.polarity:
                    rospy.logwarn("got two consecutive special events with {} edge, this should not happen".format(\
                            "rising" if event.polarity else "falling"))
                    good = False

                if event.polarity == False and good:
                    exp_time = event.ts - self.last_event.ts
                    self.raw_frame_queue.append(FrameMeasurement(event.ts + exp_time/2, exp_time))
                    self.raw_frame_queue_changed.set()

            self.last_event = event


    #---------------------------------------------------------------------------

    def image_callback(self, info, img):
        with self.mutex:
            if self.last_img_seq is not None:
                if img.header.seq != self.last_img_seq+1:
                    rospy.logerr("Non-consecutive image messages received!")

            self.last_img_seq = img.header.seq

            if self.skip_initial_image_count > 0:
                rospy.loginfo("skipping a frame")
                self.skip_initial_image_count -= 1
                return

            self.image_queue.append( (info, img) )
            self.got_new_image.set()

    ############################################################################
    # PLL THREAD
    # handles special events and collates them into frames
    # estimates frames if events are missing

    def event_pll(self):

        self.pll_is_locked = False
        self.pll_est_frame_t = None # phase (time of current frame)
        self.pll_est_inter_frame_duration = None # frequency
        self.pll_est_exposure_time = None
        self.pll_queue = []

        while not rospy.is_shutdown():
            # wait until we should publish our next image
            # TODO: consider using rospy.Rate()
            # TODO: sleep for MISSING_EVENT_TIMEOUT when we haven't gotten any matching events already

            # wait for more data
            if len(self.raw_frame_queue) == 0:
                self.raw_frame_queue_changed.clear()
                if not self.raw_frame_queue_changed.wait(timeout=self.TIMEOUT.to_sec()):
                    rospy.logfatal("timeout in PLL thread while waiting for special events")


            with self.raw_frame_queue_mutex, self.mutex:

                if not self.pll_is_locked:
                    self.enqueue_frame(self.raw_frame_queue[0])
                    del self.raw_frame_queue[0]
                else:
                    # look for good frames in event stream
                    for i in range(len(self.raw_frame_queue)):
                        if abs(self.raw_frame_queue[i].ts - self.pll_est_frame_t) <= self.pll_est_inter_frame_duration/2:

                            if i > 0:
                                rospy.logwarn("ignoring {} measured frames".format(i))

                            self.enqueue_frame(self.raw_frame_queue[i])

                            # throw away events we've used
                            del self.raw_frame_queue[:i+1]
                            break
                    else:
                        # throw away real frames that are older than what we estimated just now
                        for i, frame in enumerate(self.raw_frame_queue):
                            if frame.ts >= self.pll_est_frame_t:
                                break

                        if i > 0:
                            rospy.loginfo("ignoring {} measured frames which are older than current estimate".format(i))
                            del self.raw_frame_queue[i]

                        # no match found, estimate something instead
                        self.enqueue_frame(FrameMeasurement(self.pll_est_frame_t, self.pll_est_inter_frame_duration, is_estimated=True))


    #---------------------------------------------------------------------------

    def enqueue_frame(self, frame):

        if frame.is_estimated:
            rospy.logwarn("publishing image based on estimation: t={}, exposure time={}ms".format(frame.ts, frame.exp_time.to_sec()*1000))
        elif self.pll_is_locked:
            rospy.loginfo("offset: of estimate: {:7.3f}ms, actual: {:7.3f}ms"\
                .format(\
                (frame.ts - self.pll_est_frame_t).to_sec()*1000,
                (frame.ts - rospy.Time.now()).to_sec()*1000 )
            )

        #frame = FrameMeasurement(frame_t, exp_time, estimated)
        frame.seq = self.last_frame_seq
        self.last_frame_seq += 1

        self.frame_queue.append(frame)

        self.update_pll(frame)

    #---------------------------------------------------------------------------

    def update_pll(self, frame):

        actual_length = len([f for f in self.pll_queue if not f.is_estimated])

        if actual_length == self.PLL_QUEUE_LEN-1:
            print "got", self.PLL_QUEUE_LEN, "frames, syncstate: running"

        self.pll_queue.append(frame)
        if actual_length > self.PLL_QUEUE_LEN:
            del self.pll_queue[0] # TODO: use ring-buffer instead of appending/deleting

            # estimate exposure time
            # TODO: use average (numpy can't handle duration tough)
            self.pll_est_exposure_time = np.median([f.exp_time for f in self.pll_queue if not f.is_estimated])

            # estimate FPS
            # TODO: don't use estimated frames here!!!
            self.pll_est_inter_frame_duration = np.median([B.ts - A.ts for A,B in prevnext(self.pll_queue)])
            self.pll_est_frame_t = frame.ts + self.pll_est_inter_frame_duration

            self.pll_is_locked = True
        else:
            self.pll_is_locked = False




    ############################################################################
    # MAIN THREAD
    # the following functions shall only be called from the main-loop
    # (or while holding the mutex)

    def restart_timeout(self):
        if self.timeout_timer is not None:
            self.timeout_timer.shutdown()

        self.timeout_timer = rospy.Timer(self.TIMEOUT, self.timeout, oneshot=True)

    #---------------------------------------------------------------------------

    def reset_sync(self):
        self.currently_set_exp_time = None

        self.last_rising = None
        self.last_event  = None

        self.special_events = [] # type: List[Tuple[rospy.Time, rospy.Duration, int]]
        self.image_queue = []    # type: List[Tuple[CaptureInfo, Image]]
        # last N offsets to detect lost frames
        self.time_offset_hist = [(None,None,None)] * self.OFFSET_HISTORY_SIZE # type: List[(stamp, offset, duration)]
        self.time_offset_hist_idx = -self.OFFSET_HISTORY_SIZE

        self.seq_idx_offset = None
        self.matched_time_offset = None

        self.sync_state = 'wait_data'
        rospy.loginfo("SYNCSTATE: {}".format(self.sync_state))

        self.got_new_image.clear()

    #---------------------------------------------------------------------------

    def update_original_exp_time(self):
        if self.currently_set_exp_time is None:
            self.config = self.configclient.update_configuration({})
            self.currently_set_exp_time = self.config['exp_time']
            self.original_exp_time = self.currently_set_exp_time
            #rospy.loginfo("remembering original exposure time of {} ms".format(self.original_exp_time/1000.0))

    def start_synchronization(self):
        self.restart_timeout()
        rospy.loginfo("triggering synchronization")

        # toggle between two different exposure times
        if self.currently_set_exp_time == self.original_exp_time:
            # use something that isn't too close to the limits (0 or 10ms)
            if self.currently_set_exp_time < self.EXP_TIME_DELTA_US*2:
                t = self.currently_set_exp_time + self.EXP_TIME_DELTA_US
            else:
                t = self.currently_set_exp_time - self.EXP_TIME_DELTA_US
        else:
            # go back to original exposure time
            t = self.original_exp_time

        self.config = self.configclient.update_configuration({'exp_time': t})
        rospy.loginfo("changed exposure time to {} ms".format(self.config['exp_time']/1000.0))
        self.currently_set_exp_time = self.config['exp_time']

        response = TriggerResponse()
        response.success = True
        response.message = ""
        return response

    #---------------------------------------------------------------------------

    def match_exp_time(self, time_us, ref_us=None):
        if ref_us is None:
            ref_us = self.currently_set_exp_time

        return ref_us - ExpTimeSynchronizator.TOLERANCE_US <= time_us <= ref_us + ExpTimeSynchronizator.TOLERANCE_US

    #---------------------------------------------------------------------------

    def verify_exp_time_measurements(self):
        # check if we actually measure the exposure time we expect
        assert len(self.image_queue) > 0
        assert len(self.special_events) > 0


        duration_mean = np.median([ci.exp_time_us for ci,_ in self.image_queue])
        measured_mean = np.median([d.nsecs/1000.0 for _, d, _ in self.special_events])

        #rospy.loginfo("current exposure time: {} us, measured: {} us".format(duration_mean, measured_mean))

        if not self.match_exp_time(duration_mean, measured_mean):
            rospy.logfatal("Invalid exposure time: {} reports an exposure time of {} us, but we measured {} us".format(
                self.capture_info_sub.name, duration_mean, measured_mean))
            return False

        if not self.match_exp_time(duration_mean, self.original_exp_time):
            rospy.logfatal("Invalid exposure time: {} reports an exposure time of {} us, but dynamic reconfiguration on {} says exp_time should be {} us".format(
                self.capture_info_sub.name, duration_mean, self.configclient.name, self.original_exp_time))
            return False

        return True


    #---------------------------------------------------------------------------

    def check_for_exp_time_change(self):
        if len(self.image_queue) == 0 or len(self.special_events) == 0:
            return False

        first_img_exp_time = self.image_queue[0][0].exp_time_us
        if self.match_exp_time(first_img_exp_time):
            # this should not happen, but could if changing the exposure time failed or was done twice in too rapid succession
            rospy.logerr("Failed to match exposure time change: Oldest image in queue already has new exposure time.")
            return False

        first_event_exp_time = self.special_events[0][1].nsecs/1000.0
        if self.match_exp_time(first_event_exp_time):
            rospy.logerr("Failed to match exposure time change: Oldest events in queue already show new exposure time.")
            return False

        # look for change in exposure time of camera images

        skip_imgs = 1 # skip a frame, as camera_info reports exposure time change too early
        match_img_idx = None
        for idx, (info, img) in enumerate(self.image_queue):
            if self.match_exp_time(info.exp_time_us):
                if skip_imgs > 0:
                    skip_imgs -= 1
                    continue

                match_img_idx = idx
                break
            elif not self.match_exp_time(info.exp_time_us, first_img_exp_time):
                rospy.logerr("Got unexpected exposure time of {} ms. Expected either {} ms or {} ms.".format(
                    info.exp_time_us/1000.0, self.currently_set_exp_time/1000.0, first_img_exp_time/1000.0
                    ))
                self.reset_sync()
                return False

        if not match_img_idx:
            return False # wait for more frames

        # look for change in exposure time in special events

        match_event_idx = None
        for idx, (stamp, duration, seq) in enumerate(self.special_events):
            if self.match_exp_time(duration.nsecs/1000.0):
                match_event_idx = idx
                break
            elif not self.match_exp_time(duration.nsecs/1000.0, first_event_exp_time):
                rospy.logerr("Measured unexpected exposure time of {} ms. Expected either {} ms or {} ms.".format(
                    duration.nsecs/1000000.0, self.currently_set_exp_time/1000.0, first_event_exp_time/1000.0
                    ))
                self.reset_sync()
                return False

        if not match_event_idx:
            return False # wait for more special events

        self.seq_idx_offset      = self.image_queue[match_img_idx][0].header.seq   - self.special_events[match_event_idx][2]
        self.matched_time_offset = self.image_queue[match_img_idx][0].header.stamp - self.special_events[match_event_idx][0]

        rospy.loginfo("got match at {} ms offset (seq diff: {})".format( \
                self.matched_time_offset.to_sec()*1000, \
                self.seq_idx_offset ))


        # delete data from before match
        del self.image_queue[0:match_img_idx]
        del self.special_events[0:match_event_idx]

        # we've got a good match!
        return True

    #---------------------------------------------------------------------------

    # drops either an image frame (count < 0) or a special event pair (count > 0)
    def drop_frame(self, count):
        if count > 0:
            if len(self.special_events) > 0:
                del self.special_events[0]
                rospy.logwarn("deleting an event")
        elif count < 0:
            if len(self.image_queue) > 0:
                # TODO: don't delete image, insert fake event instead, using guess based on past N frames
                del self.image_queue[0]
                rospy.logwarn("deleting an image")
            else:
                self.skip_initial_image_count = 1

        # drop history so far
        self.time_offset_hist_idx = -self.OFFSET_HISTORY_SIZE


    #---------------------------------------------------------------------------

    def publish(self):
        while len(self.image_queue) > 0 and len(self.special_events) > 0:

            (info, img)            = self.image_queue.pop(0)
            (stamp, duration, seq) = self.special_events.pop(0)
            offset = info.header.stamp - stamp

            # check if match is still good

            if not self.match_exp_time(info.exp_time_us, duration.nsecs/1000):
                rospy.logwarn("publishing image with significantly different exposure time than measured: " \
                        + "{} ms reported by capture_info, {} ms measured by DAVIS.".format(info.exp_time_us/1000.0, duration.to_sec()*1000))

                # use theoretical exposure time (which is wrong when exposure
                # time is set higher than the driver allows, but correct when
                # events are missing/wrong)
                duration = info.exp_time_us

            # do we have a good amount of past data?
            # otherwise just go on and hope for the best
            if self.time_offset_hist_idx >= 0:
                offset_mean   = np.median([h[1] for h in self.time_offset_hist])
                offset_stddev = np.std(   [h[1] for h in self.time_offset_hist])

                seq_offset_err = (info.header.seq - seq) - self.seq_idx_offset

                fps_mean = np.median([(b[0]-a[0]).to_sec() for a, b in prevnext(self.time_offset_hist)])

                #if abs(offset.to_sec()-offset_mean) > offset_stddev*4 + 0.01: # add a const. term as stddev can get quite small
                if seq % 100 == 0:
                    rospy.loginfo("{:5d} {:5d} {} Offset: current: {:5.1f}, mean: {:5.1f} ms, stddev: {:5.2f} ms, FPS: mean: {:4.1f}, err: {:7.2f}" \
                        .format(\
                        seq, info.header.seq, seq_offset_err,
                        offset.to_sec()*1000, offset_mean*1000, offset_stddev*1000, 1/fps_mean, \
                        (abs(offset.to_sec()-offset_mean)-offset_stddev*4)*1000 ))


                if abs(offset_mean-self.matched_time_offset.to_sec()) > fps_mean/2:
                    rospy.logwarn("we seem to be off by more than half a frame, resyncing")
                    #self.drop_frame(1 if offset_mean > 0 else -1)

                """
                if abs(offset.to_sec()-offset_mean) > offset_stddev*4 + 0.01: # add a const. term as stddev can get quite small
                    rospy.logwarn("Unexpected offset measured, last {} were {} ms (+/- {}), but this is {} ms" \
                            .format(self.OFFSET_HISTORY_SIZE, offset_mean*1000, offset_stddev*1000, offset.to_sec()*1000))

                    # missing frame recovery doesn't really work, just resync if we're off
                    # offset is very dependent on CPU usage and can easily be off be a frame or two for a moment

                    # assume we've lost some frames
                    offset_frames, offset_fraction = floor_rest( (offset.to_sec() - offset_mean) / fps_mean )
                    if 0.5-abs(offset_fraction-0.5) < 0.2: # TODO: this is just a guess
                        rospy.logwarn("Detected loss of {} frames.".format(abs(offset_frames)))
                        if abs(offset_frames) >= self.OFFSET_HISTORY_SIZE/2:
                            rospy.logerr("This seems a bit high, aborting.")
                            return False

                        # try to delete corresponding frames from other camera
                        if offset_frames > 0:
                            rospy.loginfo("Lost images.")
                            if offset_frames > len(self.special_events):
                                self.special_events = []
                            else:
                                del self.special_events[0:offset_frames]
                        elif offset_frames < 0:
                            rospy.logwarn("Lost special events. Please check sync cable!")
                            if offset_frames > len(self.image_queue):
                                self.image_queue = []
                            else:
                                del self.image_queue[0:offset_frames]
                        else:
                            rospy.loginfo("ignoring offset fraction of {}".format(offset_fraction))


                        continue # try again
                    else:
                        if abs(offset_frames) > 0:
                            rospy.logerr("offset is off by a non-integer fraction of a frame ({}). Can't recover, try to sync again."\
                                    .format(offset_frames+offset_fraction))
                            return False
                    """

                """
                if seq_offset_err != 0:
                    rospy.logerr("sequence numbers got out of sync, something must have gone wrong. actual: {} expected: {}".format( \
                            info.header.seq - seq, self.seq_idx_offset))

                    # assume we've lost some frames
                    if seq_offset_err > 0:
                        rospy.logerr("Lost images.")
                        if len(self.special_events) > 0:
                            del self.special_events[0]
                            rospy.loginfo("deleting an event")
                    else:
                        seq_offset_err = abs(seq_offset_err)
                        rospy.logerr("Lost special events.")
                        if len(self.image_queue) > 0:
                            del self.image_queue[0]
                            rospy.loginfo("deleting an image")
                        else:
                            self.skip_initial_image_count = 1

                    continue
                    """

            # keep track of last N frames

            self.time_offset_hist_idx += 1
            if self.time_offset_hist_idx >= self.OFFSET_HISTORY_SIZE:
                self.time_offset_hist_idx = 0

            self.time_offset_hist[self.time_offset_hist_idx] = ( stamp, offset.to_sec(), duration )

            # actually publish synchronized image

            #rospy.loginfo("published synced image pair. offset: {} ms.".format(offset.to_sec()*1000))

            info.header.stamp = stamp
            img .header.stamp = stamp

            self.image_pub.publish(img)
            self.capt_info_pub.publish(info)

        return True

    #---------------------------------------------------------------------------

    # main state-machine
    def main(self):
        while not rospy.is_shutdown():
            self.got_new_image.wait(timeout=self.TIMEOUT.to_sec())
            if not self.got_new_image.is_set():
                rospy.logwarn("Timeout in main thread while waiting for data.")
                continue

            with self.mutex:
                self.got_new_image.clear()

                if self.sync_state == 'wait_data':
                    # we're waiting for data from both cameras
                    if len(self.image_queue) > 0 and len(self.special_events) > 0:

                        self.update_original_exp_time()
                        if not self.verify_exp_time_measurements():
                            return # fatal, shut-down...

                        self.start_synchronization()
                        self.sync_state = 'wait_match1'

                        # throw data away, as it might be bad. Also, it
                        # would mess up check_for_exp_time_change() as it
                        # assumes the oldest entries have a different
                        # exposure time
                        self.image_queue = []
                        self.special_events = []

                        rospy.loginfo("SYNCSTATE: {}".format(self.sync_state))

                elif self.sync_state == 'wait_match1':
                    if self.check_for_exp_time_change():
                        self.start_synchronization()
                        self.sync_state = 'wait_match2'
                        rospy.loginfo("SYNCSTATE: {}".format(self.sync_state))

                elif self.sync_state == 'wait_match2':
                    if self.check_for_exp_time_change():
                        self.timeout_timer.shutdown()
                        self.sync_state = 'publish'
                        rospy.loginfo("SYNCSTATE: {}".format(self.sync_state))

                elif self.sync_state == 'publish':
                    if not self.publish():
                        self.reset_sync()

                        rospy.loginfo("SYNCSTATE: {}".format(self.sync_state))

                else:
                    raise Exception("Invalid State '{}'. This should not happen, please report it.".format(self.sync_state))

################################################################################

if __name__ == "__main__":
    e = ExpTimeSynchronizator()
    #e.main()
    e.event_pll()

################################################################################
