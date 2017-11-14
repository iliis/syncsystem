#!/usr/bin/env python

import datetime
import math
import threading
import numpy as np

import rospy
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from dvs_msgs.msg import SpecialEvent, Event, EventArray
from bluefox_ros.msg import CaptureInfo
from std_srvs.srv import Trigger, TriggerResponse
import message_filters



# TODO: prevent changes to exposure time while waiting for previous change


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

class ExpTimeTest:

    # PARAMETERS
    #############################################

    TOLERANCE_US = 50 # [us] difference between reported/configured exposure time and that measured by DAVIS is quite significant

    EXP_TIME_DELTA = 1000 # [us] change exposure time by this amount to identify frames

    TIMEOUT = rospy.Duration.from_sec(2) # timeout for waiting for data/synchronized frames etc.

    OFFSET_HISTORY_SIZE = 10 # keep offset of last N frames to detect dropped frames and such

    #---------------------------------------------------------------------------

    def __init__(self):

        self.mutex = threading.Lock()

        rospy.init_node('event_visualizer', anonymous=True)

        self.configclient = dynamic_reconfigure.client.Client('/camera')

        rospy.Subscriber("/dvs/special_events", SpecialEvent, self.special_event_callback)

        self.capture_info_sub = message_filters.Subscriber("/camera/capture_info", CaptureInfo)
        self.camera_image_sub = message_filters.Subscriber("/camera/image_raw", Image)

        self.synced_cam_info = message_filters.TimeSynchronizer([self.capture_info_sub, self.camera_image_sub], 10)
        self.synced_cam_info.registerCallback(self.image_callback)


        self.capt_info_pub = rospy.Publisher("/synchronized/camera/capture_info", CaptureInfo, queue_size=2)
        self.image_pub     = rospy.Publisher("/synchronized/camera/image_raw", Image, queue_size=2)


        self.original_exp_time = None

        #rospy.Service('resynchronize', Trigger, self.start_synchronization)

        self.timeout_timer = None


        self.reset_sync()

    ############################################################################
    # CALLBACK THREADS
    # The following functions all run in their own thread!

    def timeout(self, event):
        with self.mutex:
            rospy.logerr("TIMEOUT")

            self.reset_sync() # TODO: handle this in main loop?

    #---------------------------------------------------------------------------

    def special_event_callback(self, event):
        with self.mutex:
            if event.polarity == False:
                self.last_falling = event
            elif self.last_falling is not None:

                duration = event.ts - self.last_falling.ts
                stamp    = self.last_falling.ts + duration/2 # middle of frame

                self.special_events.append( (stamp, duration) )
                self.got_new_data = True

            """
                return

            #print "got sync frame (exp_time:", (event.ts - self.last_falling).to_sec()*1000*1000, ", ts:", event.ts.to_sec(), ")"
            if self.first_good_syncframe is None and self.match_exp_time((event.ts - self.last_falling).nsecs/1000):
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
            """

    #---------------------------------------------------------------------------

    def image_callback(self, info, img):
        with self.mutex:
            self.image_queue.append( (info, img) )
            self.got_new_data = True

            """
            #print "got camera frame (exp_time:", img.exp_time_us, ", ts:", img.header.stamp.to_sec(), ")"
            if self.first_good_cameraframe is None and self.match_exp_time(info.exp_time_us):
                #print("MATCH!")
                self.first_good_cameraframe = info.header.stamp
                self.first_good_image_seq   = img.header.seq
                self.check_first_good()

            if self.image_event_seq_offset is not None:
                self.image_queue.append((info, img))
                self.check_pending_buffers()

            if self.currently_set_exp_time is None:
                self.start_synchronization(None)
            """

    #---------------------------------------------------------------------------


    """
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
    """


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

        self.last_falling = None

        self.special_events = [] # type: List[Tuple[rospy.Time, rospy.Duration]]
        self.image_queue = []    # type: List[Tuple[CaptureInfo, Image]]
        # last N offsets to detect lost frames
        self.time_offset_hist = [(None,None)] * self.OFFSET_HISTORY_SIZE # type: List[(stamp, duration)]
        self.time_offset_hist_idx = -self.OFFSET_HISTORY_SIZE


        self.sync_state = 'wait_data'
        rospy.loginfo("SYNCSTATE: {}".format(self.sync_state))

        self.got_new_data = False

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
            if self.currently_set_exp_time < self.EXP_TIME_DELTA*2:
                t = self.currently_set_exp_time + self.EXP_TIME_DELTA
            else:
                t = self.currently_set_exp_time - self.EXP_TIME_DELTA
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

        return ref_us - ExpTimeTest.TOLERANCE_US <= time_us <= ref_us + ExpTimeTest.TOLERANCE_US

    #---------------------------------------------------------------------------

    def verify_exp_time_measurements(self):
        # check if we actually measure the exposure time we expect
        assert len(self.image_queue) > 0
        assert len(self.special_events) > 0


        duration_mean = np.median([ci.exp_time_us for ci,_ in self.image_queue])
        measured_mean = np.median([d.nsecs/1000.0 for _, d in self.special_events])

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
        assert len(self.image_queue) > 0
        assert len(self.special_events) > 0

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
        for idx, (stamp, duration) in enumerate(self.special_events):
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

        rospy.loginfo("got match at {} ms offset".format( \
                (self.image_queue[match_img_idx][0].header.stamp - self.special_events[match_event_idx][0]).to_sec()*1000 ))

        # delete data from before match
        del self.image_queue[0:match_img_idx]
        del self.special_events[0:match_event_idx]

        return True

    #---------------------------------------------------------------------------

    def publish(self):
        while len(self.image_queue) > 0 and len(self.special_events) > 0:

            (info, img)       = self.image_queue.pop(0)
            (stamp, duration) = self.special_events.pop(0)
            offset = info.header.stamp - stamp

            # check if match is still good

            if not self.match_exp_time(info.exp_time_us, duration.nsecs/1000):
                # TODO: take offset by single frame into account!
                rospy.logwarn("publishing image with significantly different exposure time than measured: " \
                        + "{} ms reported by capture_info, {} ms measured by DAVIS.".format(info.exp_time_us/1000.0, duration.to_sec()*1000))

            # do we have a good amount of past data?
            # otherwise just go on and hope for the best
            if self.time_offset_hist_idx >= 0:
                offset_mean   = np.median([o for _, o in self.time_offset_hist])
                offset_stddev = np.std(   [o for _, o in self.time_offset_hist])

                fps_mean = np.median([(b-a).to_sec() for (a,_),(b,_) in prevnext(self.time_offset_hist)])

                #rospy.loginfo("Offset: mean: {} ms, stddev: {} ms, FPS: mean: {}".format(offset_mean*1000, offset_stddev*1000, 1/fps_mean))

                if abs(offset.to_sec()-offset_mean) > offset_stddev*10+10: # add a const. term as stddev can get quite small
                    rospy.logwarn("Unexpected offset measured, last {} were {} ms (+/- {}), but this is {} ms" \
                            .format(self.OFFSET_HISTORY_SIZE, offset_mean*1000, offset_stddev*1000, offset.to_sec()*1000))

                    # assume we've lost some frames
                    offset_frames, offset_fraction = floor_rest( (offset.to_sec() - offset_mean) / fps_mean )
                    if offset_fraction < 0.1: # TODO: this is just a guess
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
                            rospy.logerr("offset is off by a non-integer fraction of a frame. Can't recover, try to sync again.")
                            return False

            # keep track of last N frames

            self.time_offset_hist_idx += 1
            if self.time_offset_hist_idx >= self.OFFSET_HISTORY_SIZE:
                self.time_offset_hist_idx = 0

            self.time_offset_hist[self.time_offset_hist_idx] = ( stamp, offset.to_sec() )

            # actually publish synchronized image

            rospy.loginfo("published synced image pair. offset: {} ms.".format(offset.to_sec()*1000))

            info.header.stamp = stamp
            img .header.stamp = stamp

            self.image_pub.publish(img)
            self.capt_info_pub.publish(info)

        return True

    #---------------------------------------------------------------------------

    # main state-machine
    def main(self):
        while not rospy.is_shutdown():
            with self.mutex:

                if self.got_new_data:
                    self.got_new_data = False

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

            rospy.sleep(rospy.Duration.from_sec(0.1))

################################################################################

if __name__ == "__main__":
    e = ExpTimeTest()
    e.main()

################################################################################
