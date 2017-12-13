#!/usr/bin/env python

import datetime
import math
import threading
import numpy as np

import rospy
from dvs_msgs.msg import SpecialEvent

################################################################################

class SpecialEventVerificator:

    def __init__(self):

        rospy.init_node('special_event_verificator', anonymous=True)

        rospy.Subscriber("/dvs/special_events", SpecialEvent, self.special_event_callback, queue_size=None)

        self.last_event = None

    #---------------------------------------------------------------------------

    def special_event_callback(self, event):

        if not self.last_event:
            self.last_event = event
            return

        def err(msg):
            rospy.logerr("{}: prev: {} {} current: {} {}".format(
                msg,
                self.last_event.ts, self.last_event.header.seq,
                event.ts, event.header.seq))

        if self.last_event.header.seq != event.header.seq-1:
            err("non-consecutive sequence numbers: {} and {}".format(self.last_event.header.seq, event.header.seq))

        if self.last_event.polarity == event.polarity:
            err("got two consecutive events with same polarity ({})".format(event.polarity))

        deltaT = event.ts - self.last_event.ts
        if deltaT < rospy.Duration.from_sec(0.001):
            err("very short delay between two events ({} ms)".format((event.ts-self.last_event.ts).to_sec()*1000))

        if event.polarity == False:
            if abs(deltaT - rospy.Duration.from_sec(0.002)) > rospy.Duration.from_sec(0.0001):
                err("expected high pulse of 2ms, got {}ms instead".format(deltaT.to_sec()*1000))
        else:
            if abs(deltaT - rospy.Duration.from_sec(0.004)) > rospy.Duration.from_sec(0.0001):
                err("expected low pulse of 4ms, got {}ms instead".format(deltaT.to_sec()*1000))

        self.last_event = event




    #---------------------------------------------------------------------------

################################################################################

if __name__ == "__main__":
    verificator = SpecialEventVerificator()
    rospy.spin()

