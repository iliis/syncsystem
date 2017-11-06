#!/usr/bin/env python

import rospy
import rosbag
import datetime

if __name__ == '__main__':
    #with rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/short_jitter.bag') as bag:
    with rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/short_jitter_all_nativetime.bag') as bag:
        prev_t = None
        for topic, msg, t in bag.read_messages(topics=['/cam_remapped_to_ev']):
            ok = "?"
            if prev_t:
                if prev_t < t:
                    ok = "OK"
                else:
                    ok = "FAIL"

            print t, msg.header.stamp.secs, "{:09d}".format(msg.header.stamp.nsecs), ok
            prev_t = t
