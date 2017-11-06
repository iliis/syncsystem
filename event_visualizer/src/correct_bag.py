#!/usr/bin/env python

import rospy
import rosbag
import datetime

# use internal timestamps instead of rosbag ones

if __name__ == '__main__':
    print "correcting timestamps"
    last_t = 0
    with rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/short_jitter_all2_2017-11-06-21-30-43.bag') as inbag:
        with rosbag.Bag('output.bag', 'w') as outbag:
            for topic, msg, t in inbag.read_messages():
                if topic == "/synchronized/camera/image_raw" \
                or topic == "/cam_remapped_to_ev" \
                or topic == "/synchronized/cam_remapped_to_ev":
                    outbag.write(topic, msg, msg.header.stamp)
                else:
                    outbag.write(topic, msg, t)

                if t-last_t > 1000000:
                    print ".",
                last_t = t
