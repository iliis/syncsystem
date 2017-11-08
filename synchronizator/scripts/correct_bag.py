#!/usr/bin/env python

import rospy
import rosbag
import datetime

# use internal timestamps instead of rosbag ones

if __name__ == '__main__':
    print "correcting timestamps"
    with rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/startup_sync_40fps_2017-11-07-12-46-09.bag') as inbag:
        with rosbag.Bag('output.bag', 'w') as outbag:

            for topic, msg, t in inbag.read_messages():

                if topic == "/synchronized/camera/image_raw" \
                or topic == "/synchronized/camera/capture_info" \
                or topic == "/camera/image_raw" \
                or topic == "/camera/capture_info":
                    outbag.write(topic, msg, msg.header.stamp)
                else:
                    outbag.write(topic, msg, t)

                progress = (t.to_sec() - inbag.get_start_time()) / (inbag.get_end_time() - inbag.get_start_time())
                print "\x1b[G{:3.1f} %".format(progress*100), 
