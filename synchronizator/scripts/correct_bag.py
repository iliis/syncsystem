#!/usr/bin/env python

import rospy
import rosbag
import datetime

# use internal timestamps instead of rosbag ones

if __name__ == '__main__':
    rospy.init_node('correct_bag', anonymous=True)
    infile = rospy.get_param('~infile')

    print "correcting timestamps of", infile
    with rosbag.Bag(infile) as inbag:
        with rosbag.Bag('output.bag', 'w') as outbag:

            for topic, msg, t in inbag.read_messages():

                if hasattr(msg, 'header'):
                    outbag.write(topic, msg, msg.header.stamp)
                else:
                    outbag.write(topic, msg, t)

                progress = (t.to_sec() - inbag.get_start_time()) / (inbag.get_end_time() - inbag.get_start_time())
                print "\x1b[G{:3.1f} %".format(progress*100), 
