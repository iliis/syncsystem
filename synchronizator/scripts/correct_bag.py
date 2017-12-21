#!/usr/bin/env python

import rospy
import rosbag
import genpy
import datetime

# use internal timestamps instead of rosbag ones

def copytime(t):
    return rospy.Time(t.secs, t.nsecs)

if __name__ == '__main__':
    rospy.init_node('correct_bag', anonymous=True)
    infile = rospy.get_param('~infile')

    cam_info = None
    first_timestamp = None

    print "correcting timestamps of", infile
    with rosbag.Bag(infile) as inbag:

        for topic, msg, t in inbag.read_messages(topics=['/dvs/camera_info']):
            # workaround for broken timestamp headers in /dvs/camera_info
            # store camera_info for DVS
            cam_info = msg
            break

        with rosbag.Bag('output.bag', 'w') as outbag:

            for topic, msg, t in inbag.read_messages():

                if rospy.is_shutdown():
                    exit(0)

                # ignore ros messages
                if topic.startswith('/rosout'):
                    continue

                # ignore normal events, they take a long time to fix
                #if topic == '/dvs/events':
                    #continue

                if topic == '/dvs/camera_info':
                    continue

                if not topic.endswith('sync'):
                    continue

                timestamp = t

                if hasattr(msg, 'header'):
                    if msg.header.stamp.secs == 0 and msg.header.stamp.nsecs == 0:
                        print "WARNING: invalid header timestamp found in", topic, ", using rosbag metadata timestamp"
                        delta = (rospy.Time(1513647069, 699662341) - rospy.Time(1513262199, 422110210))
                        timestamp = t - delta
                        msg.header.stamp = timestamp
                    else:
                        timestamp = copytime(msg.header.stamp)
                elif isinstance(msg, rospy.Time):
                    timestamp = copytime(msg)
                elif isinstance(msg.data, rospy.Time) or isinstance(msg.data, genpy.rostime.Time):
                    timestamp = copytime(msg.data)
                else:
                    print "WARNING: no timestamp for", topic, "found!"

                outbag.write(topic, msg, timestamp)

                # publish dvs camera info for ALL images (so we can properly warp events from the beginning)
                if topic.endswith('/image_raw') and cam_info is not None:
                    cam_info.header.stamp = timestamp
                    outbag.write('/dvs/camera_info', cam_info, timestamp)

                if first_timestamp is None:
                    first_timestamp = timestamp

                progress = (t.to_sec() - inbag.get_start_time()) / (inbag.get_end_time() - inbag.get_start_time())
                print "\x1b[G{:3.1f} %".format(progress*100), 
