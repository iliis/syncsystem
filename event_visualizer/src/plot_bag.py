#!/usr/bin/env python

import itertools
import datetime
import math

import rospy
import rosbag
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


def print_progress(current_t, start_t, end_t):
    progress = (current_t.to_sec() - start_t) / (end_t-start_t)
    print "\x1b[G{:3.1f} %".format(progress*100), 


def stamp(ts):
    return (ts.secs % 1000) + ts.nsecs / 1000000000.0

def timestamps_to_lines(ts, offset=0):
    lines = []
    for t in ts:
        lines.append( ((t,offset),(t,offset+1)) )
    return LineCollection(lines)


def to_scaled_color(val, _min, _max):
    c = (val-_min) / float(_max-_min)
    if c > 1:
        c = 1
    elif c < 0:
        c = 0
    return ( c, 0, 0, 1)

def to_lines(objs, get_t, get_col=None, offset=0):
    lines = []
    colors = []

    def to_line(t):
        return ((t,offset), (t,offset+1))

    for obj in objs:
        ls = get_t(obj)

        if get_col:
            col = get_col(obj)

            if isinstance(ls, list):
                colors.extend([col]*len(ls))
            else:
                colors.append(col)

        if isinstance(ls, list):
            lines.extend(map(to_line, ls))
        else:
            lines.append(to_line(ls))

    return LineCollection(lines, colors=colors)

def render_events(msgs, binsize, total_duration, scale_max_to=None):
    bins_pos = []
    bins_neg = []
    cur_bin_t = None
    count_pos = 0
    count_neg = 0
    first_t = None
    for _, msg, t in msgs:
        for e in msg.events:
            if cur_bin_t is None:
                cur_bin_t = e.ts
                first_t = e.ts

            while cur_bin_t+binsize < e.ts:
                bins_pos.append(count_pos)
                bins_neg.append(count_neg)
                count_pos = 0
                count_neg = 0
                cur_bin_t += binsize

            if e.polarity:
                count_pos += 1
            else:
                count_neg += 1

        print_progress(t, total_duration[0], total_duration[1])

    if scale_max_to is not None:
        s = scale_max_to / float(max(max(bins_pos), max(bins_neg)))

        bins_pos = [b * s for b in bins_pos]
        bins_neg = [b * s for b in bins_neg]

    plt.plot(np.linspace(stamp(first_t), stamp(cur_bin_t), len(bins_pos)), bins_pos, 'blue')
    plt.plot(np.linspace(stamp(first_t), stamp(cur_bin_t), len(bins_neg)), bins_neg, 'red')


class SyncFrame:
    def __init__(self, start_t, end_t, ros_start_t, ros_end_t):
        self.start_t = start_t
        self.end_t   = end_t
        self.duration = end_t - start_t
        self.center = (end_t + start_t) / 2

        self.ros_start_t = ros_start_t
        self.ros_end_t   = ros_end_t
        #print self.duration*1000


if __name__ == '__main__':
    #bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/startup_sync_40fps_2017-11-07-12-46-09.bag')
    #bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/blink_to_bluefox_per5_frames_2017-11-07-14-26-12.bag')
    bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/blink_fresh4_otherresetorder_2017-11-08-18-36-29.bag')

    fig, ax = plt.subplots()

    start_t = bag.get_start_time()
    end_t   = bag.get_end_time()

    bridge = CvBridge()

    #ax.add_collection(timestamps_to_lines([msg.header.stamp for topic, msg, t in bag.read_messages(topics=['/camera/image_raw'])], 0))
    #ax.add_collection(timestamps_to_lines([msg.ts           for topic, msg, t in bag.read_messages(topics=['/dvs/special_events'])], 1))
    #ax.add_collection(timestamps_to_lines([msg.header.stamp for topic, msg, t in bag.read_messages(topics=['/synchronized/camera/image_raw'])], 2))
    #ax.add_collection(timestamps_to_lines([msg.header.stamp for topic, msg, t in bag.read_messages(topics=['/cam_remapped_to_ev'])], 3))


    print "rendering special events..."

    events = [(t, msg) for topic, msg, t in bag.read_messages(topics=['/dvs/special_events'])]

    sync_frames = []
    for i in range(len(events)-1):
        if events[i][1].polarity == True and events[i+1][1].polarity == False:
            sync_frames.append(SyncFrame(stamp(events[i][1].ts), stamp(events[i+1][1].ts), stamp(events[i][0]), stamp(events[i+1][0])))

    durations = [f.duration for f in sync_frames]
    #lines.set_color( [ (1,0,0,1), (1,0,0,0), (1,1,0,1) ] )



    lines = []
    colors = []
    for f in sync_frames:
        lines.append(f.start_t)
        lines.append(f.end_t)

        #colors.append( to_scaled_color(f.duration, 0.008, 0.009) )
        #colors.append( to_scaled_color(f.duration, 0.008, 0.009) )
        colors.append( (0,0,1,1) )
        colors.append( (1,0,0,1) )

    collection = timestamps_to_lines(lines, -1)
    collection.set_color(colors)

    ax.add_collection(collection)

    print "rendering image frames..."

    def mean_img_val(img):
        cv_image = bridge.imgmsg_to_cv2(img) #, desired_encoding="passthrough")
        return cv_image.mean()

    #ax.add_collection(timestamps_to_lines([stamp(msg.header.stamp) for topic, msg, t in bag.read_messages(topics=['/camera/image_raw'])], 1))


    image_events = [ (stamp(msg.header.stamp), mean_img_val(msg)) for topic, msg, t in bag.read_messages(topics=['/camera/image_raw']) ]

    min_imgval = min([v[1] for v in image_events])
    max_imgval = max([v[1] for v in image_events])
    stddev_imgval = np.std([v[1] for v in image_events])
    
    print "image intensity: min:", min_imgval, "max:", max_imgval, "stddev:", stddev_imgval

    ax.add_collection(to_lines(
        image_events,
        lambda m: m[0],
        lambda m: to_scaled_color(m[1], min_imgval, max_imgval),
        -2
        ))

    """
    ax.add_collection(to_lines(
        [msg for topic, msg, t in bag.read_messages(topics=['/camera/image_raw'])],
        lambda msg: stamp(msg.header.stamp),
        #lambda msg: to_scaled_color(msg.exp_time_us, 8000, 9000),
        lambda m: to_scaled_color(mean_img_val(m), 0.6, 3),
        -2
        ))
        """

    #ax.add_collection(timestamps_to_lines([stamp(msg.header.stamp) for topic, msg, t in bag.read_messages(topics=['/dvs/events'])], 2))

    """
    for topic, msg, t in bag.read_messages(topics=['/camera/capture_info', '/camera/image_raw']):
        if topic == '/camera/capture_info':
            #print msg.header.stamp, t.to_sec()-start_t, msg.exp_time_us
            pass
        elif topic == '/camera/image_raw':
            print msg.header.stamp, mean_img_val(msg)
    """


    print "rendering events..."
    render_events(bag.read_messages(topics=['/dvs/events']), rospy.Duration.from_sec(0.0001), (start_t, end_t), scale_max_to=1)
    print ""

    print "calculating offsets..."
    last_special_event = None
    last_image_transition = None
    last_image_avg = None
    for topic, msg, t in bag.read_messages(topics=['/camera/image_raw', '/dvs/special_events']):
        if topic == '/dvs/special_events':
            if last_special_event is not None:
                print "WARNING: unmatched special event"
            last_special_event = msg
        elif topic == '/camera/image_raw':

            avg = mean_img_val(msg)

            if last_image_avg is not None and abs(last_image_avg[1] - avg) > stddev_imgval*1.5:
                if avg-last_image_avg[1] > 0:
                    polarity = True
                else:
                    polarity = False

                #print "got transition of", abs(last_image_avg[1] - avg)

                if last_image_transition is not None:
                    print "WARNING: unmatched image transition"

                last_image_transition = (last_image_avg[0], msg.header.stamp, polarity)

            last_image_avg = (msg.header.stamp, avg)

        # align
        if last_special_event is not None and last_image_transition is not None:
            d1 = (last_image_transition[0] - last_special_event.ts).to_sec() * 1000
            d2 = (last_image_transition[1] - last_special_event.ts).to_sec() * 1000
            print "got match, delta =", d1, "ms to", d2, "ms", last_image_transition[2] == last_special_event.polarity
            last_special_event = None
            last_image_transition = None

        #print_progress(t, start_t, end_t)

    print "done"

    bag.close()

    ax.autoscale()
    ax.margins(.1)

    plt.show()
