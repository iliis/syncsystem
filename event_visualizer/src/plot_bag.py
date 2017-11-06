#!/usr/bin/env python

import rospy
import rosbag
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import datetime


def stamp(ts):
    return (ts.secs % 1000) + ts.nsecs / 1000000000.0

def timestamps_to_lines(ts, offset):
    lines = []
    for t in ts:
        s = stamp(t)
        lines.append( ((s,offset),(s,offset+1)) )
    return LineCollection(lines)


if __name__ == '__main__':
    bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/short_jitter_all_2017-11-06-19-49-49.bag')
    fig, ax = plt.subplots()

    ax.add_collection(timestamps_to_lines([msg.header.stamp for topic, msg, t in bag.read_messages(topics=['/camera/image_raw'])], 0))
    ax.add_collection(timestamps_to_lines([msg.ts           for topic, msg, t in bag.read_messages(topics=['/dvs/special_events'])], 1))
    ax.add_collection(timestamps_to_lines([msg.header.stamp for topic, msg, t in bag.read_messages(topics=['/synchronized/camera/image_raw'])], 2))
    ax.add_collection(timestamps_to_lines([msg.header.stamp for topic, msg, t in bag.read_messages(topics=['/cam_remapped_to_ev'])], 3))

    bag.close()

    ax.autoscale()
    ax.margins(.1)

    plt.show()
