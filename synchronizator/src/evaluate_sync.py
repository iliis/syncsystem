#!/usr/bin/env python

import itertools
import datetime
import math
import pickle
import hashlib

import rospy
import rosbag
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import scipy as sp

from scipy.interpolate import interp1d



VERT_POS = 280
VERT_POS_DVS = 90

MIN_HORIZ_CAM = 260
MAX_HORIZ_CAM = 520

DVS_WIDTH = 350 # 240 for DAVIS

# adjust those so the DVS plot looks good ;)
EVENT_HIST_WINDOW_SIZE = rospy.Duration.from_sec(0.01)
EVENT_HIST_WINDOW_INC  = rospy.Duration.from_sec(0.002)




# from http://scipy-cookbook.readthedocs.io/items/SignalSmooth.html
def smooth(x,window_len=11,window='hanning'):
    """smooth the data using a window with requested size.
    
    This method is based on the convolution of a scaled window with the signal.
    The signal is prepared by introducing reflected copies of the signal 
    (with the window size) in both ends so that transient parts are minimized
    in the begining and end part of the output signal.
    
    input:
        x: the input signal 
        window_len: the dimension of the smoothing window; should be an odd integer
        window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
            flat window will produce a moving average smoothing.

    output:
        the smoothed signal
        
    example:

    t=linspace(-2,2,0.1)
    x=sin(t)+randn(len(t))*0.1
    y=smooth(x)
    
    see also: 
    
    numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
    scipy.signal.lfilter
 
    TODO: the window parameter could be the window itself if an array instead of a string
    NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
    """

    if x.ndim != 1:
        raise ValueError, "smooth only accepts 1 dimension arrays."

    if x.size < window_len:
        raise ValueError, "Input vector needs to be bigger than window size."


    if window_len<3:
        return x


    if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
        raise ValueError, "Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'"


    s=np.r_[x[window_len-1:0:-1],x,x[-2:-window_len-1:-1]]
    #print(len(s))
    if window == 'flat': #moving average
        w=np.ones(window_len,'d')
    else:
        w=eval('np.'+window+'(window_len)')

    y=np.convolve(w/w.sum(),s,mode='valid')
    w2 = int(window_len/2)
    return y[w2:-w2] # cut back to original size



last_label = None
def print_progress(label, current_t, start_t, end_t):
    global last_label
    if not label == last_label:
        last_label = label
        print "" # go to new line

    progress = (current_t - start_t) / (end_t-start_t)
    print "\x1b[A\x1b[G{} {:3.1f} %".format(label, progress*100)





def find_edge(data):

    #diff = abs(data[1:] - data[:-1])
    # actually, there is no need for abs(), as long as the darker side is to the left ;)
    diff = np.ediff1d(np.array(data, dtype='float'))

    pos = np.argmax(diff)

    # improve accuracy by using weighted average over neighbouring pixels
    neighborhood = diff[pos-3:pos+4]

    return float(np.linspace(pos-3, pos+3, 7).dot(neighborhood / sum(neighborhood)))


# guess f(x) by interpolating between y1=f(x1) and y2=f(x2)
def interpolate(x1, y1, x2, y2, x):
    assert x1 <= x <= x2

    return (x-x1)/(x2-x1) * (y2-y1) + y1


def correlate(camera_pos, dvs_pos, t_offset):

    camera_pos_iter = iter(camera_pos)
    dvs_pos_iter    = iter(dvs_pos)

    accum = 0
    try:
        dvs_next_t, dvs_next_pos = next(dvs_pos_iter)

        while True: # iterate over all camera framesj
            cam_t, cam_pos = next(camera_pos_iter)

            # find events before and after current frame
            while True:
                dvs_prev_t, dvs_prev_pos = dvs_next_t, dvs_next_pos
                dvs_next_t, dvs_next_pos = next(dvs_pos_iter)
                if cam_t+t_offset < dvs_next_t:
                    break

            if cam_t+t_offset < dvs_prev_t:
                continue # ignore frames that aren't between two dvs position estimates

            if dvs_next_t - dvs_prev_t >= 0.05:
                continue # ignore frames that don't have a close position estimate from dvs

            accum += abs(interpolate(dvs_prev_t, dvs_prev_pos, dvs_next_t, dvs_next_pos, cam_t+t_offset) - cam_pos)

    except StopIteration:
        return accum


class Event(object):
    __slots__ = ('x', 'y', 'polarity', 'ts')
    def __init__(self, x, y, polarity, ts):
        self.x = x
        self.y = y
        self.polarity = polarity
        self.ts = ts


def read_events_cached(bag):

    hashed_filename = '/tmp/{}.bag_events'.format(hashlib.md5(bag.filename).hexdigest())

    # check if there is a cached version
    try:
        with open(hashed_filename, 'rb') as f:
            print "loading events from cache... "
            return pickle.load(f)

    except IOError:
        print "no cached data found, reading from bag"

    # otherwise, read from bag and cache
    # filter and buffer events
    event_buffer = []
    for topic, msg, t in bag.read_messages(topics=['/dvs/events']):

        print_progress('reading events', t.to_sec(), bag.get_start_time(), bag.get_end_time())

        #event_buffer.extend(msg.events)

        for event in msg.events:
            if VERT_POS_DVS-10 < event.y < VERT_POS_DVS+10: # we're only interested in a single row
            #if event.y == VERT_POS_DVS:
                event_buffer.append(Event(event.x, event.y, event.polarity, event.ts))

    with open(hashed_filename, 'wb') as f:
        pickle.dump(event_buffer, f, -1)

    return event_buffer



def track_line_by_events(initial_pos_estimate, events, window_size=20, max_dist=10):
    assert len(events) > 100

    local_buffer = []

    pos_t = []
    pos_x = []

    last_pos = initial_pos_estimate

    for event in events:
        if abs(last_pos - event.x) < max_dist: # close enough?
            local_buffer.append(event)

            if len(local_buffer) < window_size:
                # not enough events, collect some more
                continue

            last_pos = np.average([e.x for e in local_buffer])
            pos_t.append(np.average([e.ts.to_sec() for e in local_buffer]))
            pos_x.append(last_pos)

            del local_buffer[0] # throw out old events

    assert len(local_buffer) == window_size-1

    return pos_t, pos_x


def normalize(arr):
    arr -= np.min(arr)
    arr /= np.max(arr)
    return arr


if __name__ == '__main__':
    #bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/vert_line_2017-11-20-15-15-39.bag')
    #bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/synced_vert_line_2017-11-22-15-54-48.bag')
    #bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/linear_slider/simple_test_2017-12-05-17-52-02.bag')
    #bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/linear_slider/simple_test2_2017-12-05-18-15-09.bag')

    bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/linear_slider/nosync_2017-12-09-16-20-31.bag')

    bridge = CvBridge()

    #fig, ax = plt.subplots()


    accel_t = []
    accel_x = []
    accel_y = []
    accel_z = []
    for topic, msg, ts in bag.read_messages(topics=['/dvs/imu']):
        print_progress("reading IMU data", ts.to_sec(), bag.get_start_time(), bag.get_end_time())
        accel_t.append(msg.header.stamp.to_sec() - bag.get_start_time())
        accel_x.append(msg.linear_acceleration.x)
        accel_y.append(msg.linear_acceleration.y)
        accel_z.append(msg.linear_acceleration.z)

    #plt.plot(t, y)
    #plt.plot(t, 
    accel_y = smooth(np.array(accel_y), window_len=51)




    positions_x = []
    positions_t = []
    last_pos = None

    for topic, msg, t in bag.read_messages(topics=['/camera/image_raw']):
        cv_image = bridge.imgmsg_to_cv2(msg) #, desired_encoding="passthrough")


        #plt.plot( cv_image[VERT_POS, :, 0] )
        #plt.plot( smooth(cv_image[VERT_POS, :, 0], 20) )

        #pos = find_edge( smooth(cv_image[VERT_POS, MIN_HORIZ_CAM:MAX_HORIZ_CAM, 0]) )
        pos = find_edge( cv_image[VERT_POS, MIN_HORIZ_CAM:MAX_HORIZ_CAM, 0] )

        if last_pos is not None and pos is not None and abs(last_pos - pos) > 60:
            continue # ignore sudden jumps


        positions_x.append(pos)
        positions_t.append(msg.header.stamp.to_sec() - bag.get_start_time())
        last_pos = pos

        print_progress('processing images', t.to_sec(), bag.get_start_time(), bag.get_end_time())

        #break



    #new_t = np.linspace(positions_t[0], positions_t[-1],10000)
    #interp = interp1d(positions_t, positions_x, kind='quadratic')

    p1 = plt.figure().add_subplot(111)

    #p1.plot(new_t, interp(new_t), '-r')
    #p1.plot(positions_t, smooth(np.array(positions_x)), '-g')
    p1.plot(positions_t, positions_x, '.b')

    #p1.legend(['smoothed', 'raw measurements'])

    p1.set_title('camera movmement')
    p1.set_xlabel('time [s]')
    p1.set_ylabel('horizontal position [px]')

    #plt.show()


    positions_dvs_x = []
    positions_dvs_t = []
    positions_hist = np.zeros( (DVS_WIDTH, 1) )

    window_start_t = None

    # get the timestamp of the very first event
    for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
        window_start_t = msg.events[0].ts
        break

    event_buffer = read_events_cached(bag)
    all_events = list(event_buffer) # make copy


    last_pos = None
    while window_start_t < rospy.Time.from_sec(bag.get_end_time()):

        print_progress('processing events', window_start_t.to_sec(), bag.get_start_time(), bag.get_end_time())

        # go to next time window
        window_start_t += EVENT_HIST_WINDOW_INC
        window_end_t = window_start_t + EVENT_HIST_WINDOW_SIZE

        # find oldest event that is still in window
        for idx, event in enumerate(event_buffer):
            if event.ts >= window_start_t:
                break

        if len(event_buffer) < 3:
            #positions_hist = np.concatenate((positions_hist, np.zeros((DVS_WIDTH,1))), axis=1)
            continue

        # delete old events
        del event_buffer[:idx]

        if len(event_buffer) < 3:
            #positions_hist = np.concatenate((positions_hist, np.zeros((DVS_WIDTH,1))), axis=1)
            continue

        # find newest oldest event after window (i.e. the first one not in the window anymore)
        for idx, event in enumerate(event_buffer):
            if event.ts > window_end_t:
                break

        if idx <= 3:
            #positions_hist = np.concatenate((positions_hist, np.zeros((DVS_WIDTH,1))), axis=1)
            continue


        # calculate histogram of current window
        dvs_pixels = np.zeros( (DVS_WIDTH, 1) ) # width of DAVIS sensor

        for event in event_buffer[:idx]:
            dvs_pixels[event.x] += 1

        # smooth histogram
        dvs_pixels[:,0] = smooth(dvs_pixels[:,0])


        # find peak that is not too far away from previous one
        # TODO: save tuple with current time as well
        pos = np.argmax(dvs_pixels)

        if dvs_pixels[pos] < 4:
            #positions_hist = np.concatenate((positions_hist, np.zeros((DVS_WIDTH,1))), axis=1)
            continue # ignore lines with very low peak

        peak_cnt = 4
        while last_pos and abs(pos-last_pos) > 25:
            dvs_pixels[pos] = 0
            pos = np.argmax(dvs_pixels)
            peak_cnt -= 1
            if peak_cnt < 0:
                break

        if peak_cnt < 0:
            #positions_hist = np.concatenate((positions_hist, np.zeros((DVS_WIDTH,1))), axis=1)
            continue

        # try to get sub-pixel accuracy by using weighted average over neighbouring pixels
        neighborhood = np.array(dvs_pixels[pos-3:pos+4])
        pos = float(np.linspace(pos-3,pos+3,7).dot(neighborhood / sum(neighborhood)))

        last_pos = pos

        positions_dvs_x.append(pos)
        positions_dvs_t.append( (window_start_t + EVENT_HIST_WINDOW_INC/2).to_sec() - bag.get_start_time() )

        # normalize hist
        dvs_pixels /= float(np.max(dvs_pixels))

        positions_hist = np.concatenate((positions_hist, dvs_pixels), axis=1)



    # initialize using one of the first positions calculated using normal method
    pos_dvs2_t, pos_dvs2_x = track_line_by_events(positions_dvs_x[2], all_events)
    # use relative time
    pos_dvs2_t = [t - bag.get_start_time() for t in pos_dvs2_t]


    p2 = plt.figure().add_subplot(111)
    #plt.plot(positions_dvs)
    p2.imshow(positions_hist, cmap='hot', interpolation='nearest')
    p2.set_title('DVS raw movement')
    p2.set_xlabel('time [nonlinear, {}s/px]'.format(EVENT_HIST_WINDOW_INC.to_sec()))
    p2.set_ylabel('horizontal position [px]')
    #plt.colorbar()

    p3 = plt.figure().add_subplot(111)

    interp = interp1d(positions_dvs_t, positions_dvs_x, kind='quadratic')
    interp_t = np.linspace(positions_dvs_t[0], positions_dvs_t[-1], len(positions_dvs_t)*10)
    p3.plot(interp_t, interp(interp_t), '-')

    pos_dvs2_x_smooth = smooth(np.array(pos_dvs2_x), window_len=41)

    p3.plot(positions_dvs_t, positions_dvs_x, '.b')
    p3.plot(pos_dvs2_t, pos_dvs2_x, '.r')
    p3.plot(pos_dvs2_t, pos_dvs2_x_smooth, '-r')
    p3.set_title('DVS movement')
    p3.set_xlabel('time [s]')
    p3.set_ylabel('horizontal position [px]')

    positions_x     = np.array(positions_x, dtype=float)
    positions_dvs_x = np.array(positions_dvs_x, dtype=float)

    # normalize positions
    positions_x       -= np.min(positions_x)
    positions_dvs_x   -= np.min(positions_dvs_x)
    pos_dvs2_x_smooth -= np.min(pos_dvs2_x_smooth)

    positions_x       /= np.max(positions_x)
    positions_dvs_x   /= np.max(positions_dvs_x)
    pos_dvs2_x_smooth /= np.max(pos_dvs2_x_smooth)

    #smooth_dvs_x = smooth(positions_x)[5:-5]

    #positions_t = [t-0.014 for t in positions_t] # estimated offset

    p4 = plt.figure().add_subplot(111)
    p4.plot(positions_t, positions_x, marker='.')
    p4.plot(positions_dvs_t, positions_dvs_x, '.')
    p4.plot(pos_dvs2_t, pos_dvs2_x_smooth, '.')
    p4.set_title('unaligned camera trajectories')
    p4.set_xlabel('time [s]')
    p4.set_ylabel('horizontal position [normalized from px]')
    p4.legend(['camera frames', 'events: fixed window integration', 'events: continuous line tracking'])
    #p4.plot(positions_dvs_t, smooth_dvs_x)

    #fig.tight_layout()

    t_offsets = np.linspace(-0.02, 0.01, 1000)
    conv = []

    for t_offset in t_offsets:
        print_progress("correlating position", t_offset, t_offsets[0], t_offsets[-1])
        conv.append( correlate(zip(positions_t, positions_x), zip(pos_dvs2_t, pos_dvs2_x_smooth), t_offset) )

    p5 = plt.figure().add_subplot(111)
    p5.plot(t_offsets*1000, normalize(conv))
    p5.set_title('correlation')
    p5.set_xlabel('time offset [ms]')
    p5.set_ylabel('error [normalized]')


    
    camera_accel = normalize(np.diff(smooth(np.diff(positions_x))))
    accel_y = 1-normalize(accel_y) # IMU appearantly looks in the other direction ;)

    p = plt.figure().add_subplot(111)
    p.plot(positions_t[1:], normalize(np.diff(positions_x)))
    p.plot(positions_t[1:-1], camera_accel)
    p.plot(accel_t, accel_y)
    p.set_xlabel('time [s]')
    p.set_ylabel('acceleration / velocity [normalized]')

    p.legend(['camera velocity', 'camera acceleration', 'measured acceleration'])



    conv = []
    for t_offset in t_offsets:
        print_progress("correlating acceleration", t_offset, t_offsets[0], t_offsets[-1])
        conv.append( correlate(zip(positions_t[1:-1], camera_accel), zip(accel_t, accel_y), t_offset) )

    p5.plot(t_offsets*1000, normalize(conv))
    p5.legend(['position', 'acceleration'])

    bag.close()
    plt.show()
