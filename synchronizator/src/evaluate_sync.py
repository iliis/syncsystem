#!/usr/bin/env python

import itertools
import datetime
import math
import os
import pickle
import hashlib
import csv
from collections import OrderedDict

import rospy
import rosbag
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import scipy as sp

from scipy.interpolate import interp1d



VERT_POS = 255
VERT_POS_DVS = 160

MIN_HORIZ_CAM = 260
MAX_HORIZ_CAM = 520

MIN_HORIZ_DVS = 36
MAX_HORIZ_DVS = 320

DVS_WIDTH = 350 # 240 for DAVIS

# adjust those so the DVS plot looks good ;)
EVENT_HIST_WINDOW_SIZE = rospy.Duration.from_sec(0.01)
EVENT_HIST_WINDOW_INC  = rospy.Duration.from_sec(0.002)

PLOT_EXPORT_DIR = '/home/samuel/sync/ETH/Master/Semester 3/syncsystem/plots/long_recording/multiplication/'

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

def prevcurnext(l):
    it = iter(l)
    try:
        last_obj = it.next()
        cur_obj  = it.next()
        while True:
            next_obj = it.next()
            yield last_obj, cur_obj, next_obj
            last_obj = cur_obj
            cur_obj = next_obj
    except StopIteration:
        pass


# guess f(x) by interpolating between y1=f(x1) and y2=f(x2)
def interpolate(x1, y1, x2, y2, x):
    assert x1 <= x <= x2

    return (x-x1)/(x2-x1) * (y2-y1) + y1


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



def load_from_csv(bag, name):
    filename = '{}_{}.csv'.format(bag.filename, name)

    try:
        data = []
        with open(filename, 'rb') as f:
            print "loading", name, "from", filename
            datareader = csv.reader(f, delimiter=',')
            for row in datareader:
                if len(data) == 0:
                    for _ in range(len(row)):
                        data.append([])

                for c, col in enumerate(row):
                    data[c].append(float(col))

            return data
    except IOError:
        print "no cached CSV data found for", name, "reading from bag"
        return None

def save_to_csv(bag, name, data):
    filename = '{}_{}.csv'.format(bag.filename, name)

    i = 0
    with open(filename, 'wb') as f:
        datawriter = csv.writer(f, delimiter=',')
        for row in data:
            i+=1
            datawriter.writerow(list(row))

    assert i > 5 # otherwise something probably went wrong ;)
    print "wrote", i, "lines to", filename


def cache_in_csv(func):
    def wrapper(bag, topic, *args, **kwargs):
        # try to load cached data from disk
        data = load_from_csv(bag, topic.replace('/', '_'))
        if data is not None:
            return data

        # welp, we didn't get it. Let's compute it manually!
        data = func(bag, topic, *args, **kwargs)

        # and store it, so we can retrieve it next time
        save_to_csv(bag, topic.replace('/', '_'), zip(*data))

        return data
    return wrapper



def find_transition(data, thresh):
    for pos, (prev, cur) in enumerate(prevnext(data)):
        if prev < thresh and cur >= thresh:
            return interpolate(prev, pos, cur, pos+1, thresh)
    return None

# TODO: use dynamic programming to optimize this
def find_edge_by_clustering(data):
    data = np.array(data, dtype='float')
    best_cost = float('inf')
    best_pos  = None
    for pos in range(len(data)):
        cost = sum(abs(data[:pos]-np.average(data[:pos]))) + sum(abs(data[pos:]-np.average(data[pos:])))
        if cost < best_cost:
            best_cost = cost
            best_pos = pos

    return find_transition(data, (np.average(data[best_pos:])*2+np.average(data[:best_pos]))/3), best_pos




def find_edge(data):

    #diff = abs(data[1:] - data[:-1])

    # actually, there is no need for abs(), as long as the darker side is to the left ;)
    diff = np.ediff1d(np.array(data, dtype='float'))

    pos = np.argmax(diff)

    if pos < 3 or pos > len(diff)-4:
        return None

    # improve accuracy by using weighted average over neighbouring pixels
    neighborhood = diff[pos-3:pos+4]

    s = float(sum(neighborhood))
    if s == 0:
        return None

    return np.linspace(pos-3, pos+3, 7).dot(neighborhood / s)




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

            pos_t.append(np.average([e.ts.to_sec()-bag.get_start_time() for e in local_buffer]))

            last_pos = np.average([e.x for e in local_buffer])
            pos_x.append(last_pos)

            del local_buffer[0] # throw out old events

    assert len(local_buffer) == window_size-1

    return pos_t, pos_x


def normalize(arr):
    arr -= np.min(arr)
    arr /= np.max(arr)
    return arr

def normalize_avg(arr):
    arr -= np.average(arr)
    arr /= np.std(arr)
    return arr

@cache_in_csv
def read_acceleration(bag, topic = '/dvs/imu'):
    accel_t = []
    accel_x = []
    accel_y = []
    accel_z = []
    for topic, msg, ts in bag.read_messages(topics=[topic]):
        print_progress("reading IMU data", ts.to_sec(), bag.get_start_time(), bag.get_end_time())
        accel_t.append(msg.header.stamp.to_sec() - bag.get_start_time())
        #accel_x.append(msg.linear_acceleration.x)

        # IMU appearantly looks in the other direction ;)
        accel_y.append(-msg.linear_acceleration.y)

        #accel_z.append(msg.linear_acceleration.z)

    accel_y = smooth(np.array(accel_y), window_len=51)

    return accel_t, accel_y

@cache_in_csv
def read_image_positions(bag, topic, vert_pos, horiz_range):

    positions_x = []
    positions_t = []
    last_pos = None

    for topic, msg, t in bag.read_messages(topics=[topic]):
        cv_image = np.array(bridge.imgmsg_to_cv2(msg), dtype=float) #, desired_encoding="passthrough")

        if cv_image.ndim == 3:
            assert cv_image.shape[2] == 3
            cv_image = np.sum(cv_image, axis=2) # sum over colors

        # cut out the interesting patch of the camera image
        # sum it vertically to reduce noise
        line = np.sum( cv_image[vert_pos-10:vert_pos+10, horiz_range[0]:horiz_range[1]], axis=0 )

        #pos  = find_edge(line)
        pos, cl_pos = find_edge_by_clustering(line)

        if False and t.to_sec() - bag.get_start_time() > 47:
            diff = np.ediff1d(line)
            fig = plt.figure()
            p = fig.add_subplot(121)
            p.plot(diff)
            p.plot(line)
            p.plot(smooth(line, window_len=21))
            p.plot(np.ediff1d(smooth(line, window_len=21)))
            if pos is not None:
                p.axvline(pos, color='red')
                a1 = np.average(line[cl_pos:])
                a2 = np.average(line[:cl_pos])
                p.axhline(a1)
                p.axhline(a2)

            pos2 = find_edge(line)
            if pos2 is not None:
                p.axvline(pos2, color='blue')
            p.axhline((a1*2+a2)/3)
            p.legend(['diff', 'data', 'smoothed data', 'smoothed diff'])

            p2 = fig.add_subplot(122)
            p2.imshow(cv_image, interpolation='nearest')
            p2.set_title('DVS image')
            p2.axvline(horiz_range[0], color='red')
            p2.axvline(horiz_range[1], color='red')
            p2.axhline(vert_pos-10, color='red')
            p2.axhline(vert_pos+10, color='red')

            plt.show()

        # ignore sudden jumps
        if pos is None or (last_pos is not None and pos is not None and abs(last_pos - pos) > 60):
            continue

        positions_x.append(pos)
        positions_t.append(msg.header.stamp.to_sec() - bag.get_start_time())
        last_pos = pos

        print_progress('processing {}'.format(topic), t.to_sec(), bag.get_start_time(), bag.get_end_time())
        #print "processing image at t =", t.to_sec() - bag.get_start_time()

    return np.array(positions_t), np.array(positions_x)


def read_slider_positions(bag, topic='/linear_slider_ros_interface/pose'):
    times = []
    pos = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        #print_progress('reading slider positions', t.to_sec(), bag.get_start_time(), bag.get_end_time())
        #print msg.pose.position.x
        times.append(msg.header.stamp.to_sec()-bag.get_start_time())
        pos.append(msg.pose.position.x)

    return times, pos

@cache_in_csv
def read_dvs_position_by_integration(bag, topic): # topic is not used in here, but cache_in_csv requires it
    events = read_events_cached(bag)

    positions_dvs_x = []
    positions_dvs_t = []
    positions_hist = np.zeros( (DVS_WIDTH, 1) )

    # get the timestamp of the very first event
    window_start_t = events[0].ts

    events = list(events) # make copy, as we're going to edit this list

    last_pos = None
    while window_start_t < rospy.Time.from_sec(bag.get_end_time()):

        print_progress('processing events', window_start_t.to_sec(), bag.get_start_time(), bag.get_end_time())

        # go to next time window
        window_start_t += EVENT_HIST_WINDOW_INC
        window_end_t = window_start_t + EVENT_HIST_WINDOW_SIZE

        # find oldest event that is still in window
        for idx, event in enumerate(events):
            if event.ts >= window_start_t:
                break

        if len(events) < 3:
            #positions_hist = np.concatenate((positions_hist, np.zeros((DVS_WIDTH,1))), axis=1)
            continue

        # delete old events
        del events[:idx]

        if len(events) < 3:
            #positions_hist = np.concatenate((positions_hist, np.zeros((DVS_WIDTH,1))), axis=1)
            continue

        # find newest oldest event after window (i.e. the first one not in the window anymore)
        for idx, event in enumerate(events):
            if event.ts > window_end_t:
                break

        if idx <= 3:
            #positions_hist = np.concatenate((positions_hist, np.zeros((DVS_WIDTH,1))), axis=1)
            continue


        # calculate histogram of current window
        dvs_pixels = np.zeros( (DVS_WIDTH, 1) ) # width of DAVIS sensor

        for event in events[:idx]:
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

    return positions_dvs_t, positions_dvs_x

def clean_event_positions(times, positions, max_deltaT=0.05):
    i = 1
    while i < len(positions):
        if i < len(positions)-1:
            if times[i]-times[i-1] >= max_deltaT and times[i+1]-times[i] >= max_deltaT:
                del positions[i]
                del times[i]
                i -= 1
        i+=1


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
                if cam_t+t_offset < dvs_next_t and dvs_prev_pos is not None and dvs_next_pos is not None:
                    break

            if cam_t+t_offset < dvs_prev_t:
                continue # ignore frames that aren't between two dvs position estimates

            if dvs_next_t - dvs_prev_t >= 0.05:
                continue # ignore frames that don't have a close position estimate from dvs

            if dvs_prev_pos is None or dvs_next_pos is None or cam_pos is None:
                continue

            accum += abs(interpolate(dvs_prev_t, dvs_prev_pos, dvs_next_t, dvs_next_pos, cam_t+t_offset) - cam_pos)

    except StopIteration:
        return accum


def correlate2(query_t, query_x, ref_t, ref_x, t_offset):

    query_t = [t+t_offset for t in query_t]

    # cut query points so that we're not going outside reference values
    query_t = query_t[10:-10]
    query_x = query_x[10:-10]

    while query_t[0] <= ref_t[10]:
        query_t = query_t[10:]
        query_x = query_x[10:]

    while query_t[-1] >= ref_t[-10]:
        query_t = query_t[:-10]
        query_x = query_x[:-10]

    # evaluate 'reference' at query positions
    interp_ref_f = interp1d(ref_t, ref_x, copy=False, assume_sorted=True, kind='quadratic')
    try:
        interp_ref_x = interp_ref_f(query_t)
    except ValueError:
        print ref_t[0], ref_t[-1]
        print query_t[0], query_t[-1]

    if False:
        p = plt.figure().add_subplot(111)
        p.plot(query_t, query_x, '.')
        p.plot(ref_t, ref_x)
        p.plot(query_t, interp_ref_x, '.')
        p.legend(['query', 'ref', 'inter'])
        p.set_title("dot = {}".format(np.dot(query_x, interp_ref_x)))
        plt.show()

    #return np.sum(abs(query_x - interp_ref_x))
    return -np.dot(query_x, interp_ref_x)


# positions is a dict: name => [(t, x)]
def correlate_all(positions, max_offset=0.04):

    t_offsets = np.linspace(-max_offset, max_offset, 1000)

    with open(os.path.join(PLOT_EXPORT_DIR, 'minima.csv'), 'wb') as filehandle:
        csv_export = csv.writer(filehandle, delimiter=',')
        csv_export.writerow(['against'] + list(positions.keys()))


        for i, (base_name, (base_t, base_x)) in enumerate(positions.iteritems()):
            f = plt.figure()
            p = f.add_subplot(111)

            base_x = normalize_avg(base_x)

            names = []
            minima = []
            for sub_name, (data_t, data_x) in positions.iteritems():
                #if sub_name == base_name:
                    #continue
                data_x = normalize_avg(data_x)

                if base_name == 'acceleration':
                    if sub_name != 'acceleration':
                        # calculate second derivative
                        data_x = normalize_avg(np.diff(smooth(np.diff(data_x))))
                        data_t = data_t[1:-1] # derivation cuts of first and last sample
                elif sub_name == 'acceleration':
                    minima.append('n/a')
                    continue


                conv = []
                for t_offset in t_offsets:
                    print_progress("correlating {} against {}".format(sub_name, base_name), t_offset, t_offsets[0], t_offsets[-1])
                    #conv.append( correlate(zip(data_t, data_x), zip(base_t, base_x), t_offset) )
                    conv.append( correlate2(data_t, data_x, base_t, base_x, t_offset) )

                p.plot(t_offsets*1000, normalize(conv))
                cur_min = t_offsets[np.argmin(conv)]*1000
                print "min at offset = ", cur_min, "ms"
                #print "max at offset = ", t_offsets[np.argmax(conv)]*1000, "ms"
                names.append(sub_name)
                minima.append(cur_min)

            csv_export.writerow([base_name] + minima)


            p.set_title('correlation against {}'.format(base_name))
            p.set_xlabel('time offset [ms]')
            p.set_ylabel('error [normalized]')
            p.legend(names)
            f.savefig(PLOT_EXPORT_DIR + 'correlation_against_{}.png'.format(base_name), bbox_inches='tight')
            plt.show(block=False)

    # plot all positions in one plot
    f = plt.figure()
    p = f.add_subplot(111)
    p.set_title('position')
    names = []
    for name, (data_t, data_x) in positions.iteritems():
        p.plot(data_t, normalize_avg(data_x), '.-')
        names.append(name)

    p.legend(names)
    p.set_xlabel('time [s]')
    p.set_ylabel('position [normalized]')
    f.savefig(PLOT_EXPORT_DIR + 'all_positions.png', bbox_inches='tight')

def correlate_against_acceleration(positions, accel_t, accel_x, max_offset=0.04):
    t_offsets = np.linspace(-max_offset, max_offset, 200)

    accel_x = normalize_avg(accel_x)

    f = plt.figure()
    p = f.add_subplot(111)
    p.set_title('correlation against acceleration')
    p.set_xlabel('time offset [ms]')
    p.set_ylabel('error [normalized]')
    names = []
    for sub_name, (data_t, data_x) in positions.iteritems():
        names.append(sub_name)

        # calculate second derivative
        dd_data_x = normalize_avg(np.diff(smooth(np.diff(data_x))))

        conv = []
        for t_offset in t_offsets:
            print_progress("correlating {} against acceleration".format(sub_name), t_offset, t_offsets[0], t_offsets[-1])
            #conv.append( correlate2(zip(data_t[1:-1], dd_data_x), zip(accel_t, accel_x), t_offset) )
            conv.append( correlate2(data_t[1:-1], dd_data_x, accel_t, accel_x, t_offset) )

        p.plot(t_offsets*1000, normalize(conv))

        print "min at offset = ", t_offsets[np.argmin(conv)]*1000, "ms"
        #print "max at offset = ", t_offsets[np.argmax(conv)]*1000, "ms"

    p.legend(names)
    f.savefig(PLOT_EXPORT_DIR + 'correlation_against_acceleration', bbox_inches='tight')




if __name__ == '__main__':

    t_start = datetime.datetime.now()

    #bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/vert_line_2017-11-20-15-15-39.bag')
    #bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/synced_vert_line_2017-11-22-15-54-48.bag')
    #bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/linear_slider/simple_test_2017-12-05-17-52-02.bag')
    #bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/linear_slider/simple_test2_2017-12-05-18-15-09.bag')

    #bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/linear_slider/nosync_2017-12-09-16-20-31.bag')
    bag = rosbag.Bag('/home/samuel/rpg_syncsystem_local/bagfiles/linear_slider/linear_slider_sam.bag')

    bridge = CvBridge()

    #fig, ax = plt.subplots()



    positions_dvs_t, positions_dvs_x = read_dvs_position_by_integration(bag, 'dvs_position_by_integration')

    pos_dvs = load_from_csv(bag, 'event_pos')
    if pos_dvs is None:


        # initialize using one of the first positions calculated using normal method
        pos_dvs2_t, pos_dvs2_x = track_line_by_events(positions_dvs_x[2], events)

        # remove positions that are likely to be bogus (i.e. no estimate before or after 50ms)
        clean_event_positions(pos_dvs2_t, pos_dvs2_x)

        save_to_csv(bag, 'event_pos', zip(pos_dvs2_t, pos_dvs2_x))
    else:
        pos_dvs2_t = pos_dvs[0]
        pos_dvs2_x = pos_dvs[1]

    
    positions = OrderedDict([
        ("linear slider",                   read_slider_positions(bag, '/linear_slider_ros_interface/pose')),
        ("BlueFox camera",                  read_image_positions(bag, '/camera/image_raw',              VERT_POS,       (MIN_HORIZ_CAM, MAX_HORIZ_CAM))),
        ("BlueFox camera (synchronized)",   read_image_positions(bag, '/synchronized/camera/image_raw', VERT_POS,       (MIN_HORIZ_CAM, MAX_HORIZ_CAM))),
        ("DAVIS camera",                    read_image_positions(bag, '/dvs/image_raw',                 VERT_POS_DVS,   (MIN_HORIZ_DVS, MAX_HORIZ_DVS))),
        ("DAVIS events (integrated)",       (positions_dvs_t, positions_dvs_x)),
        ("DAVIS events (tracker, smoothed)", (pos_dvs2_t, smooth(np.array(pos_dvs2_x), window_len=101))),
        ("acceleration",                    read_acceleration(bag, '/dvs/imu')),
        #("DAVIS events",                    (pos_dvs2_t, pos_dvs2_x)),
    ])


    #accel_t, accel_y = read_acceleration(bag)
    #correlate_against_acceleration(positions, accel_t, accel_y)
    #plt.show(block=False)

    correlate_all(positions)

    bag.close()

    total_t = datetime.datetime.now() - t_start
    print "analysis took", total_t

    plt.show()
