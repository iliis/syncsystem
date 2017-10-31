#!/usr/bin/env python

import inspect
import rospy
from sensor_msgs.msg import Image

from dvs_msgs.msg import SpecialEvent, Event, EventArray
from bluefox_ros.msg import CaptureInfo

from sfml import sf

import time
import threading

W = 2000
H = 500



LED_DELAY   = rospy.Duration.from_sec(0.001)
LED_ON_TIME = rospy.Duration.from_sec(0.004)

HIST_DURATION = rospy.Duration.from_sec(0.2)




font = sf.Font.from_file("FreeSans.ttf")


def ringbuffer(ring, write_idx):
    idx = write_idx + 1
    while idx != write_idx:
        if idx >= len(ring):
            idx = 0

        yield ring[idx]

        idx += 1


class TimeHistogram:
    def __init__(self, width): #, bin_count):
        self.width     = width # duration!
        self.bin_count = W #bin_count

        self.in_callback = False

        self.mutex = threading.Lock()

        # derived parameters
        self.bin_width = width/self.bin_count # duration

        # initialize bins
        self.bins_pos = [0 for i in range(self.bin_count)]
        self.bins_neg = [0 for i in range(self.bin_count)]
        self.write_idx = 0
        self.latest_bin_t = None # rospy.Time

        self.special_events = []
        self.camera_frames = []

        self.info_text = sf.Text()
        self.info_text.font = font
        self.info_text.string = u"{}uS per pixel".format((self.width/self.bin_count).nsecs/1000)
        self.info_text.position = sf.Vector2(5, H-self.info_text.character_size-5)

    def event_callback(self, data):
        with self.mutex:
            self.in_callback = True
            if self.latest_bin_t is None:
                self.latest_bin_t = data.events[0].ts

            for ev in data.events:
                self.append(ev)
            self.in_callback = False

    def special_event_callback(self, data):
        self.special_events.append(data)

    def camera_frame_callback(self, data):
        #print "got camera frame:", data.header.stamp
        #print "latest timestamp:", self.latest_bin_t
        #print "exposure time:", data.exp_time_us/1000, "ms"
        #print "time difference:", (self.latest_bin_t - data.header.stamp).to_sec(), "sec"
        #print "exposure time:", data.exp_time_us, "=", 1000000/data.exp_time_us, "FPS"
        self.camera_frames.append(data)

    def shift_bins(self):
        self.write_idx += 1
        if self.write_idx >= self.bin_count:
            self.write_idx = 0

        self.bins_neg[self.write_idx] = 0
        self.bins_pos[self.write_idx] = 0

        self.latest_bin_t += self.bin_width

        """
        for i in range(len(self.bins_neg)-1):
            self.bins_neg[i] = self.bins_neg[i+1]
            self.bins_pos[i] = self.bins_pos[i+1]

        self.bins_neg[-1] = 0
        self.bins_pos[-1] = 0
        """

    def append(self, event):
        while not event.ts < self.latest_bin_t + self.bin_width:
            # shift bins
            #del self.bins_neg[0]
            #del self.bins_pos[0]
            #self.bins_neg.append(0)
            #self.bins_pos.append(0)
            self.shift_bins()

        if event.polarity:
            self.bins_pos[self.write_idx] += 1
        else:
            self.bins_neg[self.write_idx] += 1

    def draw(self, target):

        if self.latest_bin_t is None:
            return

        with self.mutex:
            if self.in_callback:
                rospy.logwarn("draw() called while callback still active!")
            latest_bin_t = self.latest_bin_t
            bins_pos = list(self.bins_pos)
            bins_neg = list(self.bins_neg)
            write_idx = self.write_idx


            from_t = latest_bin_t - self.width

            # filter out old events
            self.special_events = [e for e in self.special_events if e.ts+LED_DELAY+LED_ON_TIME >= from_t]
            special_events = list(self.special_events)

            orig_len = len(self.camera_frames)
            self.camera_frames = [e for e in self.camera_frames if e.header.stamp+LED_DELAY+LED_ON_TIME >= from_t]
            camera_frames = list(self.camera_frames)

            if len(camera_frames) == 0: # and orig_len > 0:
                rospy.logwarn("no camera frames in current time window :(")



        width  = target.default_view.size.x
        height = target.default_view.size.y

        halfheight = float(int(height/2))

        rect = sf.RectangleShape()
        # horizontal axis
        arr = sf.VertexArray(sf.PrimitiveType.LINES, 2)
        #arr[0] = sf.Vertex(sf.Vector2(0,     halfheight))
        #arr[1] = sf.Vertex(sf.Vector2(width, halfheight))
        #target.draw(arr)

        # shift everything so rightmost pixel is current time
        tshift = 0 # (rospy.Time.now() - latest_bin_t) / self.width * width
        #print(tshift)


        def width_from_duration(t):
            return t / self.width * width

        def horiz_pos_from_time(t):
            return width_from_duration(t-from_t)- tshift

        # highlight estimated on-window of LED
        if False:
            rect.size = sf.Vector2(width_from_duration(LED_ON_TIME), height)
            rect.fill_color = sf.Color(80,0,0)
            for event in special_events:
                rect.position = sf.Vector2(horiz_pos_from_time(event.ts + LED_DELAY), 0)
                target.draw(rect)

        # highlight estimated exposure window of camera
        rect.fill_color = sf.Color(80,0,0)
        for event in camera_frames:
            rect.size = sf.Vector2(width_from_duration(rospy.Duration(nsecs=1000*event.exp_time_us)), height)
            rect.position = sf.Vector2(horiz_pos_from_time(event.header.stamp) - rect.size.x, 0)
            target.draw(rect)



        for polarity in [True, False]:

            if polarity:
                rect.fill_color = sf.Color(0,100,255)
                bins = bins_pos
            else:
                rect.fill_color = sf.Color(255,0,0)
                bins = bins_neg

            arr[0].color = rect.fill_color
            arr[1].color = rect.fill_color


            for i, cnt in enumerate(ringbuffer(bins, write_idx)):
                if cnt > 0:
                    h = cnt

                    #rect.size = sf.Vector2(width / self.bin_count, h)
                    #rect.position = sf.Vector2(i * rect.size.x, height/2-(rect.size.y+1 if polarity else -1))
                    #target.draw(rect)


                    x = i * width / self.bin_count - tshift
                    if polarity:
                        arr[0].position = sf.Vector2(x, halfheight-1)
                        arr[1].position = sf.Vector2(x, halfheight-1-h)
                    else:
                        arr[0].position = sf.Vector2(x, halfheight)
                        arr[1].position = sf.Vector2(x, halfheight+h)
                    target.draw(arr)

        # draw special events
        rect.size = sf.Vector2(1, height)
        for event in special_events:
            if event.polarity:
                rect.fill_color = sf.Color(0,255,0)
            else:
                rect.fill_color = sf.Color(255,255,0)

            rect.size = sf.Vector2(1, height)
            rect.position = sf.Vector2(horiz_pos_from_time(event.ts), 0)
            target.draw(rect)

            rect.size = sf.Vector2(1, height/2)
            rect.position = sf.Vector2(horiz_pos_from_time(event.header.stamp), 0)
            target.draw(rect)


        # draw camera frames
        rect.fill_color = sf.Color(0,255,255)
        rect.size = sf.Vector2(1, height)
        for event in camera_frames:
            rect.position = sf.Vector2(horiz_pos_from_time(event.header.stamp), 0)
            target.draw(rect)

        target.draw(self.info_text)




if __name__ == '__main__':

    window = sf.RenderWindow(sf.VideoMode(W, H), u"Event Visualizer")
    window.vertical_synchronization = True

    hist = TimeHistogram(HIST_DURATION)

    #for name, val in inspect.getmembers(window):
        #print(name)

    rospy.init_node('event_visualizer', anonymous=True)

    rospy.Subscriber("/dvs/special_events", SpecialEvent, hist.special_event_callback)
    rospy.Subscriber("/dvs/events", EventArray, hist.event_callback)
    rospy.Subscriber("/camera/capture_info", CaptureInfo, hist.camera_frame_callback)
    #rospy.Subscriber("/camera/image_raw", Image, hist.camera_frame_callback)

    rospy.loginfo("listening for messages...")

    paused = False

    while window.is_open and not rospy.is_shutdown():
        for event in window.events:
            if event == sf.Event.CLOSED:
                rospy.signal_shutdown('window closed')
                window.close()
            elif event == sf.Event.KEY_PRESSED:
                if event['code'] == sf.Keyboard.SPACE:
                    paused = not paused
                elif event['code'] == sf.Keyboard.S:
                    window.capture().to_file("screenshot.png")
                elif event['code'] == sf.Keyboard.ESCAPE:
                    window.close()


        if not paused:
            window.clear(sf.Color(0,0,0))

            hist.draw(window)

            window.display()

        #sf.sleep(sf.milliseconds(20))
        time.sleep(0.01)

