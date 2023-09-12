#!/usr/bin/env python3
########################################################################
# Filename    : __videostream.py
# Description : display video via OpenCV
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
#from multiprocessing import Queue
import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

MAX_QUEUE_SIZE = 4

class VideoStream( object ):

    def __init__( self, output_queue, input_queue ):

        super().__init__()

        self.frame_queue = output_queue
        self.command_queue = input_queue

        self._pipeline = None
        self._isHighQualityCodec = False

    @property
    def udpPort( self ): 
        return 3000
        

    def OnNewSample(self, sink):
            
        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()

        caps = sample.get_caps()

        frame_width = caps.get_structure(0).get_value("width")
        frame_height = caps.get_structure(0).get_value("height")

        _, buffer = buf.map(Gst.MapFlags.READ)

        frame = np.frombuffer(buffer.data, dtype=np.uint8)
        frame = frame.reshape((frame_height, frame_width,3))
            
        frame_data = {
            "frame": frame, 
            "size": (frame_width, frame_height)
        }

        if self.frame_queue.qsize() >= MAX_QUEUE_SIZE:
            self.frame_queue.get() 
        
        self.frame_queue.put(frame_data)

        buf.unmap(buffer)

        return Gst.FlowReturn.OK

        
    def h264_decoder( self ):

        pipeline_string = (

            f"udpsrc port={self.udpPort} retrieve-sender-address=false ! "
            "application/x-rtp, media=video, encoding-name=H264, payload=96 ! "
            "rtph264depay ! "
            "h264parse ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "videoscale ! "
            "identity drop-allocation=true ! "
            "appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false async=false" #drop=true
    
        )

        return pipeline_string


    def h265_decoder( self ):

        pipeline_string = (

                f"udpsrc port={self.udpPort} retrieve-sender-address=false ! "
                "application/x-rtp, media=video, encoding-name=H265, payload=96 ! "
                "rtph265depay ! "
                "h265parse ! "
                "avdec_h265 ! "
                "videoconvert ! "
                "video/x-raw, format=BGR ! "
                "videoscale ! "
                "appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false async=false"
    
        )

        return pipeline_string


    def start( self ):

        self.loop = GObject.MainLoop()

        Gst.init(None)

        pipeline_string = self.h265_decoder() if self._isHighQualityCodec is True else self.h264_decoder()#self.h264_decoder()

        self._pipeline = Gst.parse_launch(pipeline_string)

        appsink = self._pipeline.get_by_name("appsink")
        appsink.connect("new-sample", self.OnNewSample)

        self._pipeline.set_state(Gst.State.PLAYING)

        self.loop.idle_add( self.handle_commands )
        self.loop.run()


    def quit( self ):

        if self._pipeline is not None: 

            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None

        if self.loop:
            self.loop.quit()


    def pause( self, enable = True ):

        if self._pipeline is not None:

            if enable is True : 

                if self.isPlaying is False:
                    self.isPlaying = True
                    self._pipeline.set_state( Gst.State.PLAYING )

            else:
                if self.isPlaying is True:
                    self.isPlaying = False
                    self._pipeline.set_state(Gst.State.PAUSED)


    def handle_commands(self):

        try:
            command = self.command_queue.get(timeout=1)  # Check for commands every second
            if command == "start":
                self.pause(False)
            elif command == "stop":
                self.pause(True)
            elif command == "quit":
                self.quit()

        except self.command_queue.Empty:
            pass  # Handle the case when the queue is empty

 

