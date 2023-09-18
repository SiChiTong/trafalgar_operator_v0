#!/usr/bin/env python3
########################################################################
# Filename    : __gstPlayer.py
# Description : basic video reader via gstreamer
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################

import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from time import sleep

MAX_QUEUE_SIZE = 2

class GstPlayer( object ):

    def __init__( self,  output_queue, input_queue, videoPlayer = False, framerate = 1/24, loopVideo = False ):

        super().__init__()
        
        self.output_queue = output_queue
        self.input_queue = input_queue

        self._pipeline = None
        self._bus = None

        self.frame_buffer = None
        self.buffer_datas = None

        self.isPaused = True
        
        self.isAVideoPlayer = videoPlayer
        self._active_track = ""
        self.last_frame_timestamp = 0
        self._framerate = framerate
        self._loopVideo = loopVideo

    def start( self ):

        Gst.init(None)

        if self.isAVideoPlayer is True:
            return
        
        self.pipeline_launch()


    def get_pipeline_desc( self ):

        pipeline = "" 
                
        if self.isAVideoPlayer is True :
            
            if self._active_track != "" :

                pipeline = (

                    f"filesrc name=videoplayersrc location={self._active_track} ! "
                    "decodebin ! "
                    "videoconvert ! "
                    "video/x-raw, format=(string)RGB ! "
                    "videoscale ! "
                    "identity drop-allocation=true ! "
                    "appsink name=videoplayersink emit-signals=true max-buffers=1 drop=true sync=false async=false"
    
                )

        else:

            pipeline = (

                "udpsrc name=videostreamsrc port=3000 retrieve-sender-address=false ! "
                "application/x-rtp, media=video, encoding-name=H264, payload=96 ! "
                "rtph264depay ! "
                "h264parse ! "
                "avdec_h264 ! "
                "videoconvert ! "
                "video/x-raw, format=(string)RGB ! "
                "videoscale ! "
                "identity drop-allocation=true ! "
                "appsink name=videostreamsink emit-signals=true max-buffers=1 drop=true sync=false async=false"
        
            ) 


        return pipeline


    def OnNewSample(self, sink):
        
        sample = sink.emit("pull-sample")
        frame_buffer = sample.get_buffer()
        
        caps = sample.get_caps()

        frame_width = caps.get_structure(0).get_value("width")
        frame_height = caps.get_structure(0).get_value("height")

        _, buffer_datas = frame_buffer.map(Gst.MapFlags.READ)
        
        timestamp = frame_buffer.pts
        
        frame = np.frombuffer( buffer_datas.data, dtype=np.uint8 )
        frame = frame.reshape((frame_height, frame_width,3))

        frame_buffer.unmap( buffer_datas )

        if self.isAVideoPlayer is True and timestamp is not None:

            if self.last_frame_timestamp != 0:
                
                time_diff = timestamp - self.last_frame_timestamp
                wait_time = max(0, (1 / self._framerate) - (time_diff / Gst.SECOND))  # Divisez par la fréquence de l'horloge
 
                sleep( 1 / wait_time )

            self.last_frame_timestamp = timestamp
        

        frame_data = {
            "frame": frame, 
            "size": (frame_width, frame_height)
        }

        if self.output_queue.qsize() >= MAX_QUEUE_SIZE:
            self.output_queue.get() 
        
        self.output_queue.put(frame_data)

        return Gst.FlowReturn.OK


    def pipeline_launch( self ):

        pipeline_string = self.get_pipeline_desc()

        if pipeline_string == "":
            return

        try:

            self._pipeline = Gst.parse_launch( pipeline_string )
            self._bus = self._pipeline.get_bus()

            sinkname = "videoplayersink" if self.isAVideoPlayer is True else "videostreamsink"
            appsink = self._pipeline.get_by_name( sinkname )
            appsink.connect("new-sample", self.OnNewSample)

            srcname = "videoplayersrc" if self.isAVideoPlayer is True else "videostreamsrc"
            self._framesrc = self._pipeline.get_by_name( srcname )

            self._pipeline.set_state( Gst.State.PLAYING )

            self.isPaused = False
            self.EOS = False

        except Exception as e:
            print(f"pipeline launch error : {e}")
        

    def play_track(self, track=""):
        
        if not track:
            return

        if self._pipeline is None:

            self._active_track = track
            self.pipeline_launch()
 
        else:

            if track != self._active_track :

                self.last_frame_timestamp = 0
                self._active_track = track

                self.quit()
                self.pipeline_launch()


    def read_bus( self ): 

        if self._bus is not None:
            
            message = self._bus.timed_pop(0.1 * Gst.SECOND)
            
            if message == None:
                return

            elif message.type == Gst.MessageType.EOS:

                if self._loopVideo is True:

                    self._pipeline.seek_simple(Gst.Format.TIME, Gst.SeekFlags.FLUSH, 0)
                    self.EOS = False

                else:
                    
                    if self.EOS is False:
                        self.EOS = True
                        self.quit()


    def pause( self, pauseState = True ):

        if self.isAVideoPlayer is True and self._loopVideo is False:
            return 
        
        if self._pipeline is not None:

            if pauseState is False: 

                if self.isPaused is True:
                    self.isPaused = False
                    self._pipeline.set_state( Gst.State.PLAYING )

            else:

                if self.isPaused is False:
                    self.isPaused = True
                    self._pipeline.set_state(Gst.State.PAUSED)


    def loop(self):
        
        if self.isAVideoPlayer is True and self._pipeline is not None:
            self.read_bus()

        if self.input_queue.empty():
            return 
        
        command = self.input_queue.get() 
        
        if "release" in command.keys():

            release = command["release"]

            if release is True:
                self.quit()
                return
        
        if "pause" in command.keys():

            playState = command["pause"]
            self.pause( playState )

        if "videotrack" in command.keys():

            trackToPlay = command["videotrack"]
            
            if trackToPlay != "":
                self.play_track( trackToPlay )

  


    def quit( self ):

        if self._pipeline is not None: 

            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None

            self._bus = None
            self.isPaused = True
