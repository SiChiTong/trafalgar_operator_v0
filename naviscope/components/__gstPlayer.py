#!/usr/bin/env python3
########################################################################
# Filename    : __videostream.py
# Description : display video via OpenCV
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

    def __init__( self,  output_queue, input_queue, videoTracks = {}, framerate = 1/30 ):

        super().__init__()
        
        self.frame_queue = output_queue
        self.command_queue = input_queue

        self.isPipelinePaused = True

        self._framerate = framerate

        self._active_track = ""
        self._pipeline = None
        
        self._videoTracks = videoTracks
        self._videoPlayer = None

    def get_pipeline( self ):

        pipeline = (

            "udpsrc port=3000 retrieve-sender-address=false ! "
            "application/x-rtp, media=video, encoding-name=H264, payload=96 ! "
            "rtph264depay ! "
            "h264parse ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "videoscale ! "
            "identity drop-allocation=true ! "
            "appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false async=false"
    
        ) 
                
        if len(self._videoTracks) >= 1 :
             
            videotracks = self._videoTracks.values()
            firstVideoTrack = videotracks[0]

            pipeline = (

                f"filesrc name=videoplayer location={firstVideoTrack} ! "
                "avdec_h264 ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! "
                "videoscale ! "
                "identity drop-allocation=true ! "
                "appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false async=false"
    
            )
        
        return pipeline


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


    def start( self ):

        Gst.init(None)

        pipeline_string = self.get_pipeline()
        self._pipeline = Gst.parse_launch( pipeline_string )

        appsink = self._pipeline.get_by_name("appsink")
        appsink.connect("new-sample", self.OnNewSample)

        if len(self._videoTracks) >= 1:

            self._videoPlayer = self._pipeline.get_by_name( "videoplayer" )
            self._pipeline.set_state( Gst.State.PAUSED )

        else:

            self._pipeline.set_state( Gst.State.PLAYING )


    def change_track( self, video_location = "" ):

        if self._videoPlayer is None: 
            return
        
        if video_location != "" and video_location != self._active_track: 

            self.pause( True )
            self._videoPlayer.set_property("location", video_location )  
            self.pause( False )


    def quit( self ):

        if self._pipeline is not None: 

            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None


    def pause( self, pauseState = True ):

        if self._pipeline is not None:

            if pauseState is False: 

                if self.isPipelinePaused is True:
                    self.isPipelinePaused = False
                    self._pipeline.set_state( Gst.State.PLAYING )

            else:

                if self.isPipelinePaused is False:
                    self.isPipelinePaused = True
                    self._pipeline.set_state(Gst.State.PAUSED)


    def loop( self ):

        if not self.command_queue.empty():
            self.handle_commands()
        
        sleep( self._framerate )
            

    def handle_commands(self):

        command = self.command_queue.get() 
        
        if "release" in command.keys():

            release = command["release"]

            if release is True:
                self.quit()
                return
            
        if "videotrack" in command.keys():

            trackToPlay = command["videotrack"]
            
            if trackToPlay != "":
                self.change_track( trackToPlay )

        if "pause" in command.keys():

            playState = command["pause"]
            self.pause( playState )
 

