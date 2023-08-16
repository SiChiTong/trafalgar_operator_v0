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
from gi.repository import Gst, GObject



class VideoStream( object ):

        def __init__( self, Master=None):

            super().__init__()

            self._master = Master
            self._pipeline = None
            self._isHighQualityCodec = False
            self.isPlaying = False

        @property
        def udpPort( self ): 
            return 3000
        
        @property
        def udpPort( self ): 
            return 3000
        
        def start(self):
            self._render_pipeline()

        def OnNewSample(self, sink):
            
            sample = sink.emit("pull-sample")
            buf = sample.get_buffer()

            caps = sample.get_caps()

            frame_width = caps.get_structure(0).get_value("width")
            frame_height = caps.get_structure(0).get_value("height")

            _, buffer = buf.map(Gst.MapFlags.READ)

            frame = np.frombuffer(buffer.data, dtype=np.uint8)
            frame = frame.reshape((frame_height, frame_width,3))
            
            self._master._rosVideoUpdate( frame,frame_width,frame_height )

            buf.unmap(buffer)

            frame = None
            sample = None
            buf = None

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
                "videoflip method=2 ! "
                "videoscale ! "
                "identity drop-allocation=true ! "
                "appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false async=false"
    
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


        def _render_pipeline( self ):

            self.loop = GObject.MainLoop()

            Gst.init(None)

            pipeline_string = self.h265_decoder() if self._isHighQualityCodec is True else self.h264_decoder()

            self._pipeline = Gst.parse_launch(pipeline_string)

            appsink = self._pipeline.get_by_name("appsink")
            appsink.connect("new-sample", self.OnNewSample)

            self._pipeline.set_state(Gst.State.PLAYING)
            self.isPlaying = True

            self.loop.run()

        def updatePipelineStatus(self):

            if self._master._isGamePlayEnable is True:

                if self.isPlaying is False:
                    self.isPlaying = True
                    self._pipeline.set_state(Gst.State.PLAYING)

            else: 

                if self.isPlaying is True:
                    self.isPlaying = False
                    self._pipeline.set_state(Gst.State.PAUSED) 

        def exit( self ):

            if self._pipeline is not None: 

                self._pipeline.set_state(Gst.State.NULL)
                self._pipeline = None

            if self.loop:
                self.loop.quit()

 

