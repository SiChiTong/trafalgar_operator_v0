#!/usr/bin/env python3
########################################################################
# Filename    : __videostream.py
# Description : display video via OpenCV
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import sys
import json
import time
import numpy as np
import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from rclpy.qos import qos_profile_sensor_data

from ..utils.__utils_objects import AVAILABLE_TOPICS, PEER

class VideoStream( Node ):

        def __init__( self, MASTER_APP):

            super().__init__("videostream", namespace=f"{PEER.USER.value}_0")

            self._master = MASTER_APP

            self._sub_video = None
            self._sub_watchdog = None
            self._sub_master = None

            self.PlayTime = 0
            self.TimeLeft = 0

            self._pipeline = None

            self.isGamePlayEnable = False

            self._peer_type = PEER.USER.value

            self._playtime = 10 * 60
 
            self.start()


        @property
        def udpPort( self ): 
            return 3000
        
        
        def start(self):
            
            self._declare_parameters()
            self._init_subscribers()
            self._render_pipeline()


        def _declare_parameters( self ):

            self.declare_parameter( "verbose", False )
            self.declare_parameter( "peer_index", 0 )
            self.declare_parameter( "resolution", (720,480) )


        def _init_subscribers( self ):
            
            """
            self._sub_watchdog = self.create_subscription(
                Bool,
                AVAILABLE_TOPICS.WATCHDOG.value,
                self.OnPeerConnection,
                qos_profile=qos_profile_sensor_data
            )

            self._sub_watchdog   

            """

            self._sub_gameplay = self.create_subscription(
                String, 
                f"/{PEER.MASTER.value}/{AVAILABLE_TOPICS.GAMEPLAY.value}",
                self.OnGameplay,
                10
            )

            self._sub_master


        def OnGameplay( self, msg ):

            playUpdate = json.loads( msg.data )

            if "index" in playUpdate: 
                
                peer_index = self.get_parameter('peer_index').value
                updateIndex = playUpdate["index"]

                if updateIndex == peer_index :
                    
                    if "enable" in playUpdate and "playtime" in playUpdate:

                        self.isGamePlayEnable = playUpdate["enable"]
                        self.PlayTime = playUpdate["playtime"]

                else:
                    
                    self.isGamePlayEnable = False
                        


        def OnNewSample(self, sink):

            sample = sink.emit("pull-sample")
            buf = sample.get_buffer()

            caps = sample.get_caps()
            frame_width = caps.get_structure(0).get_value("width")
            frame_height = caps.get_structure(0).get_value("height")
            _, buffer = buf.map(Gst.MapFlags.READ)

            frame = np.frombuffer(buffer.data, dtype=np.uint8)
            frame = frame.reshape((frame_height, frame_width, 3))

            if self._master._gui is not None: 
                self._master._gui._rosVideoUpdate( frame, self.isGamePlayEnable, self._playtime )

            return Gst.FlowReturn.OK
        

        def _render_pipeline( self ):
            
            Gst.init(None)

            pipeline_string = (

                f"udpsrc port={self.udpPort} ! "
                "application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! "
                "rtph264depay ! "
                "decodebin ! "
                "videoconvert ! "
                "videoscale ! "
                "! appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false "
    
            )

            self._pipeline = Gst.parse_launch(pipeline_string)

            appsink = self._pipeline.get_by_name("appsink")
            appsink.connect("new-sample", self.OnNewSample)

            self._pipeline.set_state(Gst.State.PLAYING)
        

        def exit( self ):

            if self._pipeline is not None: 

                self._pipeline.set_state(Gst.State.NULL)
                self._pipeline = None
 


def main(args=None):

    rclpy.init(args=args)

    videostream_node = None 

    try:

        videostream_node  = VideoStream()

        rclpy.spin( videostream_node  )

    except Exception as e:
        print( "an exception has been raised while spinning videostream node : ", e )
        exception_type, exception_object, exception_traceback = sys.exc_info()
        filename = exception_traceback.tb_frame.f_code.co_filename
        line_number = exception_traceback.tb_lineno

        print("Exception type: ", exception_type)
        print("File name: ", filename)
        print("Line number: ", line_number)

    except KeyboardInterrupt:
        print("user force interruption")
        
    finally:

        if videostream_node is not None:
            videostream_node.exit()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

