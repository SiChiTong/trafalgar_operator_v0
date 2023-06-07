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

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from rclpy.qos import qos_profile_sensor_data

from ..utils.__utils_objects import AVAILABLE_TOPICS, PEER

class VideoStreamNode( Node ):

        def __init__( self, **kwargs):

            super().__init__("videostream", namespace=f"{PEER.USER.value}_0")

            self._sub_video = None
            self._sub_watchdog = None
            self._sub_master = None

            self.PlayTime = 0
            self.TimeLeft = 0

            self._pipeline = None
            self.isPlaying = False

            self.isGamePlayEnable = False
            self.isBlackScreenRendered = True

            self.sink_black = None
            self.sink_drone = None 

            self._peer_type = PEER.USER.value

            self._playtime = 10 * 60
            self._timeLeft = self._playtime

            self.start()
            self._timeout = 60


        @property
        def udpPort( self ): 
            return 3000
        
        
        def start(self):
            
            self._declare_parameters()
            self._init_subscribers()

            self.startTimer()
            self.start_render_pipeline()


        def _declare_parameters( self ):

            self.declare_parameter( "verbose", False )
            self.declare_parameter( "peer_index", 0 )
            self.declare_parameter( "resolution", (320,240) )


        def _init_subscribers( self ):
            

            self._sub_watchdog = self.create_subscription(
                Bool,
                AVAILABLE_TOPICS.WATCHDOG.value,
                self.OnPeerConnection,
                qos_profile=qos_profile_sensor_data
            )

            self._sub_watchdog   

        
            self._sub_gameplay = self.create_subscription(
                String, 
                f"/{PEER.MASTER.value}/{AVAILABLE_TOPICS.GAMEPLAY.value}",
                self.OnGameplay,
                10
            )

            self._sub_master


        def startTimer( self ):

            self.create_timer(  1, self.elapsed )

        def elapsed( self ): 

            if self.isGamePlayEnable is True: 

                if self._timeLeft > 0:
                    self._timeLeft -= 1


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

                    if self.isBlackScreenRendered is False: 
                        self.enableDroneView( False )
                        

        def OnPeerConnection( self, msg ):

            peer_connected = msg.data

            if( peer_connected is True ):

                if self.isGamePlayEnable is True:
                    
                    if self.isBlackScreenRendered is True: 
                        self.enableDroneView( True )

                else:

                    if self.isBlackScreenRendered is False: 
                        self.enableDroneView( False )
            else:

                if self.isBlackScreenRendered is False: 
                    self.enableDroneView( False )



        def start_render_pipeline( self ):
            
            Gst.init(None)

            display_resolution = self.get_parameter('resolution').value

            pipeline_string = (
                "videotestsrc pattern=black ! "
                f"video/x-raw, framerate=(fraction)5/1, width=(int){display_resolution[0]},height=(int){display_resolution[1]} ! "
                "compositor name=comp sink_0::alpha=1 sink_1::alpha=0 ! "
                "videoconvert ! autovideosink "
                f"udpsrc port={self.udpPort} ! "
                "application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! "
                "rtph264depay ! "
                "decodebin ! "
                "videoconvert ! "
                "videoscale ! "
                f"video/x-raw, width=(int){display_resolution[0]}, height=(int){display_resolution[1]} ! "
                "comp. "
            )

            self._pipeline = Gst.parse_launch(pipeline_string)

            compositor = self._pipeline.get_by_name("comp")

            self.sink_black = compositor.get_static_pad("sink_0")
            self.sink_drone = compositor.get_static_pad("sink_1")

            self.isBlackScreenRendered = True

            self._pipeline.set_state(Gst.State.PLAYING)
        

        def enableDroneView(self, enable = True ):
            
            self.isBlackScreenRendered = enable

            if enable:
                
                self.sink_black.set_property("alpha", 0)
                self.sink_drone.set_property("alpha", 1)
            
                self._pipeline.set_state(Gst.State.PLAYING)

                self._timeLeft = self._playtime

            else:

                self.isBlackScreenEnable = True

                self.sink_drone.set_property("alpha", 0)
                self.sink_black.set_property("alpha", 1)
            
                time.sleep(1)
                
                self._pipeline.set_state(Gst.State.PAUSED)




        def exit( self ):

            if self._pipeline is not None: 

                self._pipeline.set_state(Gst.State.NULL)
                self._pipeline = None
 


def main(args=None):

    rclpy.init(args=args)

    videostream_node = None 

    try:

        videostream_node  = VideoStreamNode()

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

