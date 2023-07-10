#!/usr/bin/env python3
########################################################################
# Filename    : __videostream.py
# Description : display video via OpenCV
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import sys
import json
import numpy as np
from threading import Thread, Lock
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

from ..utils.__utils_objects import AVAILABLE_TOPICS, PEER

class VideoStream( Node ):

        def __init__( self, Master=None):

            super().__init__("videostream", namespace=f"{PEER.USER.value}_0")

            self._master = Master

            self._sub_video = None
            self._sub_master = None

            self.TimeLeft = 0

            self._pipeline = None

            self._peer_type = PEER.USER.value

            self._playtime = 10 * 60
            
            self.isGamePlayEnable = True
            self.forceStreamDebug = True
            
            self._isHighQualityCodec = False

            self._thread = None
            self._lock = Lock()

            self.start()


        @property
        def udpPort( self ): 
            return 3000
        
 
        @property
        def udpPort( self ): 
            return 3000
        
        def start(self):
            
            self._declare_parameters()
            self._init_subscribers()
            self._render_pipeline()

        def _declare_parameters( self ):
            self.declare_parameter( "peer_index", 0 )
            self.declare_parameter( "resolution", (720,480) )


        def _init_subscribers( self ):
            

            self._sub_master = self.create_subscription(
                String,
                f"/{PEER.MASTER.value}/{AVAILABLE_TOPICS.HEARTBEAT.value}",
                self.OnMasterPulse,
                qos_profile=qos_profile_sensor_data
            )
            
            self._sub_master  # prevent unused variable warning
            self.get_logger().info("subscriber is running")


        def OnNewSample(self, sink):
            
            #with self._lock:

                sample = sink.emit("pull-sample")
                buf = sample.get_buffer()

                caps = sample.get_caps()

                frame_width = caps.get_structure(0).get_value("width")
                frame_height = caps.get_structure(0).get_value("height")

                _, buffer = buf.map(Gst.MapFlags.READ)

                frame = np.frombuffer(buffer.data, dtype=np.uint8)
                frame = frame.reshape((frame_height, frame_width,3))
            
                if self._master._gui is not None: 
                    self.get_logger().info(f"new sample video should be {self.isGamePlayEnable}")
                    self._master._gui._rosVideoUpdate( frame, self.isGamePlayEnable, self._playtime )
                    self._master._gui._isGamePlayEnable = self.isGamePlayEnable

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
                "video/x-raw, format=BGR ! "
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

        def _start_pipeline( self ): 

            self._thread = Thread(target=self._render_pipeline)
            self._thread.daemon = True  # Définit le thread en tant que thread démon
            self._thread.start()

        def _render_pipeline( self ):
            
            Gst.init(None)

            pipeline_string = self.h265_decoder() if self._isHighQualityCodec is True else self.h264_decoder()

            self._pipeline = Gst.parse_launch(pipeline_string)

            appsink = self._pipeline.get_by_name("appsink")
            appsink.connect("new-sample", self.OnNewSample)

            self._pipeline.set_state(Gst.State.PLAYING)

            """
            try:
                while True:
                    pass  # Garder le thread actif
            except KeyboardInterrupt:
                pass  # Permettre l'interruption du thread lorsque le programme principal est interrompu
            finally:
                self._pipeline.set_state(Gst.State.NULL)
                self._pipeline = None       
            """



        def OnMasterPulse( self, msg ):
            
            with self._lock:

                master_pulse = json.loads( msg.data )
                self.get_logger().info("masterPulse")
                if "peers" in master_pulse: 

                    peers = master_pulse["peers"]
                    peerUpdate = f"peer_{self.get_parameter('peer_index').value}"
                
                    if peerUpdate in peers: 

                        statusUpdate = peers[peerUpdate]

                        if "enable" in statusUpdate and "playtime" in statusUpdate:

                            self.isGamePlayEnable = self.forceStreamDebug #statusUpdate["enable"]
                            self._playtime = statusUpdate["playtime"]

                            """
                            if self._master._gui is not None: 
                                self._master._gui._isGamePlayEnable = statusUpdate["enable"]
                                self.get_logger().info(f"video should be {self.isGamePlayEnable}")
                            """
                        else:
                    
                            self.isGamePlayEnable = self.forceStreamDebug      
                else:
                        self.isGamePlayEnable = self.forceStreamDebug 
                    
                self.updatePipelineStatus()


        def updatePipelineStatus(self):

            if self.isGamePlayEnable is True:

                if self.isPlaying is False:
                    self.isPlaying = True
                    #self._pipeline.set_state(Gst.State.PLAYING)

            else: 

                if self.isPlaying is True:
                    self.isPlaying = False
                    #self._pipeline.set_state(Gst.State.PAUSED) 

            

                    
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

