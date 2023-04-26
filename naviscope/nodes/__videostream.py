#!/usr/bin/env python3
########################################################################
# Filename    : __videostream.py
# Description : display video via OpenCV
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import json
import cv2

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage # Image is the message type

from ..utils.__utils_objects import AVAILABLE_TOPICS, PEER

class VideoStreamNode( Node ):

        def __init__( self, **kwargs):

            super().__init__("videostream", namespace="operator_0")

            self._sub_video = None
            self._sub_peer= None

            self._video_bridge = None 

            self._gst_pipeline = None

            self._cv_window_opened = False
            self._gst_window_opened = False 

            self._operator_type = PEER.USER.value
            
            self.start()


        def start(self):
            
            self._declare_parameters()
            self._init_subscribers()


        def get_render_pipeline( self, address ):

            pipeline_str = (f"udpsrc address= {address} port={self._upd_port} caps=application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 " 
                "! rtph264depay "
                "! h264parse "
                "! decodebin "
                "! videoconvert "
                "! videoscale "
                "! autovideosink sync=false")

            return pipeline_str


        def _declare_parameters( self ):

            self.declare_parameter( "verbose", False )
            self.declare_parameter( "peer_index", 0 )
            self.declare_parameter( "resolution", (640,480) )

            self.declare_parameter( "opencv_render", True )


        def _init_subscribers( self ):
            
            self._sub_peer= self.create_subscription(
                String, 
                f"/drone_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.HEARTBEAT.value}",
                self._on_peer_pulse,
                10
            )

            self._sub_peer
        
            self._video_bridge = CvBridge()

            self._sub_video = self.create_subscription(
                CompressedImage, 
                f"/drone_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.STREAM.value}", 
                self._on_videostream, 
                qos_profile=rclpy.qos.qos_profile_sensor_data
            )

            self._sub_video


        def _instantiate_opencv_window( self ):

            if( self.get_parameter("opencv_render").value is True and self._cv_window_opened is False ):
                
                cv2.namedWindow("view", cv2.WINDOW_NORMAL)
                # Configurer la fenêtre en plein écran
                cv2.setWindowProperty("view", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

                self._cv_window_opened = True
        

        def _instantiate_gstreamer( self, address ):
            
            if( self.get_parameter("opencv_render").value is False and self._gst_window_opened is False  ):
                
                self._gst_window_opened = True
                
                pipeline_str = self.get_render_pipeline( address )
                self._gst_pipeline = Gst.parse_launch( pipeline_str )

                self._gst_pipeline.set_state( Gst.State.PLAYING )        


        def _on_videostream( self, frame ):

            if( self.get_parameter("opencv_render").value is True ):
                
                if self._cv_window_opened is False:
                    self._instantiate_opencv_window()

                current_frame = self._video_bridge.compressed_imgmsg_to_cv2(frame, desired_encoding="passthrough")
                cv2.imshow('view', current_frame)


        def _on_peer_pulse( self, pulse_msg ):
            
            if self._cv_window_opened is False:
                if self._gst_window_opened is False:
                    peer_info = json.loads( pulse_msg.data )
                    self._instantiate_gstreamer(  peer_info["address"] )
               

        def exit( self ):

            if( self.get_parameter("opencv_render").value is True ):
                # cv2.destroyAllWindows() simply destroys all the windows we created.
                cv2.destroyAllWindows()

            else:

                if self._gst_pipeline is not None: 

                    self._gst_pipeline.set_state(Gst.State.NULL)
                    self._gst_pipeline = None
 


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

