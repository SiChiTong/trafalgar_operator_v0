#!/usr/bin/env python3
########################################################################
# Filename    : __videoPlayer.py
# Description : audio manager
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################

import cv2 
from PIL import Image
from time import sleep

FLIP_VERTICALLY = True
FLIP_HORIZONTALLY = False
MAX_QUEUE_SIZE = 2

class VideoPlayer( object ):

    def __init__(self, output_queue, input_queue ):
        
        self.playlist = None

        self.frame_queue = output_queue
        self.command_queue = input_queue

        self.video_file = None

        self.videoTrackToDisplay = None
        self.activeVideoTrack = None

        self.video_capture = None

        self.frame = None
        self._isPaused = True


    def updateCapture( self  ): 

        self._isPaused = False

        if self.video_capture is None:

            videoFile = self.playlist[ self.videoTrackToDisplay ]

            if videoFile is not None:

                try:
                    self.video_capture = cv2.VideoCapture( videoFile )
                    self.activeVideoTrack = self.videoTrackToDisplay
                except Exception as e: 
                    print(f"exception has occured while openning cv2.videocapture : {e}")

        else:
            
            if self.activeVideoTrack != self.videoTrackToDisplay :
                
                videoFile = self.playlist[ self.videoTrackToDisplay ]

                if videoFile is not None: 

                    try:

                        self.video_capture.release()
                        self.frame = None

                        self.video_capture.open(videoFile) 
                
                        self.activeVideoTrack = self.videoTrackToDisplay

                    except Exception as e:
                        print(f"exception has occured while openning cv2.videocapture : {e}")


    def read_frame(self):

        if self.video_capture is not None:
                
            if self._isPaused is True:
                return
            
            ret, frame = self.video_capture.read()

            if ret:

                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = Image.fromarray(image)

                #cv2.imshow("Video", frame)  # Afficher la vidéo avec OpenCV
                #cv2.waitKey(30)  # Attendre 30 ms entre chaque frame (ajustez selon vos besoins)

                if FLIP_VERTICALLY is True:
                    image = image.transpose(Image.FLIP_TOP_BOTTOM)

                frame_data = image

                self.frame = frame_data

                if self.frame_queue.qsize() >= MAX_QUEUE_SIZE:
                    self.frame_queue.get() 
        
                self.frame_queue.put(frame_data)

            else:

                self.frame = None



    def loop( self ):
        
        while True:

            if not self.command_queue.empty():
                self.handle_commands()
       
            self.read_frame()
            sleep( 1 / 25 )
   

    def handle_commands(self):

        try:

            command = self.command_queue.get()  

            if "paused" in command.keys():
                self._isPaused = command["paused"]

            if "playlist" in command.keys():
                if self.playlist is None:
                    self.playlist = command["playlist"]
       
            if "voice_index" in command.keys():
                self._video_index = command["voice_index"]
       
            if "videotrack" in command.keys():
                self.videoTrackToDisplay = command["videotrack"]

            if self.playlist is not None:

                if self._isPaused is False:
                    self.updateCapture( )

            if "kill" in command.keys():
                exit = command["kill"]
                if exit is True: 
                    self.quit()
        
        except Exception as e:
            print(f"{e}")

    def is_playing(self):
        return self.video_capture.isOpened()

    def release(self):
        if self.video_capture is not None:
            self.video_capture.release()

    def quit( self ):
        self.release()
