import cv2 
from time import sleep
from PIL import Image
import queue

FLIP_VERTICALLY = True
FLIP_HORIZONTALLY = False
MAX_QUEUE_SIZE = 4

class VideoPlayer( object ):

    def __init__(self, output_queue, input_queue ):
        
        self.playlist = None

        self.frame_queue = output_queue
        self.command_queue = input_queue

        self.video_file = None
        self.video_capture = None

        self.frame = None
        self._isPaused = True


    def updateCapture( self,  ): 

        self._isPaused = False

        videoFile = self.playlist[ self._video_index ]
        
        if self.video_capture is None:

            self.video_capture = cv2.VideoCapture( videoFile )
            self.video_file = videoFile

        else:
            
            if videoFile != self.video_file:

                self.video_capture.release()
                self.video_capture.open(videoFile) 
                
                self.video_file = videoFile


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
            sleep(1/30)
            

    def handle_commands(self):

        try:

            command = self.command_queue.get(timeout=1)  

            if "paused" in command.keys():
                self._isPaused = command["paused"]

            if "playlist" in command.keys():
                if self.playlist is None:
                    self.playlist = command["playlist"]
                    
            if "voice_index" in command.keys():
                self._video_index = command["voice_index"]

            if self.playlist is not None:

                if self._isPaused is False:
                    self.updateCapture( )
                    print( f"videoCapture should be updated" )

            if "kill" in command.keys():
                exit = command["kill"]
                if exit is True: 
                    self.quit()
        
        except queue.Empty:
            pass 

    def is_playing(self):
        return self.video_capture.isOpened()

    def release(self):
        self.video_capture.release()

    def quit( self ):
        self.release()
