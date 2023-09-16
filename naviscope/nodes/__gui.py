#!/usr/bin/env python3
########################################################################
# Filename    : __gui.py
# Description : 
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################

import os
import traceback
import datetime
import json

import math
import numpy as np

from multiprocessing import Process, Event, Queue
from threading import Thread,Lock

import cv2 

import rclpy

import customtkinter
from PIL import ImageTk, Image

customtkinter.set_appearance_mode("light")  # Modes: system (default), light, dark
customtkinter.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

from .__controller import Controller
from ..components.__videoPlayer import VideoPlayer
from ..components.__videostream import VideoStream

from ..utils.__utils_objects import DIRECTION_STATE


ENABLE_UDP_STREAM = True
ENABLE_OCV_PLAYER = True

STOP_EVENT = Event()

Process_videoStream = None
Q_COMMAND_videoStream = Queue()
Q_FRAME_videoStream = Queue()

Process_videoPlayer = None 
Q_COMMAND_videoPlayer = Queue()
Q_FRAME_videoPlayer = Queue()


class Display(customtkinter.CTk):

    APP_NAME = ""
    WIDTH = 480
    HEIGHT = 480
    
    def __init__(
            self
        ):
        
        super().__init__()
        
        global ENABLE_UDP_STREAM, ENABLE_OCV_PLAYER
        self._enableUDPStream = ENABLE_UDP_STREAM
        self._enableOCVPlayer = ENABLE_OCV_PLAYER

        self.ZoomLevel = 10
        
        self.last_image = None 

        self.gameplayEnable = False
        self._nameIsVisible = False

        self._node = None

        self._drone_orientation = None

        self._last_center_image = None
        self.rawFrame = None

        self._videoFrame = None
        self._squareFrameEnabled = True


        self._videostream = None
        
        self.video_capture=None
        self.flipVertically = True

        self.videoPlayerFrame = None

        self.title(Display.APP_NAME)

        self.geometry(str( self.winfo_screenwidth() ) + "x" + str( self.winfo_screenheight() ))
        #self.minsize(Display.WIDTH, Display.HEIGHT)

        self.attributes("-fullscreen", True) 

        self.canvaResolution = (self.winfo_screenwidth(), self.winfo_screenheight()) 

        self._blackScreen = False 

        self._playtime = 10*60
        self._playtimeLeft = 0

        self._thread = None
        self._lock = Lock()

        self._videoStreamPlayState = "stop"
        
        self._text_name = None
        self._text_elapsed_time = None
        self._text_angle = None
        self._text_direction = None
        
        self._text_img = None
        self._text_center = None
        
        self.hudTexts = None

        self.imgList = {}
        self._center_image_name = ""

        self._vidList = {}

        self._center_image = None
        self._canvas_frame = None

        self.arrow_color = self.arrowColor_hide

        self._videoPlayerCommand = {

            "paused" : False,
            "playlist" : {},
            "voice_index" : 0,
            "videotrack" : "",
            "released": True

        }


    @property
    def loop_gameplay( self ):
        return 33
    
    @property
    def loop_idle( self ):
        return 100
    
    @property
    def screen_center_x(self):
        return 240
    @property 
    def screen_center_y(self):
        return 240

    @property
    def arrow_radius(self):
        return 190

    @property
    def videoWidth(self):
        return 480

    @property
    def videoHeight(self):
        return 480
    
    @property
    def tagHud(self):
        return "hud"
    
    @property
    def tagVideo(self):
        return "video"
    
    @property
    def tagImg(self):
        return "img"
    
    @property
    def tagArrow(self):
        return "arrow"
    
    @property
    def tagDescription(self):
        return "description"
    
    @property
    def centerTextSize(self):
        return 40        #ideal_font_size = 80
    
    @property 
    def regularTextSize( self ):
        return 20
    
    @property
    def arrowColor_base( self ):
        return "black"
    
    @property
    def arrowColor_forward(self):
        return "green"
    
    @property
    def arrowColor_backward(self):
        return "black"
    
    @property
    def arrowColor_steer_limit(self):
        return "red"
    
    @property
    def arrowColor_hide(self):
        return "black"
    
    @property
    def text_flip_method( self ):
        return 180
    
    def _initialize( self ):   
        
        self.bind_events()
        self._start_controller_node()

        self._create_window()

        self.updateHud()



    def _start( self ):

        self._initialize()
        self.mainloop()
    

    def bind_events( self ):

        self.bind("<Control-c>", self._closing_from_gui)
        #self.bind("z", self.set_buttonPress)
        #self.bind("s", self.set_buttonPress)
        #self.bind("g", self.set_gameplayEnable)
        #self.bind("<<closeGUI>>", self._stop)

    """

    def set_buttonPress( self, event ):
        self._node.OnButtonPress(shortPress=True,longPress=False)
        self._node.droneDirection = self._node._direction
        self._node._audioManager.unlock_direction = True
        self._node._audioManager.unlock_orientation = True


    def set_gameplayEnable( self, event ):

        self._node.isGamePlayEnable = not self._node.isGamePlayEnable
        self._node._playtimeLeft = 5*60
        self._node._audioManager.set_playtime(5*60)
        self._node._update_direction(DIRECTION_STATE.STOP.value)

        if self._node.isGamePlayEnable:
            self._node._audioManager.gameplayMusic( True, 0 )
        else:
            self._node._audioManager.onGameOver()

    """


    def _create_window( self ): 

        self.canvas = customtkinter.CTkCanvas(self, width=self.canvaResolution[0], height=self.canvaResolution[1])
        self.canvas.pack( side="top", fill="both", expand=True )

        self.loadDatas()

        self.loadImages()
        self.draw_image_frame()

        self.loadVideo()

        self.draw_boxes()
        self.draw_texts()
        
        self.renderBlackScreen()

    def loadDatas( self ):

        DIR = os.path.join(
        os.getcwd(), 
        'install',            
        'naviscope',   
        'share',         
        'naviscope',    
        'datas'                
        )

        files = [
            os.path.join( DIR, filename )
            for filename in os.listdir( DIR )
            if filename.endswith(".json")
        ]

        self.datasList = { os.path.splitext(os.path.basename(file))[0] :  self.open_datas(file) for file in files}

    def open_datas( self, file_path ):
        
        datas = None

        with open( file_path, "r" ) as fichier_json:
            datas = json.load(fichier_json)

        return datas


    def loadImages( self ):

        DIR = os.path.join(
        os.getcwd(), 
        'install',            
        'naviscope',   
        'share',         
        'naviscope',    
        'media',                  
        'img'                 
        )

        files = [
            os.path.join( DIR, filename )
            for filename in os.listdir( DIR )
            if filename.endswith(".jpg")
        ]

        self.imgList = { os.path.splitext(os.path.basename(file))[0] : self.load_and_resize( file, flipVertically=True ) for file in files }



    def loadVideo( self ):

        DIR = os.path.join(
        os.getcwd(), 
        'install',            
        'naviscope',   
        'share',         
        'naviscope',    
        'media',                  
        'vid'                 
        )

        files = [
            os.path.join( DIR, filename )
            for filename in os.listdir( DIR )
            if filename.endswith(".mp4")
        ]

        self._vidList = { os.path.splitext(os.path.basename( file ))[0]: file for file in files }



    def load_and_resize(self, path, width=480, height=480, flipVertically=False):
   
        image = Image.open(path)
        image = image.resize((width, height), Image.ANTIALIAS)
    
        if flipVertically is True:
            image = image.transpose(Image.FLIP_TOP_BOTTOM)

        return ImageTk.PhotoImage(image)


    def draw_image_frame( self ): 

        self._center_image_name = "pirateHead"

        self._canvas_frame = self.canvas.create_image(
            0, 
            0, 
            anchor="nw", 
            image=self.imgList[self._center_image_name], 
            tags=self.tagVideo
        )


    def draw_boxes( self ):

        self._box_elapsed = self.canvas.create_rectangle(
            0, 
            0, 
            0, 
            0,
            fill="black", 
            tags=self.tagHud
        )


    def draw_texts( self ):

        self._text_elapsed_time = self.canvas.create_text(
            280, 
            100, 
            text="", 
            fill="white", 
            font=("Arial", 15, "bold"), 
            anchor="w", 
            angle = self.text_flip_method,
            tags=self.tagHud
        )
        
        self._text_img = self.canvas.create_text(
            280, 
            100, 
            text="", 
            fill="black", 
            font=("Arial", self.regularTextSize, "bold"), 
            justify=customtkinter.CENTER,
            anchor="w", 
            angle = self.text_flip_method,
            tags=self.tagImg
        )

        self._text_center = self.canvas.create_text(
            360, 
            250,
            text="", 
            fill="blue", 
            font=("Arial", 40), 
            justify=customtkinter.CENTER,
            anchor="w", 
            angle = self.text_flip_method,
            tags=self.tagHud
        )

        self.hudTexts = [
            self._text_elapsed_time,
            self._text_center
        ]

        
    def crop_from_center(self, frame, frameSize, ZoomLevel = 10):
        #for later: > zoom method via circular btn
        #  
        frame_height, frame_width = frameSize

        ZoomLevel = 100 - ZoomLevel
        
        if ZoomLevel == 0:
            return frame

        elif ZoomLevel == 100:
            ZoomLevel = 90

        crop_width = int(frame_width * ZoomLevel/100)
        crop_height = int(frame_height * ZoomLevel/100)

        start_x = (frame_width - crop_width) // 2
        start_y = (frame_height - crop_height) // 2

        end_x = start_x + crop_width
        end_y = start_y + crop_height
    
        cropped_image = frame[start_y:end_y, start_x:end_x]
    
        return cropped_image


    def set_text( self, canvasText, textToDisplay = "" ): 
        self.canvas.itemconfig( canvasText, text=textToDisplay)

    def set_center_text( self, textToDisplay = "" ):
        
        window_width = self.canvas.winfo_width()
        window_height = self.canvas.winfo_height()

        text = self.canvas.itemconfig(
            self._text_center, 
            text=textToDisplay
        )

        text_bbox = self.canvas.bbox(text)
        text_width = text_bbox[2] - text_bbox[0]
        text_height = text_bbox[3] - text_bbox[1]
        text_x = (window_width - text_width) // 2 + text_width / 2
        text_y = (window_height - text_height) // 2 + text_height / 2

        self.canvas.coords(text, text_x, text_y)


    def set_bounty_figure( self ):
    
        if self._audioManager.HistIndexReached is False:
            return        
     
        if self._node.tiltSwitchTriggered is True:

            index = 3 #random number between all bounty names
            self._bounty_figure = ""

        else: 
                
            self._bounty_figure = ""


    def set_center_img( self, imgName = "", isAVideo = False ):

        if imgName == "":
            return 
        
        if isAVideo is False:
            
            if self._center_image_name != imgName:

                image = self.imgList[imgName]
                self.updateImageOnCanva( image )
                
        else:
            
            if self._vidList is None:
                return
        
            image = self.getVideoFileFrame()

            if image is not None:
                self.videoPlayerFrame = image
                self.updateImageOnCanva( image )
        
        
        self._center_image_name = imgName


    
    def updateImageOnCanva( self, image = None ):
    
        if image is not None and image != self._last_center_image:
            self._last_center_image = image
            self.canvas.itemconfig( self._canvas_frame, image=self._last_center_image )


    def updateVideoPlayerPlayState( self, released = False, paused = False ):

        global Q_COMMAND_videoPlayer

        if self._node is not None:
            
            videoList = self._videoPlayerCommand["playlist"] is None
            pauseState = self._videoPlayerCommand["paused"] != paused
            releaseState = self._videoPlayerCommand["released"] != released
            trackSelection = self._videoPlayerCommand["videotrack"] != self._node._audioManager.imgToDisplay

            onStateChange = videoList or pauseState or releaseState

            if onStateChange is True:
                
                self._videoPlayerCommand["paused"] = paused
                self._videoPlayerCommand["playlist"] = self._vidList
                self._videoPlayerCommand["voice_index"] = self._node._audioManager.voice_index
                self._videoPlayerCommand["videotrack"] = self._node._audioManager.imgToDisplay
                self._videoPlayerCommand["released"] = released
            
                if self._enableOCVPlayer is True:
                
                    Q_COMMAND_videoPlayer.put( self._videoPlayerCommand )



    def getVideoFileFrame( self ):
        
        global Q_FRAME_videoPlayer

        frame = None

        if self._enableOCVPlayer is True:

            if not Q_FRAME_videoPlayer.empty():

                frame = Q_FRAME_videoPlayer.get()

        if frame is not None:
            frame = ImageTk.PhotoImage(image=frame)

        return frame



    def set_box( self, box, fillColor="", outlineColor="" ):
        self.canvas.itemconfig(box, fill=fillColor, outline=outlineColor)

    def format_time(self, delta):
        total_seconds = int(delta.total_seconds())
        minutes, seconds = divmod(total_seconds, 60)
        return f"{minutes:02}:{seconds:02}"


    def render_elapsed( self ):
            
        self._playtimeLeft = self._node._playtimeLeft

        if self._playtimeLeft > 0:

            time_delta = datetime.timedelta(seconds=self._playtimeLeft)
            formatted_time = self.format_time(time_delta)
                
            self.set_text( self._text_elapsed_time, f"{ formatted_time }" )


        self.render_elapsed_box( self._playtimeLeft ) 


    def render_elapsed_box(self, timeLeft = 0 ):
       
        bbox = self.canvas.bbox( self._text_elapsed_time )
    
        padding = 2
        rectangle_height = 20
        
        yOffset = -25

        x1 = bbox[0] - padding
        y1 = bbox[3] + yOffset + padding
        x2 = bbox[2] + padding
        y2 = bbox[3] + + yOffset + padding + rectangle_height

        self.canvas.coords( self._box_elapsed, x1, y1, x2, y2 )
        
        if timeLeft > 0:
            if timeLeft <= 60:
                self.set_box(self._box_elapsed, "red", "black")
            else: 
                self.set_box(self._box_elapsed, "black", "white")
        else:
            self.set_box( self._box_elapsed )

        if timeLeft > 0:
            self.canvas.tag_raise( self._text_elapsed_time )  


    def render_directional_arrow(self, currentAngle = 90 ):

        adapted_angle = 90

        if currentAngle > 90: 
            adapted_angle = 90 - ( currentAngle - 90 )

        if currentAngle < 90: 
            adapted_angle = 90 + ( 90 - currentAngle )
        
        if self.between(adapted_angle, 80, 100): 
            adapted_angle = 90

        self.canvas.delete(self.tagArrow)

        pivot_x = self.screen_center_x + self.arrow_radius * math.cos(math.radians( adapted_angle ))
        pivot_y = self.screen_center_y + self.arrow_radius * math.sin(math.radians( adapted_angle ))

        offset_length = 10 

        # Arrow points relative to pivot point
        points = [
            pivot_x, pivot_y + offset_length,
            pivot_x + 12, pivot_y,
            pivot_x + 3, pivot_y,
            pivot_x + 3, pivot_y - 8,
            pivot_x - 3, pivot_y - 8,
            pivot_x - 3, pivot_y,
            pivot_x - 12, pivot_y
        ]

        # Rotate arrow to face outward from circle's center
        angle_rad = math.radians( adapted_angle - 90 )

        pivot_x = sum(points[::2]) / len(points[::2])
        pivot_y = sum(points[1::2]) / len(points[1::2])

        for i in range(0, len(points), 2):
            x, y = points[i] - pivot_x, points[i+1] - pivot_y
            points[i] = pivot_x + (x * math.cos(angle_rad) - y * math.sin(angle_rad))
            points[i+1] = pivot_y + (x * math.sin(angle_rad) + y * math.cos(angle_rad))


        arrowColorFill = self.arrowColor_base
        arrowColorOutline = "white"

        if not self._node._audioManager.shipIsIddling:

            arrowColorFill = self.arrowColor_forward
            
            if self._node.droneSteerMin < adapted_angle < self._node.droneSteerMax:
                arrowColorFill = self.arrowColor_forward

            else:
                arrowColorFill = self.arrowColor_steer_limit


        self.canvas.create_polygon(
            points, 
            outline=arrowColorOutline, 
            fill=arrowColorFill,
            tags=self.tagArrow
        )


    def render_orientation(self ):
  
        current_angle = self._node.droneSteering

        if current_angle is not None:

            update_orientation = ""

            if current_angle >= 80 and current_angle <= 100:
                update_orientation = f"CENTRE: 0°"
                
            elif current_angle < 80:
                update_orientation = f"GAUCHE: {current_angle - 90}°"
                
            else:
                update_orientation = f"DROIT: {current_angle - 90}°"
        
            self._drone_orientation = update_orientation
            #self.set_text( self._text_steering, update_orientation )
            
            self.render_directional_arrow( current_angle )


    def render_direction( self, droneDirection ):

        if droneDirection is not None:

            if droneDirection == DIRECTION_STATE.FORWARD.value:  
                self.set_box(self._box_direction, "green", "white")

            elif droneDirection == DIRECTION_STATE.BACKWARD.value:
                self.set_box(self._box_direction, "red", "white")

            else :
                self.set_box(self._box_direction, "black", "white")


    def OnGstSample( self, frame, frameSize ):
        self.rawFrame = (frame, frameSize )

    def OnStandardSample( self ):
        
        frame, frameSize = self.rawFrame
        frameWidth, frameHeight = frameSize[0], frameSize[1]

        resized_frame = cv2.resize(frame, (self.videoWidth, self.videoHeight))
        color_conv = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
        
        img = Image.fromarray( color_conv )
        
        return ImageTk.PhotoImage( img )
    

    def OnSquareSample( self ):
        
        frame, frameSize = self.rawFrame

        frameToRender = frame

        #frameToRender = np.fliplr( frameToRender )
        frameToRender = np.flipud( frameToRender )
        #frameToRender = self.crop_from_center( frameToRender, frameSize, 10 )
        #frameToRender = cv2.resize(frameToRender, (self.videoWidth, self.videoHeight))
        frameToRender = cv2.cvtColor( frameToRender, cv2.COLOR_BGR2RGB)
        
        img = Image.fromarray( frameToRender )
  
        return ImageTk.PhotoImage( img )


    def OnCVBridgeFrame(self, frame):
        self.rawFrame = (frame, None)

    def readCVFrame( self ):

        frame, frameSize = self.rawFrame

        # Convertir les couleurs
        color_conv = cv2.cvtColor( frame, cv2.COLOR_BGR2RGB )

        # Convertir en image PIL et appliquer le flip vertical
        img = Image.fromarray(color_conv)
        img = img.transpose(Image.FLIP_TOP_BOTTOM)

        # Centrer sur un fond de 480x480 si nécessaire
        background = Image.new("RGB", (480, 480), (0, 0, 0))
        bg_w, bg_h = background.size
        img_w, img_h = img.size
        offset = ((bg_w - img_w) // 2, (bg_h - img_h) // 2)
        background.paste(img, offset)
        img = background

        return ImageTk.PhotoImage( img )
    

    def between(self, x, start, end):
        return start <= x < end
    
    
    def renderDroneView( self ):
        self._center_image_name = "frame"   
        if self.rawFrame is not None:

            self.clear_img_text()

            frameToRender = None
    
            if self._node.EnableUDPStream is True:
        
                frameToRender = self.OnSquareSample( ) if self._squareFrameEnabled is True else self.OnStandardSample( )
            else:
                frameToRender = self.readCVFrame()

            if frameToRender is not None:

                self._videoFrame = frameToRender
                self.canvas.itemconfig( self._canvas_frame, image= self._videoFrame)
  
            self.rawFrame = None


    def renderMainFrame( self ):

        self._blackScreen = False

        if self._node._audioManager is not None:
                        
            mediaToDisplay, isAVideo = self._node._audioManager.get_media_to_display()
            
            if mediaToDisplay is not None:

                if self._node._audioManager.HistIndexReached is False:
                    
                    self.updateVideoPlayerPlayState( released = True )
                    self.updateVideoStreamPlayState("stop")

                    self.set_center_img( mediaToDisplay, isAVideo )

                else:
                    
                    
                    if self._node._audioManager._voice_is_playing is True and self._node._audioManager.shipIsIddling is True: 
                        
                        self.updateVideoPlayerPlayState( released = False )
                        self.updateVideoStreamPlayState("stop")

                        self.set_center_img(mediaToDisplay, isAVideo)
                        
                    else:
                        
                        self.updateVideoPlayerPlayState( released = True )

                        if self._node.tiltSwitchTriggered is True:
                            
                            mediaToDisplay = "logo_moa"
                            isAVideo = False

                            self.updateVideoStreamPlayState("stop")
                            self.set_center_img(mediaToDisplay, isAVideo)

                        else:

                            self.updateVideoStreamPlayState("start")
                            self.renderDroneView()

            else:
                
                self.updateVideoPlayerPlayState( released = True )

                if self._node.tiltSwitchTriggered is True:

                    mediaToDisplay = "logo_moa"
                    isAVideo = False

                    self.updateVideoStreamPlayState("stop")
                    self.set_center_img( mediaToDisplay, isAVideo )

                else:

                    self.updateVideoStreamPlayState("start")
                    self.renderDroneView()


    
    def updateVideoStreamPlayState( self, state = "stop" ):
        
        global Q_COMMAND_videoStream

        if self._videoStreamPlayState != state:
            
            self._videoStreamPlayState = state
            Q_COMMAND_videoStream.put( self._videoStreamPlayState )

    
    def clear_hud_texts( self ):

        if self.hudTexts is not None:
            for text in self.hudTexts: 
                self.canvas.itemconfig(text, text="")

    def clear_img_text( self ):

        if self._text_img is not None:
            self.canvas.itemconfig(self._text_img, text="")
            self._center_image_name = ""


    def renderBlackScreen( self ):
        
        self.updateVideoPlayerPlayState( released = True )
        self.updateVideoStreamPlayState("stop")

        if self._blackScreen is False: 

            self.canvas.update_idletasks()

            self.set_center_img( "pirateHead" )
      
            self.clear_hud_texts()
 
            self.canvas.delete(self.tagArrow)
            self.set_box( self._box_elapsed )

            self._blackScreen = True    

      
    def renderName( self ):

        if self._nameIsVisible is False:

            droneName = self._node.droneName

            if droneName is not None:
          
                self.set_text( self._text_name, droneName )
                self._nameIsVisible = True


    def updateStream(self):
        
        global Q_FRAME_videoStream, Process_videoStream

        if Process_videoStream is not None: 
            
            if not Q_FRAME_videoStream.empty():
                        
                frame_data = Q_FRAME_videoStream.get()
                self.OnGstSample( frame_data["frame"], frame_data["size"])
                

    def updateHud(self):
        
        loop_delay = self.loop_idle

        if self._node is None:
            self.renderBlackScreen()
            self.after( loop_delay, self.updateHud)
            return
             
        if self._node.isGamePlayEnable is True or self._node._playtimeLeft > 0:
                
            loop_delay = self.loop_gameplay

            self.updateStream()

            self.renderMainFrame()

            if self._node._audioManager.FullHudIndexReached is True:

                self.render_orientation( )
                self.render_elapsed()
     
        else:
                
            self.renderBlackScreen()
        

        self.after( loop_delay, self.updateHud)



    def _stop(self):
        
        global STOP_EVENT

        if self._node is not None:

            try:

                self._node.destroy_node()
                
                if self._thread is not None: 
                    self._thread.join()
                    
            except Exception as e:
                traceback.print_exc()
        

        STOP_EVENT.set()

        try:
            self.destroy()

        except Exception as e:
            traceback.print_exc()


    def _action_on_shutdown(self, value):
        self._stop()


    def _closing_from_gui(self, event):
        self._stop()


    def _start_controller_node(self):

        self._thread = Thread( target=self._run_controller_node )
        self._thread.daemon = True
        self._thread.start()


    def _run_controller_node(self):

        rclpy.init( args=None )

        if self._node is None:
            self._node = Controller( Master=self, enableUDPStream = self._enableUDPStream ) 

        try:

            rclpy.spin( self._node )

        except Exception as e:
            traceback.print_exc()
        
        finally:

            rclpy.try_shutdown()






def start_videoStream():

    global ENABLE_UDP_STREAM, Q_FRAME_videoStream, Q_COMMAND_videoStream, STOP_EVENT, Process_videoStream

    if ENABLE_UDP_STREAM is True:

        Process_videoStream = Process(
            target=run_process_videoStream, 
            args=( Q_FRAME_videoStream, Q_COMMAND_videoStream, STOP_EVENT )
        )

        Process_videoStream.daemon = True
        Process_videoStream.start()


def run_process_videoStream( queueFrame, queueCommand, stopEvent  ):
        
    try:
            
        videostream = VideoStream(queueFrame, queueCommand)
        videostream.start()

        while not stopEvent.is_set():
            videostream.loop()

        videostream.quit()

    except Exception as e:
        traceback.print_exc()



def start_videoPlayer( ):

    global Q_FRAME_videoPlayer, Q_COMMAND_videoPlayer, STOP_EVENT, Process_videoPlayer

    Process_videoPlayer= Process(

        target=run_process_videoPlayer, 
        args=( Q_FRAME_videoPlayer , Q_COMMAND_videoPlayer, STOP_EVENT )
    
    )
            
    Process_videoPlayer.daemon = True
    Process_videoPlayer.start()


def run_process_videoPlayer( queueFrame, queueCommand, stopEvent ):

    try:
            
        video_capture = VideoPlayer( queueFrame, queueCommand)

        while not stopEvent.is_set():
            video_capture.loop()
  

        video_capture.quit()

    except Exception as e:
        traceback.print_exc()



def main(args=None):

    global Process_videoPlayer, Process_videoStream

    app = Display()

    try:

        if ENABLE_OCV_PLAYER is True:
            start_videoPlayer()
        
        if ENABLE_UDP_STREAM is True:
            start_videoStream()

        app._start()

    except Exception as e:
        traceback.print_exc()

    finally:

        app._stop()

        if Process_videoStream is not None:
            Process_videoStream.terminate()

        if Process_videoPlayer is not None: 
            Process_videoPlayer.terminate()


if __name__ == '__main__':
    main()

    