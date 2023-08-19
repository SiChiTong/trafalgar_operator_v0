#!/usr/bin/env python
import os
import traceback
import datetime
import cv2 # OpenCV library
import math

from time import sleep
from multiprocessing import Process, Event, Queue
from threading import Thread,Lock

import rclpy

import customtkinter
from PIL import ImageTk, Image

customtkinter.set_appearance_mode("light")  # Modes: system (default), light, dark
customtkinter.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

from .__controller import Controller
from ..components.__videostream import VideoStream

from ..utils.__utils_objects import DIRECTION_STATE

class Display(customtkinter.CTk):

    APP_NAME = ""
    WIDTH = 480
    HEIGHT = 480

    def __init__(
            self
        ):
        
        super().__init__()
        
        self.ZoomLevel = 10
        self.last_image = None 

        self._nameIsVisible = False

        self._node = None

        self._drone_orientation = None

        self._frame = None

        self._current_frame = None
        self._next_frame = None
        self.last_frame = None

        self._frame_has_been_updated = False

        self._videostream = None

        self.title(Display.APP_NAME)

        self.geometry(str( self.winfo_screenwidth() ) + "x" + str( self.winfo_screenheight() ))
        self.minsize(Display.WIDTH, Display.HEIGHT)

        self.attributes("-fullscreen", True) 

        self.canvaResolution = (self.winfo_screenwidth(), self.winfo_screenheight()) 

        self.bind("<Control-c>", self._closing_from_gui)

        self._blackScreen = False 

        self._playtime = 10*60
        self._playtimeLeft = 0

        self._loop_stream = 1
        self._loop_hud = 50

        self.iddleLoop = False

        self._thread = None
        self._lock = Lock()

        self._process = None

        self._frame_stop_event = Event()
        self._frame_queue = Queue()

        self._text_name = None
        self._text_elapsed_time = None
        self._text_angle = None
        self._text_direction = None
        
        self._text_img = None
        self._text_center = None
        
        self.hudTexts = None

        self.imgList = {}
        self._center_image_name = ""

        self._center_image = None
        self._canvas_frame = None

        self._hud_is_paused = False
        self.arrow_color = self.arrowColor_hide

        self._initialize()

    @property
    def screen_center_x(self):
        return 240
    @property 
    def screen_center_y(self):
        return 240

    @property
    def arrow_radius(self):
        return 185

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
        return "blue"
    
    @property
    def arrowColor_forward(self):
        return "green"
    
    @property
    def arrowColor_backward(self):
        return "red"
    
    @property
    def arrowColor_hide(self):
        return "black"
    
    @property
    def text_flip_method( self ):
        return 180
    
    def _initialize( self ):   
         
        self._init_components()
        self._create_window()
        self.updateHud()

    
    def _start( self ):
        self.mainloop()
    

    def _create_window( self ): 

        self.canvas = customtkinter.CTkCanvas(self, width=self.canvaResolution[0], height=self.canvaResolution[1])
        self.canvas.pack( side="top", fill="both", expand=True )

        self.loadImages()
        self.draw_image_frame()

        self.draw_texts()
        self.renderBlackScreen()


    def loadImages( self ):

        IMG_DIR = os.path.join(
        os.getcwd(), 
        'install',            
        'naviscope',   
        'share',         
        'naviscope',    
        'media',                  
        'img'                 
        )

        img_files = [
            os.path.join(IMG_DIR, filename)
            for filename in os.listdir(IMG_DIR)
            if filename.endswith(".jpg")
        ]

        self.imgList = { os.path.splitext(os.path.basename(file))[0] : self.load_and_resize(file, flipVertically=True) for file in img_files}


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


    def draw_texts( self ):

        self._text_elapsed_time = self.canvas.create_text(
            280, 
            100, 
            text="15:53", 
            fill="red", 
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
            text="CENTER", 
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

        
    def crop_from_center(self, frame, frame_width, frame_height, ZoomLevel ):
        
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

    def set_center_img( self, imgName = "" ):

        if self._center_image_name != imgName:

            image = self.imgList[imgName]

            if image is not None:
                self.canvas.itemconfig( self._canvas_frame, image=image )
                self._center_image_name = imgName


    def set_box( self, box, fillColor, outlineColor ):
        self.canvas.itemconfig(box, fill=fillColor, outline=outlineColor)

    def format_time(self, delta):
        total_seconds = int(delta.total_seconds())
        minutes, seconds = divmod(total_seconds, 60)
        return f"{minutes:02}:{seconds:02}"


    def render_elapsed( self ):

        self._playtimeLeft = self._node._playtimeLeft

        if self._playtime > 0:

            if self._playtimeLeft > 0:

                time_delta = datetime.timedelta(seconds=self._playtimeLeft)
                formatted_time = self.format_time(time_delta)
                
                if self._node is not None:
                    self._node.get_logger().info(f"{ formatted_time }")

                self.set_text( self._text_elapsed_time, f"{ formatted_time }" )
   

    def render_directional_arrow(self, currentAngle = 90, arrowColorFill = "black", arrowColorOutline = "white" ):

        self.canvas.delete(self.tagArrow)

        pivot_x = self.screen_center_x + self.arrow_radius * math.cos(math.radians(currentAngle))
        pivot_y = self.screen_center_y + self.arrow_radius * math.sin(math.radians(currentAngle))

        offset_length = 15 

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
        angle_rad = math.radians(currentAngle - 90)

        pivot_x = sum(points[::2]) / len(points[::2])
        pivot_y = sum(points[1::2]) / len(points[1::2])

        for i in range(0, len(points), 2):
            x, y = points[i] - pivot_x, points[i+1] - pivot_y
            points[i] = pivot_x + (x * math.cos(angle_rad) - y * math.sin(angle_rad))
            points[i+1] = pivot_y + (x * math.sin(angle_rad) + y * math.cos(angle_rad))

        self.canvas.create_polygon(
            points, 
            outline=arrowColorOutline, 
            fill=arrowColorFill,
            tags=self.tagArrow
        )


    def render_orientation(self, arrow_color = "white" ):
  
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
                
            self.render_directional_arrow( current_angle, arrow_color )


    def render_direction( self, droneDirection ):

        if droneDirection is not None:

            if droneDirection == DIRECTION_STATE.FORWARD.value:  
                self.set_box(self._box_direction, "green", "white")

            elif droneDirection == DIRECTION_STATE.BACKWARD.value:
                self.set_box(self._box_direction, "red", "white")

            else :
                self.set_box(self._box_direction, "black", "white")



    def OnGstSample( self, frame, frameSize ):
        
        frameWidth, frameHeight = frameSize[0], frameSize[1]

        cropFrame = self.crop_from_center(frame, frameWidth, frameHeight, self.ZoomLevel)
        resized_frame = cv2.resize(cropFrame, (self.videoWidth, self.videoHeight))
        color_conv = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(color_conv)
                    
        self._last_frame = ImageTk.PhotoImage(img)
        self._frame, self._last_frame = self._last_frame, self._frame

        self._frame_has_been_updated = True
                

    def OnCVBridgeFrame( self, frame ):
        
        with self._lock:

            current_frame = frame

            resized_frame = cv2.resize( current_frame, ( self.winfo_screenwidth(), self.winfo_screenheight() ))
            color_conv = cv2.cvtColor( resized_frame  , cv2.COLOR_BGR2RGB)
            
            img = Image.fromarray( color_conv )

            self._frame = ImageTk.PhotoImage( img )
            self._frame_has_been_updated = True
            

    def renderVideoFrame( self ):
    
        if self._node._audioManager is not None:
                
            if self._node._audioManager.tutorial_index <= 2:

                self.set_center_img( "bountyAtSea" )
                self.set_text( self._text_img, "Bounty" )

            elif self._node._audioManager.tutorial_index > 2 and self._node._audioManager.tutorial_index <= 6:

                self.set_center_img( "naviscopeSketch" )
                self.clear_img_text()

            else:

                if self._frame is not None:
            
                    if self._frame_has_been_updated is True:
                
                        if self.last_frame != self._frame:
                            self.last_frame = self._frame   

                    self.clear_img_text()
                    self.set_center_img( self._frame )    
            
            self._blackScreen = False

            self._frame_has_been_updated = False


    def clear_hud_texts( self ):

        if self.hudTexts is not None:
            for text in self.hudTexts: 
                self.canvas.itemconfig(text, text="")

    def clear_img_text( self ):

        if self._text_img is not None:
            self.canvas.itemconfig(self._text_img, text="")
            self._center_image_name = ""


    def renderBlackScreen( self ):

        if self._blackScreen is False: 

            self.canvas.update_idletasks()

            self.set_center_img( "pirateHead" )
      
            self.clear_hud_texts()
 
            self.canvas.delete(self.tagArrow)

            self._blackScreen = True    

      
    def renderName( self ):

        if self._nameIsVisible is False:

            droneName = self._node.droneName

            if droneName is not None:
          
                self.set_text( self._text_name, droneName )
                self._nameIsVisible = True


    def renderHUD( self ):
        
        self.updateStream()
        
        if self._hud_is_paused is False: 

            self.render_elapsed()
            self.render_orientation(self.arrowColor_base)
            #self.render_direction(DIRECTION_STATE.STOP.value)


    def updateStream(self):
            
        if self._process is not None: 

            if not self._frame_queue.empty():
                        
                frame_data = self._frame_queue.get()
                self.OnGstSample( frame_data["frame"], frame_data["size"])

        self.renderVideoFrame()
        


    def updateHud(self):
    
        if self._node is not None: 

            with self._lock:

                gameplayEnable = self._node.isGamePlayEnable

                if gameplayEnable is True:
                    self.renderHUD()

                else:

                    self.renderBlackScreen()

        self.after(self._loop_hud, self.updateHud)


    def _stop(self):
        
        #sleep(2)
        if self._node is not None:

            try:

                self._kill_rosNode() 
                
                if self._thread is not None: 
                    self._thread.join()
                    
            except Exception as e:
                traceback.print_exc()
        
        self.destroy()

        self._kill_videostream()


    def _action_on_shutdown(self, value):
        self._stop()


    def _closing_from_gui(self, event):
        self._stop()


    def _init_components( self):

        self._start_controller_node()
        self._start_videostream()


    def _start_controller_node(self):

        self._thread = Thread( target=self._rosNode )
        self._thread.daemon = True
        self._thread.start()

    def _rosNode(self):

        if self._node is None:
            self._node = Controller( Master=self ) 

        try:

            rclpy.spin( self._node )

        except Exception as e:
            traceback.print_exc()


    
    def _kill_rosNode(self):

        if self._node is not None:
            self._node.destroy_node()

        rclpy.shutdown()


    def _start_videostream(self):
        # Créez le processus en utilisant la méthode _stream_process comme cible.
        self._process = Process(target=self._stream_process, args=(self._frame_queue, self._frame_stop_event ))
        self._process.daemon = True
        self._process.start()


    def _stream_process(self, queue, event ):
        
        try:
            
            videostream = VideoStream(queue)
            videostream.start()

            while not self._frame_stop_event.is_set():
                sleep(0.1)  # Sleep for a short while

            videostream.stop()

        except Exception as e:
            traceback.print_exc()


    def _kill_videostream( self ):

        self._frame_stop_event.set()

        self._process.terminate()
        self._process.join()

        

        
def main(args=None):

    rclpy.init(args=args)

    app = Display()

    try:

        app._start()

    except Exception as e:
        traceback.print_exc()

    finally:

        app._stop()
        

if __name__ == '__main__':
    main()

    