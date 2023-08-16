#!/usr/bin/env python
import traceback
import datetime
import rclpy
import cv2 # OpenCV library
import math
from threading import Thread,Lock
import customtkinter
from PIL import ImageTk, Image

customtkinter.set_appearance_mode("default")  # Modes: system (default), light, dark
customtkinter.set_default_color_theme("default")  # Themes: blue (default), dark-blue, green

from ..components.__videostream import VideoStream
from .__controller import Controller

from ..utils.__utils_objects import DIRECTION_STATE
class Display(customtkinter.CTk):

    APP_NAME = ""
    WIDTH = 480
    HEIGHT = 480

    def __init__(
            self
        ):
        
        super().__init__()
        
        self.EnableText = False
        self.ZoomLevel = 10

        self.arrowAngle = 90
        self.arrowColor = "white"

        self._droneName = ""
        
        self._nameIsVisible = False

        self._node_controller = None
        self._videostream = None

        self.title(Display.APP_NAME)

        self.geometry(str( self.winfo_screenwidth()  ) + "x" + str(self.winfo_screenheight() ))
        self.minsize(Display.WIDTH, Display.HEIGHT)

        self.attributes("-fullscreen", True) 

        self.canvaResolution = (self.winfo_screenwidth(), self.winfo_screenheight()) 

        self.bind("<Control-c>", self._closing_from_gui)

        self._frame = None
        self.last_frame = None
        self._is_frame_updated = False
        
        self._blackScreen = False 
        self._isGamePlayEnable = False
        self._playtime = 10*60
        self._playtimeLeft = 0

        self._loop_stream = 1
        self._loop_hud = 50

        self._textToDisplay = "GAME OVER"
        self.iddleLoop = False

        self._thread_controller = None
        self._lock_controller = Lock()

        self._thread_stream = None
        self._lock_lock = Lock()

        self._is_running = True

        self._text_name = None
        self._text_elapsed_time = None
        self._text_angle = None
        self._text_direction = None
        
        self._text_img = None
        self._text_center = None
        
        self.hudTexts = None
        self.hudBoxes = None

        self._center_image = None

        self._hud_is_paused = False

        self._initialize()


    @property
    def videoWidth(self):
        return 480

    @property
    def videoHeight(self):
        return 320
    
    @property
    def tagHud(self):
        return "hud"
    
    @property
    def tagVideo(self):
        return "video"
    
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
        return "white"
    
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
        return "180"
    
    def _initialize( self ):    

        self._start_rosNode()
        self._create_window()
        self.update()

    
    def _start( self ):
        self.mainloop()
    

    def _create_window( self ): 

        self.canvas = customtkinter.CTkCanvas(self, width=self.canvaResolution[0], height=self.canvaResolution[1])
        self.canvas.pack( side="top", fill="both", expand=True )
        
        self.draw_header_footer()
        self.draw_texts_boxes()
        self.drawBlackScreen()


    def draw_header_footer(self):
        # Header
        self.canvas.create_rectangle(0, 420, 480, 480, fill="black", tags="header")
        # Footer
        self.canvas.create_rectangle(0, 0, 480, 100, fill="black", tags="footer")


    def draw_texts_boxes( self ):

        self._text_name = self.canvas.create_text(
            50, 
            20, 
            text="", 
            fill="white", 
            font=("Arial", self.regularTextSize), 
            justify=customtkinter.CENTER,
            anchor="w", 
            angle = self.text_flip_method,
            tags=self.tagHud
        )

        self._text_elapsed_time = self.canvas.create_text(
            240, 
            460, 
            text="", 
            fill="white", 
            font=("Arial", 20, "bold"), 
            anchor="w", 
            angle = self.text_flip_method,
            tags=self.tagHud
        )

        self._text_steering = self.canvas.create_text(
            320, 
            30, 
            text="", 
            fill="white", 
            font=("Arial", self.regularTextSize), 
            justify=customtkinter.CENTER,
            anchor="w", 
            angle = self.text_flip_method,
            tags=self.tagHud
        )

        
        self._text_img = self.canvas.create_text(
            240, 
            30, 
            text="", 
            fill="white", 
            font=("Arial", self.regularTextSize), 
            justify=customtkinter.CENTER,
            anchor="w", 
            angle = self.text_flip_method,
            tags=self.tagHud
        )

        self._text_center = self.canvas.create_text(
            50, 
            430, 
            text="", 
            fill="white", 
            font=("Arial", self.regularTextSize), 
            justify=customtkinter.CENTER,
            anchor="w", 
            angle = self.text_flip_method,
            tags=self.tagHud
        )

        self.hudTexts = [
            self._text_name,
            self._text_elapsed_time,
            self._text_steering,
            self._text_img,
            self._text_center
        ]


    def draw_outlined_boxes( self ):
        self._box_names = self.canvas.create_rectangle(290, 20, 390, 40, fill="blue", outline="white", tags=self.tagHud)

        self.hudBoxes = [
            self._box_names
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


    def _rosVideoUpdate( self, frame, frameWidth, frameHeight ):

        if( frame is not None and self._isGamePlayEnable is True ):

            cropFrame = self.crop_from_center(frame, frameWidth, frameHeight, self.ZoomLevel )
            resized_frame = cv2.resize(  cropFrame , ( self.videoWidth, self.videoHeight ))
            color_conv = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)

            img = Image.fromarray(color_conv)
            img_canvas = ImageTk.PhotoImage(img)

            self._frame = img_canvas
            self._is_frame_updated = True
    

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

    def set_box( self, box, fillColor, outlineColor ):
        self.canvas.itemconfig(box, fill=fillColor, outline=outlineColor)

    def renderElapsedTime( self ):

        self._playtimeLeft = self._node_controller._playtimeLeft

        if self._playtime > 0:

            if self._playtimeLeft > 0:

                time_delta = datetime.timedelta(seconds=self._playtimeLeft)
                formatted_time = str(time_delta).split(".")[0]
                
                self.set_text( self._text_elapsed_time, f"Temps restant: { formatted_time }" )


    def render_directional_arrow(self):

        cx, cy = 240, 30  # Positioned near the top

        points = [
            cx, cy + 15,
            cx + 12, cy,
            cx + 3, cy,
            cx + 3, cy - 8,
            cx - 3, cy - 8,
            cx - 3, cy,
            cx - 12, cy
        ]

        pivot_x = sum(points[::2]) / len(points[::2])
        pivot_y = sum(points[1::2]) / len(points[1::2])

        angle_rad = math.radians(self._droneSteering + 90)

        for i in range(0, len(points), 2):
            x, y = points[i] - pivot_x, points[i+1] - pivot_y
            points[i] = pivot_x + (x * math.cos(angle_rad) - y * math.sin(angle_rad))
            points[i+1] = pivot_y + (x * math.sin(angle_rad) + y * math.cos(angle_rad))

        self.canvas.create_polygon(points, outline=self.arrow_color, fill=self.arrow_color, tags=self.tagArrow)


    def render_orientation(self):

        if self._node_controller is not None:
            
            current_angle = self._node_controller.droneSteering

            if current_angle is not None:

                update_orientation = ""

                if current_angle >= 80 and current_angle <= 100:
                    update_orientation = f"CENTRE: 0°"
                elif self.current_angle < 80:
                    update_orientation = f"GAUCHE: {current_angle - 90}°"
                else:
                    update_orientation = f"DROIT: {current_angle - 90}°"
        
                self.set_text( self._text_steering, update_orientation )
                self.render_directional_arrow()


    def render_direction( self ):

        if self._node_controller is not None: 

            droneDirection = self._node_controller.droneDirection

            if droneDirection is not None:

                if droneDirection == DIRECTION_STATE.FORWARD.value:  
                    self.set_box(self._box_direction, "green", "white")

                elif droneDirection == DIRECTION_STATE.BACKWARD.value:
                    self.set_box(self._box_direction, "red", "white")

                else :
                    self.set_box(self._box_direction, "black", "white")

    def renderVideoFrame( self ):

        if self._is_frame_updated is True:

            if self.last_frame != self._frame:

                self.last_image = self._frame
                self.canvas.delete(self.tagVideo)
                self.canvas.create_image(0, 0, anchor="nw", image=self.last_image, tags="video")

                self._blackScreen = False


    def clear_hud_texts( self ):

        for text in self.hudTexts: 
            self.canvas.itemconfig(text, text="")

    def clear_hud_boxes(self):
        
        for box in self.hudBoxes:
            self.canvas.itemconfig(box, fill="black", outline="black")
        
    def renderBlackScreen( self ):

        if self._blackScreen is False: 

            self.canvas.update_idletasks()

            self.canvas.delete("video")

            self.clear_hud_texts()
            self.clear_hud_boxes()

            self.canvas.create_rectangle(0, 0, self.canvas.winfo_width(), self.canvas.winfo_height(), fill="black")
            
            if self.EnableText is True:
                self.render_text_at_center()

            self._blackScreen = True    
      

    def renderHUD( self ):

        if self._nameIsVisible is False: 

            if self._node_controller is not None:

                droneName = self._node_controller.droneName

                if droneName is not None:
                    self.set_text( self._text_name, self._droneName )
                    self._nameIsVisible = True

        self.renderElapsedTime()

        if self._hud_is_paused is False: 

            self.render_orientation()
            self.render_direction()


    def updateStream(self):

        if self._isGamePlayEnable is True: 
            self.renderVideoFrame()


    def update(self):
    
        if self._isGamePlayEnable is True:

            if self.tutorialComplete is False:
                self.tutorialUpdate()

            self.arrow_color = self.arrowColor_base
            self.renderHUD()

        else:

            self.arrow_color = self.arrowColor_hide
            self.renderBlackScreen()

        self.render_directional_arrow()
        self._is_frame_updated = False

        self.after(self._loop_hud, self.updateHud)
        self.after(self._loop_stream, self.updateStream)


    def _stop(self):
        
        #sleep(2)
        if self._node_controller is not None:

            with self._lock_controller:
                try:

                    self._kill_rosNode() 
                    
                    if self._thread_controller is not None: 
                        self._thread_controller.join()
                    
                except Exception as e:
                    traceback.print_exc()
        
        if self._videostream is not None:

            with self._lock_videostream:

                try:

                    self._videostream.exit() 
                    
                    if self._thread_videostream is not None: 
                        self._thread_videostream.join()
                    
                except Exception as e:
                    traceback.print_exc()

        self.destroy()


    def _action_on_shutdown(self, value):
        self._stop()


    def _closing_from_gui(self, event):
        self._stop()


    def _start_rosNode( self):

        self._thread_controller = Thread( target=self._rosNode_thread)
        self._thread_controller.daemon = True
        self._thread_controller.start()

    def _rosNode_thread(self):

        if self._node_controller is None:
            self._node_controller = Controller() 
        
        self._is_running = True
        
        with self._lock:

            try:

                rclpy.spin( self._node_controller )

            except Exception as e:
                traceback.print_exc()

    def _kill_rosNode(self):

        if self._node_controller is not None:
            self._node_controller.destroy_node()
            rclpy.shutdown()


    def _start_stream_pipeline( self):

        self._thread_videostream = Thread( target=self._videostream_thread )
        self._thread_videostream.daemon = True
        self._thread_videostream.start()   

    def _videostream_thread(self):

        if self._videostream is None:
            self._videostream = VideoStream( self ) 
        
        self._is_running = True
        
        with self._lock:

            try:
                self._videostream.start()

            except Exception as e:
                traceback.print_exc()


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

    