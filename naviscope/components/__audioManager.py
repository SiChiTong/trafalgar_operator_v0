#!/usr/bin/env python3
########################################################################
# Filename    : __audioManager.py
# Description : audio manager
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import os
import pygame
import numpy as np
import itertools

from pygame import mixer

from ..utils.__utils_objects import AVAILABLE_LANG

GAME_OVER_VOICE_ENDED = pygame.USEREVENT + 1
STANDARD_VOICE_ENDED = pygame.USEREVENT + 2
HIERARCHY_VOICE_ENDED = pygame.USEREVENT + 3
HIST_DELAY_ENDED = pygame.USEREVENT + 4

class AudioManager(object):

    def __init__(self, args=None ):
        
        super().__init__()

        self._mixer = None
        
        self.tutorialTime = 100

        self.IsAdventurePlaying = False
        self.IsIdlePlaying = False

        self.displayCameraFeed = False
        self.imgToDisplay = ""
        self.imgIsFromAVideo = False
        
        self.userlock_direction = True
        self.userlock_orientation = True

        self.shipIsIddling = False
        
        self._music_playlist = []
        self._sfx_playlist = []

        self._voices_playlist = {
            AVAILABLE_LANG.FR.value : {}
        }

        self.voices_cmd = None
        self.voices_history = None
        self.voices_hierarchy = None

        self._lang = AVAILABLE_LANG.FR.value
        self._voice_is_playing = False

        self._volume_levels = {
            "music" : 0.3,
            "sfx" : 0.6,
            "voice" : 0.5
        }

        self.voice_index = 0

        self.unlock_direction = False
        self.unlock_orientation = False
        self.unlock_hist = False

        self.history_delay = 60*1000
        self.histDelayIsRunning = False

    @property
    def FullHudIndexReached(self):
        return self.voice_index >= 5
    
    @property
    def HistIndexReached(self):
        return self.voice_index >= len( self.voices_cmd ) 
    
    @property
    def readAllVoices(self):
        return self.voice_index >= len( self.voices_hierarchy )


    @property
    def canPlayHistory(self):
        return self.shipIsIddling and self.unlock_hist
    
    def _enable( self, droneIndex = 0 ):
        
        pygame.init()

        self._mixer = mixer.music

        self.MUSIC_DIR = os.path.join(
        os.getcwd(), 
        'install',  
        'naviscope', 
        'share',   
        'naviscope',  
        'media',    
        'audio',    
        'music'     
        )

        self.SFX_DIR = os.path.join(
        os.getcwd(), 
        'install',  
        'naviscope', 
        'share',      
        'naviscope',  
        'media',     
        'audio',   
        'sfx'                
        )

        self.VOICES_DIR = os.path.join(
        os.getcwd(), 
        'install',          
        'naviscope', 
        'share',           
        'naviscope',  
        'media',           
        'audio',              
        'voices'                 
        )

        self.reset_tutorial()

        self._load_music()
        self._load_sfx()
        self._load_voices()

        self._set_voices_hierarchy( droneIndex )


    def _load_music( self ):
        
        music_files = [
            os.path.join(self.MUSIC_DIR, filename)
            for filename in os.listdir(self.MUSIC_DIR)
            if filename.endswith(".wav")
        ]
        
        self._music_playlist = {os.path.splitext(os.path.basename(file))[0]: file for file in music_files}


    def _load_sfx( self ):

        sfx_files = [
            os.path.join(self.SFX_DIR, filename)
            for filename in os.listdir(self.SFX_DIR)
            if filename.endswith(".wav")
        ]

        self._sfx_playlist = {os.path.splitext(os.path.basename(file))[0]: file for file in sfx_files}



    def _load_voices( self ):

        for lang_dir in self._voices_playlist.keys():
            
            dir_path = os.path.join(self.VOICES_DIR, lang_dir ) 

            voices_files = [
                os.path.join(dir_path, filename)
                for filename in os.listdir(dir_path)
                if filename.endswith(".wav")
            ]

            self._voices_playlist[lang_dir] = {os.path.splitext(os.path.basename(file))[0]: file for file in voices_files}
    
    def _set_voices_hierarchy( self, droneIndex = 0 ):

        self.voices_cmd = [

        {"voice": f"drone_{droneIndex}", "displayCamera" : False, "lockDirection" : True, "lockOrientation" : True, "img" : "pirateHead", "isAVideo" : False, "delay": 500, "condition": lambda: True},#0
        
        {"voice": "cmd_introduction", "displayCamera" : False, "lockDirection" : True,"lockOrientation" : True, "img" : "cmd_introduction","isAVideo" : False,  "delay": 2000, "condition": lambda: True},#1

        {"voice": "cmd_direction", "displayCamera" : False,"lockDirection" : False, "lockOrientation" : True,"img" : "cmd_buttonClick","isAVideo" : False,  "delay": 2000,  "condition": lambda: True},#2
        {"voice": "cmd_orientation", "displayCamera" : False,"lockDirection" : False, "lockOrientation" : False,"img" : "cmd_orientation","isAVideo" : False,  "delay": 500,  "condition": lambda: self.unlock_direction},#3
        {"voice": "cmd_invert", "displayCamera" : False,"lockDirection" : False, "lockOrientation" : False,"img" : "cmd_orientation","isAVideo" : False,  "delay": 1000, "condition": lambda: self.unlock_orientation},#4
        {"voice": "cmd_spyglass", "displayCamera" : True,"lockDirection" : False, "lockOrientation" : False,"img" : "cmd_cam","isAVideo" : False,  "delay": 500,  "condition": lambda: True},#5 > display frame
        {"voice": "cmd_cam", "displayCamera" : False,"lockDirection" : False, "lockOrientation" : False,"img" : "cmd_cam","isAVideo" : False,  "delay": 500,  "condition": lambda: True},#6
        {"voice": "cmd_end", "displayCamera" : True, "lockDirection" : False,"lockOrientation" : False, "img" : None,"isAVideo" : False,  "delay": 1000,  "condition": lambda: True},#7

        ]
        
        self.voices_history = [

        {"voice": "hist_handcraft", "displayCamera" : False, "lockDirection" : False,"lockOrientation" : False,"img" : "hist_handcraft", "isAVideo" : True,  "delay": 1000, "condition": lambda: self.canPlayHistory },#8
        {"voice": "hist_bounty", "displayCamera" : False, "lockDirection" : False,"lockOrientation" : False,"img" : "hist_bountyAtSea", "isAVideo" : True,  "delay": 1000, "condition": lambda: self.canPlayHistory },#9 
        {"voice": "hist_breadfruit", "displayCamera" : False, "lockDirection" : False,"lockOrientation" : False,"img" : "hist_breadfruit","isAVideo" : True, "delay": 1000, "condition": lambda: self.canPlayHistory },#10
        {"voice": "hist_sugarFarm", "displayCamera" : False, "lockDirection" : False,"lockOrientation" : False, "img" : "hist_sugarFarm","isAVideo" : True, "delay": 1000,  "condition": lambda: self.canPlayHistory },#11
        {"voice": "hist_mutiny", "displayCamera" : False, "lockDirection" : False,"lockOrientation" : False, "img" : "hist_mutiny", "delay": 1000,"isAVideo" : True, "condition": lambda: self.canPlayHistory },#12 > displayframe
        {"voice": "hist_pitcairn", "displayCamera" : False, "lockDirection" : False,"lockOrientation" : False, "img" : "hist_pitcairn", "delay": 1000,"isAVideo" : True, "condition": lambda: self.canPlayHistory }#13 > displayframe

        ]
    
        self.voices_hierarchy = list(itertools.chain(self.voices_cmd, self.voices_history)) 
    
    def play_sfx( self, sfx = None ): 
        
        if self._mixer and sfx in self._sfx_playlist:
            
            sfx_path = self._sfx_playlist[sfx]
            
            self.sound_sfx = mixer.Sound( sfx_path )

            self.sound_sfx.set_volume( self._volume_levels[ "sfx" ] )
            self.sound_sfx.play()

            


    def play_music( self, music = None ): 

        if self._mixer and music in self._music_playlist:

            if self._mixer.get_busy():
                self._mixer.stop()
            
            musicToPlay = self._music_playlist[music]
            self._mixer.load(musicToPlay)
            
            self._mixer.set_volume(self._volume_levels[ "music" ])

            self._mixer.play(loops=-1)


    def play_voice( self, voice = None, delay=500, event=None ): 

        if self._voice_is_playing is False:
            
            if len(self._voices_playlist[self._lang]) > 0: 

                if self._mixer and voice in self._voices_playlist[self._lang]:

                    voice_path = self._voices_playlist[self._lang][voice]

                    self.sound_voice = mixer.Sound( voice_path )

                    self.sound_voice.set_volume( self._volume_levels[ "voice" ] )

                    if self._mixer:
                        self._mixer.set_volume(0.1)

                    self._voice_is_playing = True

                    self.sound_voice.play()
                    #add a pause function at some point 

                    if event is not None :
                        pygame.time.set_timer( event, int( self.sound_voice.get_length() * 1000 + delay ) )
     
    
    def next_voice( self ):

        if self.voice_index > len( self.voices_hierarchy ) - 1 :
            self.displayCameraFeed = True
            return
        
        if self._voice_is_playing is False:

            step = self.voices_hierarchy[ self.voice_index ]

            if step["condition"]():

                self.displayCameraFeed = step["displayCamera"]
                self.imgToDisplay = step["img"]
                self.imgIsFromAVideo = step["isAVideo"]

                self.userlock_direction = step["lockDirection"]
                self.userlock_orientation = step["lockOrientation"]

                self.play_voice(voice=step["voice"], delay=step["delay"], event = HIERARCHY_VOICE_ENDED)

                self.unlock_hist = False



    def get_media_to_display( self ):

        if self.displayCameraFeed is True:
            return (None, False)
         
        return (self.imgToDisplay, self.imgIsFromAVideo )
    
    
    

    def onGameOver( self ):
        self.play_voice(voice="game_over", delay=1000, event = GAME_OVER_VOICE_ENDED )
        self.reset_tutorial()
    
    def onHistoryReady( self ):
        self.play_voice(voice="hist_listen", delay=1000, event = STANDARD_VOICE_ENDED )

    def reset_tutorial( self ):

        self.voice_index = 0

        self.unlock_direction = False
        self.unlock_orientation = False
        self.unlock_hist = False

        self.displayCameraFeed = False

        self.userlock_direction = True
        self.userlock_orientation = True

    def abort_tutorial( self ):

        self.voice_index = len(self.voices_hierarchy) if self.voices_hierarchy is not None else 14
        self.displayCameraFeed = True

        self.unlock_direction = True
        self.unlock_orientation = True
        self.unlock_hist = True

        self.userlock_direction = False
        self.userlock_orientation = False

         
    def gameplayMusic( self, Enable, direction ): 

        if Enable is True:

            if direction != 0:

                if self.IsIdlePlaying is True:
                    self.stop_music()
                    self.IsIdlePlaying = False

                if self.IsAdventurePlaying is False:
                    self.IsAdventurePlaying = True
                    self.play_music("adventure")

            else:

                if self.IsAdventurePlaying is True:
                    self.stop_music()
                    self.IsAdventurePlaying = False

                if self.IsIdlePlaying is False:
                    self.IsIdlePlaying = True
                    self.play_music("idle")
        else:
            
            self.mute()
                    

    def stop_music( self ): 

        self.IsAdventurePlaying = False
        self.IsIdlePlaying = False

        if self._mixer and self._mixer.get_busy():
            self._mixer.stop()

    def stop_sfx( self ):

        if self.sound_sfx is None:
            return

        if self.sound_sfx.get_busy():
            self.sound_sfx.stop()

    def stop_voice( self ):

        if self.sound_voice is None:
            return

        if self.sound_voice.get_busy():
            self.sound_voice.stop()

    def mute( self ):

        self.stop_music()
        self.stop_sfx()
        self.stop_voice()

    def reset_music_volume( self ):

        self._voice_is_playing = False   

        if self._mixer:    
            self._mixer.set_volume(self._volume_levels[ "music" ] )

    def set_playtime( self, playtime = 10*60 ):

        self.history_delay = int( np.floor( (playtime - self.tutorialTime ) / ( len( self.voices_history ) +1 ) ) )
    
    def wait_before_next_play( self ):

        if self.voice_index > len( self.voices_hierarchy ):
            return 
        
        pygame.time.set_timer( HIST_DELAY_ENDED, int(self.history_delay * 1000 ) )


    def loop( self ):

        for event in pygame.event.get():

            if event.type == HIERARCHY_VOICE_ENDED:

                self._voice_is_playing = False   
                self.reset_music_volume()

                pygame.time.set_timer(event.type, 0)
                
                self.voice_index += 1

                if self.readAllVoices is False:

                    if self.HistIndexReached is True and self.histDelayIsRunning is False:
                        self.histDelayIsRunning = True
                        self.wait_before_next_play()


            elif event.type == STANDARD_VOICE_ENDED:
                
                self._voice_is_playing = False   
                self.reset_music_volume()

                pygame.time.set_timer(event.type, 0)

            elif event.type == GAME_OVER_VOICE_ENDED:
                
                self._voice_is_playing = False   
                self.reset_tutorial()
                pygame.time.set_timer(event.type, 0)

            elif event.type == HIST_DELAY_ENDED:
                
                self.unlock_hist = True
                self.histDelayIsRunning = False

                if self.shipIsIddling is False:
                    self.onHistoryReady()

                pygame.time.set_timer(event.type, 0)

        
        


    def _disable( self ):
        
        if self._mixer is not None:
            self._mixer.music.stop()
            self._mixer.quit()
                 
                 
