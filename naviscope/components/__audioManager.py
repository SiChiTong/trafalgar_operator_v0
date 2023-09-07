#!/usr/bin/env python3
########################################################################
# Filename    : __audioManager.py
# Description : audio manager
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import os
import pygame
from pygame import mixer

from ..utils.__utils_objects import AVAILABLE_LANG

GAME_OVER_VOICE_ENDED = pygame.USEREVENT + 1
STANDARD_VOICE_ENDED = pygame.USEREVENT + 1
TUTORIAL_VOICE_ENDED = pygame.USEREVENT + 1

class AudioManager(object):

    def __init__(self, args=None ):
        
        super().__init__()

        self._mixer = None
        self.IsAdventurePlaying = False
        self.IsIdlePlaying = False

        self.displayCameraFeed = False
        self.imgToDisplay = ""

        self.userlock_direction = True
        self.userlock_orientation = True
        
        self._music_playlist = []
        self._sfx_playlist = []

        self._voices_playlist = {
            AVAILABLE_LANG.FR.value : {}
        }

        self._lang = AVAILABLE_LANG.FR.value
        self._voice_is_playing = False

        self._volume_levels = {
            "music" : 0.3,
            "sfx" : 0.6,
            "voice" : 0.5
        }

        self.tutorial_index = 0

        self.FullHudIndexReached = False
        self.HistIndexReached = False

        self.tutorialIsComplete = False

        self.unlock_direction = False
        self.unlock_orientation = False
        self.unlock_hist = False

    @property
    def display_frame_index(self):
        return 9
    

    def _enable( self ):
        
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
            

    def play_sfx( self, sfx = None ): 
        
        if self._mixer and sfx in self._sfx_playlist:
            
            sfx_path = self._sfx_playlist[sfx]
            sfxClip = mixer.Sound( sfx_path )

            sfxClip.set_volume( self._volume_levels[ "sfx" ] )
            sfxClip.play()

            


    def play_music( self, music = None ): 

        if self._mixer and music in self._music_playlist:

            if self._mixer.get_busy():
                self._mixer.stop()
            
            musicToPlay = self._music_playlist[music]
            self._mixer.load(musicToPlay)
            
            self._mixer.set_volume(self._volume_levels[ "music" ])

            self._mixer.play(loops=-1)


    def play_voice( self, voice = None, delay=500, tutorialEvent = True ): 

        if self._voice_is_playing is False:
            
            if len(self._voices_playlist[self._lang]) > 0: 

                if self._mixer and voice in self._voices_playlist[self._lang]:

                    voice_path = self._voices_playlist[self._lang][voice]
                    voiceClip = mixer.Sound( voice_path )

                    voiceClip.set_volume( self._volume_levels[ "voice" ] )

                    if self._mixer:
                        self._mixer.set_volume(0.1)

                    self._voice_is_playing = True

                    voiceClip.play()
                    #add a pause function at some point 

                    pygame.time.set_timer( TUTORIAL_VOICE_ENDED if tutorialEvent is True else STANDARD_VOICE_ENDED, int( voiceClip.get_length() * 1000 + delay ) )
     
    
    def follow_tutorial(self, droneIndex = 0 ):

        if self.tutorialIsComplete is True:
            return
        
        tutorial_steps = [
        {"voice": f"drone_{droneIndex}", "displayCamera" : False, "lockDirection" : True, "lockOrientation" : True, "img" : "pirateHead", "isAVideo" : False, "delay": 500, "condition": lambda: True},#0
        {"voice": "cmd_introduction", "displayCamera" : False, "lockDirection" : True,"lockOrientation" : True, "img" : "cmd_introduction","isAVideo" : False,  "delay": 2000, "condition": lambda: True},#1

        {"voice": "cmd_direction", "displayCamera" : False,"lockDirection" : False, "lockOrientation" : True,"img" : "cmd_buttonClick","isAVideo" : False,  "delay": 2000,  "condition": lambda: True},#2
        {"voice": "cmd_orientation", "displayCamera" : False,"lockDirection" : False, "lockOrientation" : False,"img" : "cmd_orientation","isAVideo" : False,  "delay": 500,  "condition": lambda: self.unlock_direction},#3
        {"voice": "cmd_invert", "displayCamera" : False,"lockDirection" : False, "lockOrientation" : False,"img" : "cmd_orientation","isAVideo" : False,  "delay": 1000, "condition": lambda: self.unlock_orientation},#4
        {"voice": "cmd_spyglass", "displayCamera" : True,"lockDirection" : False, "lockOrientation" : False,"img" : "cmd_cam","isAVideo" : False,  "delay": 500,  "condition": lambda: True},#5 > display frame
        {"voice": "cmd_cam", "displayCamera" : False,"lockDirection" : False, "lockOrientation" : False,"img" : "cmd_cam","isAVideo" : False,  "delay": 500,  "condition": lambda: True},#6
        {"voice": "cmd_end", "displayCamera" : True, "lockDirection" : False,"lockOrientation" : False, "img" : None,"isAVideo" : False,  "delay": 30 * 1000,  "condition": lambda: True},#7

        {"voice": "hist_intro", "displayCamera" : True, "lockDirection" : False,"lockOrientation" : False, "img" : None,"isAVideo" : False,  "delay": 500,  "condition": lambda: True},#7
        
        {"voice": "hist_handcraft", "displayCamera" : False, "lockDirection" : False,"lockOrientation" : False,"img" : "hist_handcraft", "isAVideo" : True,  "delay": 1000, "condition": lambda: self.unlock_hist },#8
        {"voice": "hist_bounty", "displayCamera" : False, "lockDirection" : False,"lockOrientation" : False,"img" : "hist_bountyAtSea", "isAVideo" : True,  "delay": 1000, "condition": lambda: self.unlock_hist },#9 
        {"voice": "hist_breadfruit", "displayCamera" : False, "lockDirection" : False,"lockOrientation" : False,"img" : "hist_breadfruit","isAVideo" : True, "delay": 1000, "condition": lambda: self.unlock_hist },#10
        {"voice": "hist_sugarFarm", "displayCamera" : False, "lockDirection" : False,"lockOrientation" : False, "img" : "hist_sugarFarm","isAVideo" : True, "delay": 1000,  "condition": lambda: self.unlock_hist },#11
        {"voice": "hist_mutiny", "displayCamera" : False, "lockDirection" : False,"lockOrientation" : False, "img" : "hist_mutiny", "delay": 1000,"isAVideo" : True, "condition": lambda: self.unlock_hist },#12 > displayframe
        {"voice": "hist_pitcairn", "displayCamera" : False, "lockDirection" : False,"lockOrientation" : False, "img" : "hist_pitcairn", "delay": 1000,"isAVideo" : True, "condition": lambda: self.unlock_hist }#13 > displayframe

        ]

        if self._voice_is_playing is False:

            if self.tutorial_index > len( tutorial_steps ) - 1 :
                self.tutorialIsComplete = True
                self.displayCameraFeed = True
                return

            step = tutorial_steps[ self.tutorial_index ]

            if step["condition"]():
                self.displayCameraFeed = step["displayCamera"]
                self.imgToDisplay = step["img"]
                self.imgIsFromAVideo = step["isAVideo"]

                self.userlock_direction = step["lockDirection"]
                self.userlock_orientation = step["lockOrientation"]

                self.play_voice(voice=step["voice"], delay=step["delay"], tutorialEvent=True)


    def get_media_to_display( self ):

        if self.displayCameraFeed is True:
            return None
         
        if self.HistIndexReached is True and self.unlock_hist is False:
            return None
        
        return (self.imgToDisplay, self.imgIsFromAVideo )
    

    def onGameOver( self ):
        self.play_voice(voice="game_over", delay=1000, tutorialEvent=GAME_OVER_VOICE_ENDED )
        

    def reset_tutorial( self ):

        self.tutorial_index = 0
        self.tutorialIsComplete = False

        self.unlock_direction = False
        self.unlock_orientation = False
        self.unlock_hist = False

        self.displayCameraFeed = False

        self.userlock_direction = True
        self.userlock_orientation = True

        self.FullHudIndexReached = False
        self.HistIndexReached = False

    def abort_tutorial( self ):

        self.tutorial_index = 13
        self.tutorialIsComplete = True
        self.displayCameraFeed = True

        self.unlock_direction = True
        self.unlock_orientation = True
        self.unlock_hist = True

        self.userlock_direction = False
        self.userlock_orientation = False

        self.FullHudIndexReached = True
        self.HistIndexReached = True
            
            
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
            
            self.stop_music()
                    

    def stop_music( self ): 

        self.IsAdventurePlaying = False
        self.IsIdlePlaying = False

        if self._mixer and self._mixer.get_busy():
            self._mixer.stop()

    def reset_music_volume( self ):

        self._voice_is_playing = False   

        if self._mixer:    
            self._mixer.set_volume(self._volume_levels[ "music" ] )


    def loop( self ):

        for event in pygame.event.get():

            if event.type == TUTORIAL_VOICE_ENDED:
                self.reset_music_volume()
                pygame.time.set_timer(event.type, 0)
                self.tutorial_index += 1
            
                self.FullHudIndexReached = self.tutorial_index >= 5
                self.HistIndexReached = self.tutorial_index >= 8

            elif STANDARD_VOICE_ENDED:
                self.reset_music_volume()
                pygame.time.set_timer(event.type, 0)

            elif GAME_OVER_VOICE_ENDED:
                self.reset_tutorial()
                pygame.time.set_timer(event.type, 0)

            


    def _disable( self ):
        
        if self._mixer is not None:
            self._mixer.music.stop()
            self._mixer.quit()
                 
                 
