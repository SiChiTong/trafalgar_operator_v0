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

STANDARD_VOICE_ENDED = pygame.USEREVENT + 1
TUTORIAL_VOICE_ENDED = pygame.USEREVENT + 1

class AudioManager(object):

    def __init__(self, args=None ):
        
        super().__init__()

        self._mixer = None
        self.IsAdventurePlaying = False
        self.IsIdlePlaying = False

        self._music_playlist = []
        self._sfx_playlist = []

        self._voices_playlist = {
            AVAILABLE_LANG.FR.value : {}
        }

        self._lang = AVAILABLE_LANG.FR.value
        self._voice_is_playing = False

        self._volume_levels = {
            "music" : 0.3,
            "sfx" : 0.8,
            "voice" : 0.5
        }

        self.tutorial_index = 0
        self.tutorialIsComplete = False

        self.unlock_direction = False
        self.unlock_orientation = False
     
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

                    if self._mixer and self._mixer.get_busy():
                        self._mixer.set_volume(0.1)

                    self._voice_is_playing = True

                    voiceClip.play()

                    pygame.time.set_timer( TUTORIAL_VOICE_ENDED if tutorialEvent is True else STANDARD_VOICE_ENDED, int( voiceClip.get_length() * 1000 + delay ) )
     

    def reset_tutorial( self ):

        self.tutorial_index = 0
        self.tutorialIsComplete = False
        self.unlock_direction = False
        self.unlock_orientation = False

    
    def follow_tutorial(self, droneIndex = 0 ):

        if self.tutorialIsComplete is True:
            return
        
        tutorial_steps = [
        {"voice": f"drone_{droneIndex}", "delay": 500, "condition": lambda: True},#0
        {"voice": "cmd_introduction", "delay": 2000, "condition": lambda: True},#1

        {"voice": "hist_bounty", "delay": 1000, "condition": lambda: True},#2
        {"voice": "hist_breadfruit", "delay": 1000, "condition": lambda: True},#3
        {"voice": "hist_sugarPlantation","delay": 1500,  "condition": lambda: True},#4
        {"voice": "hist_mutiny", "delay": 2000, "condition": lambda: True},#5

        {"voice": "cmd_direction", "delay": 2000,  "condition": lambda: True},#6
        {"voice": "cmd_orientation", "delay": 500,  "condition": lambda: self.unlock_direction},#7
        {"voice": "cmd_invert",  "delay": 1000, "condition": lambda: self.unlock_orientation},#8
        {"voice": "cmd_spyglass", "delay": 500,  "condition": lambda: True},#9 display_frame
        {"voice": "cmd_cam", "delay": 500,  "condition": lambda: True},#10
        {"voice": "cmd_end", "delay": 500,  "condition": lambda: True}#11
        ]

        if self._voice_is_playing is False:

            if self.tutorial_index > len( tutorial_steps ) - 1 :
                self.tutorialIsComplete = True
                return

            step = tutorial_steps[ self.tutorial_index ]

            if step["condition"]():
                self.play_voice(voice=step["voice"], delay=step["delay"], tutorialEvent=True)



    def abort_tutorial( self ):

        self.tutorial_index = 7
        self.tutorialIsComplete = True

            
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

        if self._mixer and self._mixer.get_busy():

            self.IsAdventurePlaying = False
            self.IsIdlePlaying = False

            self._mixer.stop()

    def reset_music_volume( self ):

        self._voice_is_playing = False   

        if self._mixer and self._mixer.get_busy():    
            self._mixer.set_volume(self._volume_levels[ "music" ] )


    def loop( self ):

        for event in pygame.event.get():

            if event.type == TUTORIAL_VOICE_ENDED:
                self.reset_music_volume()
                pygame.time.set_timer(event.type, 0)
                self.tutorial_index += 1

            elif STANDARD_VOICE_ENDED:
                self.reset_music_volume()
                pygame.time.set_timer(event.type, 0)


    def _disable( self ):
        
        if self._mixer is not None:
            self._mixer.music.stop()
            self._mixer.quit()
                 
                 
