import os
import pygame
from gtts import gTTS
import pyttsx3


class AudioFeedback:

    def __init__(self):
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)    # setting up new voice rate
        self.engine.setProperty('volume', 1.0)       # setting up volume level  between 0 and 1

    def __del__(self):
        self.engine.stop()

    def say(self, text):
        self.engine.say(text)
        self.engine.runAndWait()


class AudioFeedbackGTTS:

    def __init__(self, audio_folder_path="."):
        self.audio_folder_path = audio_folder_path
        self.pg = pygame
        self.pg.mixer.pre_init(44100, -16, 2, 4096)
        self.pg.mixer.init(44100, -16, 2, 4096)
        self.pg.init()
        self.language = "en"

    def __del__(self):
        self.pg.mixer.quit()

    def play(self, mp3_filename):
        self.pg.mixer.music.load(self.audio_folder_path + mp3_filename)
        self.pg.mixer.music.play(0)

    def stop(self):
        self.pg.mixer.stop()

    def say(self, text):
        file_name = "voice.mp3"
        tts = gTTS(text, lang=self.language)
        tts.save(self.audio_folder_path + file_name)
        self.play(file_name)
        while self.pg.mixer.get_busy():
            print(self.pg.mixer.get_busy())
            pygame.time.wait(100)
        os.remove(self.audio_folder_path + file_name)

