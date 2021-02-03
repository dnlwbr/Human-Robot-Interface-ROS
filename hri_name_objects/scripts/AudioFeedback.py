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
