import vosk
import json
import time
import queue
import sounddevice
import sys
import os
import pyaudio
import speech_recognition
import constants
import denoise

vosk.SetLogLevel(-1)

DEFAULT_MODEL_PATH = os.path.join(constants.ROOT_DIR, "data", "vosk-model")