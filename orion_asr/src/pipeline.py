#!/usr/bin/env python3

from vosk import Model, KaldiRecognizer, SetLogLevel
from denoise import apply_denoise
from zipfile import ZipFile
#from pydub import AudioSegment

import speech_recognition as sr
import sys
import os
import wave

from constants import ROOT_DIR, DATA_DIR

MODEL_LIST = ["google","vosk","whisper"]
TMP_DIR = os.path.join(ROOT_DIR,"tmp")

class Pipeline:
    def __init__(self, filename, **kwargs):
        self.AUDIO_FILE = os.path.join(TMP_DIR, filename+".wav")
        self.filename = filename
        self.model = []
        self.denoise = kwargs.get("denoise",1)

        for modelname in MODEL_LIST:
            if kwargs.get(modelname,0) == 1:
                self.model.append(modelname)
        
        if not self.model:
            self.model = MODEL_LIST

    def transcribe_auto(self):
        if "google" in self.model:
            result_google = self.transcribe_google()
            print(result_google)
            self.text_commit("google",result_google)

        if "vosk" in self.model:
            result_vosk = self.transcribe_vosk()
            print(result_vosk)
            self.text_commit("vosk",result_vosk)

        if "whisper" in self.model:
            # Implement with Whisper model transcriber code
            '''
            result_whisper = self.transcribe_whisper()
            print(result_whisper)
            self.text_commit("whisper", result_whisper)
            '''

    def transcribe_google(self):
        r = sr.Recognizer()
        with sr.AudioFile(self.AUDIO_FILE) as source:
            audio = r.record(source)
        return(r.recognize_google(audio))

    def transcribe_vosk(self):
        SetLogLevel(0)
        DEFAULT_MODEL_PATH = os.path.join(DATA_DIR, "vosk-model-small-en-us-0.15")

        if not os.path.exists(DEFAULT_MODEL_PATH):
            if os.path.exists(DEFAULT_MODEL_PATH+".zip"):
                print(f"Unpacking VOSK model into {DEFAULT_MODEL_PATH}")
                with ZipFile(DEFAULT_MODEL_PATH+".zip","r") as f:
                    f.extractall(os.path.join(DEFAULT_MODEL_PATH,".."))

            else:
                print("Please download the model from https://alphacephei.com/vosk/model")
                exit(1)

        wf = wave.open(self.AUDIO_FILE, "rb")
        if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getcomptype() != "NONE":
            print("Audio file must be in WAV format mono PCM.")
            exit(1)

        model = Model(DEFAULT_MODEL_PATH)
        rec = KaldiRecognizer(model, wf.getframerate())
        rec.SetWords(True)

        while True:
            data = wf.readframes(100)
            if len(data) == 0:
                break
            """
            if rec.AcceptWaveform(data):
                print(rec.Result())
            else:
                print(rec.PartialResult())
            """

        return(rec.FinalResult())
    
    def text_commit(self, model, result):
        RESULT_DIR = os.path.join(TMP_DIR,model)

        if not os.path.exists(RESULT_DIR):
            print(model + " result directory missing; creating...")
            os.mkdir(RESULT_DIR)
            print("=====DIRECTORY CREATED=====\n")

        TEXT_FILE = os.path.join(RESULT_DIR,self.filename+".txt")
        file = open(TEXT_FILE,'w')
        file.write(result)
        file.close()
        print("=====TEXT COMMITTED=====\n")
    

if __name__ == "__main__":
    test1 = Pipeline("orion_asr_src_examples_test",google=1,vosk=1)
    
    print(test1.transcribe_google(),"\n")
    print("============\n")
    print(test1.transcribe_vosk())
    test1.transcribe_auto()
    