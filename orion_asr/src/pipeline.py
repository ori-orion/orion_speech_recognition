#!/usr/bin/env python3

from vosk import Model, KaldiRecognizer, SetLogLevel
import speech_recognition as sr
import sys
import os
import wave

from constants import ROOT_DIR, DATA_DIR

class pipeline:
    def __init__(self, filename, model):
        self.model = [model]
        self.filename = filename

    def __init__(self, filename, **kwargs):
        self.filename = filename
        self.model = []
        if not kwargs:
            for modelname in kwargs.keys:
                if kwargs.get(modelname) == 1:
                    self.model.append(modelname)
        else:
            self.model = ["google","vosk"]

    def transcribe_auto(self):
        if "google" in self.model:
            result_google = self.transcribe_google()
            print(result_google)
            self.text_commit("google",result_google)

        if "vosk" in self.model:
            result_vosk = self.transcribe_vosk()
            print(result_vosk)
            self.text_commit("vosk",result_vosk)

    def transcribe_google(self):
        AUDIO_FILE = os.path.join(DATA_DIR, self.filename+".wav")

        r = sr.Recognizer()
        with sr.AudioFile(AUDIO_FILE) as source:
            audio = r.record(source)
        return(r.recognize_google(audio))

    def transcribe_vosk(self):
        SetLogLevel(0)
        DEFAULT_MODEL_PATH = os.path.join(DATA_DIR, "vosk-model-small-en-us-0.15")
        AUDIO_FILE = os.path.join(DATA_DIR, self.filename+".wav")

        if not os.path.exists(DEFAULT_MODEL_PATH):
            print("Please download the model from https://alphacephei.com/vosk/model")
            exit(1)

        wf = wave.open(AUDIO_FILE, "rb")
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
        RESULT_DIR = os.path.join(DATA_DIR,model)

        if not os.path.exists(RESULT_DIR):
            print(model+" result directory missing; creating...")
            os.mkdir(RESULT_DIR)
            print("=====DIRECTORY CREATED=====\n")

        TEXT_FILE = os.path.join(RESULT_DIR,self.filename+".txt")
        file = open(TEXT_FILE,'w')
        file.write(result)
        file.close()
        print("=====TEXT COMMITTED=====\n")
    

if __name__ == "__main__":
    test1 = pipeline("orion_asr_src_examples_test",google=1,vosk=1)
    
    print(test1.transcribe_google(),"\n")
    print(test1.transcribe_vosk())
    test1.transcribe_auto()