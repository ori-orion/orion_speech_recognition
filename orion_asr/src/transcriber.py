#!/usr/bin/env python3

from vosk import Model, KaldiRecognizer, SetLogLevel
import sys
import os
import wave

from constants import ROOT_DIR

SetLogLevel(0)

DEFAULT_MODEL_PATH = os.path.join(ROOT_DIR, "data", "vosk-model-small-en-us-0.15")

def transcribe():
    if not os.path.exists(DEFAULT_MODEL_PATH):
        print("Please download the model from https://alphacephei.com/vosk/model")
        exit(1)

    wf = wave.open(sys.argv[1], "rb")
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

    print(rec.FinalResult())

if __name__ == "__main__":
    transcribe()
