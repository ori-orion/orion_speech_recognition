#!/usr/bin/env python3
import time
import speech_recognition as sr

import vosk

r = sr.Recognizer()

with sr.Microphone() as source:
    while True:
        try:
            r.adjust_for_ambient_noise(source, duration=0.2)
            print("Started recording")
            audio = r.listen(source)

            res = r.recognize_google(audio)
            print(f"Google says: {res}")
            with open("results_google.txt", "a") as f:
                f.write(res + "\n")
            #
            # res = r.recognize_sphinx(audio)
            # print(f"Sphinx says: {res}")
            # with open("results_sphinx.txt", "a") as f:
            #     f.write(res + "\n")
        except KeyboardInterrupt:
            print('\nDone')
            break
        except Exception as e:
            print(e)

