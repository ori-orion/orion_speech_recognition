#!/usr/bin/env python3

import vosk  # has to be imported at the top
import json
import threading
import time
from zipfile import ZipFile

import argparse
import queue
import sounddevice as sd
import sys
import os
import pyaudio

from speech_recognition import AudioData, Recognizer
import speech_recognition as sr

from constants import ROOT_DIR
from denoise import apply_denoise

vosk.SetLogLevel(0)

DEFAULT_MODEL_PATH = os.path.join(ROOT_DIR, "data", "vosk-model-small-en-us-0.15")
AUDIO_SAVE_PATH = os.path.join(ROOT_DIR, "tmp")


def default_callback(frames, results: dict):
    """
    :param frames:
    :param results:
    :return: boolean of whether recording should terminate
    """
    for name, text in results.items():
        print(f"{name} says: ", text)
    return False


class Recorder:
    def __init__(self, model_path=DEFAULT_MODEL_PATH, filename=None, device=None, samplerate=None, blocksize=8000,
                 audio_path=AUDIO_SAVE_PATH, save_audio=False, keep_frame_duration=5, denoise=True,
                 noise_thresh=0.0, volume_amp=10, **kwargs):
        if not os.path.exists(model_path):
            if os.path.exists(model_path + ".zip"):
                print(f"Unpacking vosk model into {model_path}")
                with ZipFile(model_path + ".zip", "r") as f:
                    f.extractall(os.path.join(model_path, ".."))
            else:
                raise Exception(f"""Please download a model for your language from https://alphacephei.com/vosk/models
                and unpack at '{model_path}'""")
        if samplerate is None:
            device_info = sd.query_devices(device, 'input')
            # soundfile expects an int, sounddevice provides a float:
            samplerate = int(device_info['default_samplerate'])

        self.samplerate = samplerate
        self.device = device
        self.blocksize = blocksize

        self.audio_q = queue.Queue()
        self.outputs_q = queue.Queue(maxsize=100)

        self.sample_width = pyaudio.get_sample_size(pyaudio.paInt16)  # size of each sample

        self.model = vosk.Model(model_path)
        self.r = Recognizer()
        with sr.Microphone() as source:
            self.r.adjust_for_ambient_noise(source, duration=3)
        self.dump_fn = open(filename, "wb") if filename else None
        # keep frames for a minimum duration of time
        self.keep_frames = keep_frame_duration * samplerate // blocksize

        if not os.path.exists(audio_path):
            os.makedirs(audio_path)
        self.audio_path = audio_path
        self.save_audio = save_audio

        self.denoise = denoise
        self.noise_thresh = noise_thresh
        self.volume_amp = volume_amp

        self.keep_running = False
        self.running = False

    def __del__(self):
        if self.running:
            self.stop()

    def start(self, callback=default_callback):
        if self.running:
            print("Already running")
        else:
            th = threading.Thread(target=self.run, args=(callback,))
            th.start()

    def stop(self):
        print("Stopping recording...")
        self.keep_running = False
        while self.running:
            time.sleep(0.1)
        print("Stopped recording")

    def run(self, callback=default_callback):
        print("Started recording audio")
        try:
            with sd.RawInputStream(samplerate=self.samplerate, blocksize=self.blocksize, device=self.device,
                                   dtype='int16', channels=1, callback=self._stream_callback):
                rec = vosk.KaldiRecognizer(self.model, self.samplerate)
                frames = []
                self.keep_running = True
                self.running = True
                while self.keep_running:
                    data = self.audio_q.get()
                    frames.append(data)
                    if rec.AcceptWaveform(data):
                        frame_data = b"".join(frames)
                        if self.denoise:
                            frame_data = apply_denoise(self.samplerate, frame_data, self.noise_thresh, self.volume_amp)
                        audio = AudioData(frame_data, self.samplerate, self.sample_width)

                        vosk_res = json.loads(rec.Result())['text']
                        # try:
                        #     google_res = self.r.recognize_google(audio)
                        # except:
                        #     google_res = ""
                        # # sphinx_res = self.r.recognize_sphinx(audio)

                        if self.save_audio:
                            # save audio file
                            th = threading.Thread(target=self.save_to_wave, args=(audio, vosk_res))
                            th.start()

                        # results = {"vosk": vosk_res, "google": google_res}
                        results = {"vosk": vosk_res}

                        self.outputs_q.put((results, time.time()))

                        # use results in callback
                        terminate = callback(frames, results)

                        if len(frames) > self.keep_frames:
                            frames = []
                        if terminate:
                            break
                    else:
                        # print(rec.PartialResult())
                        pass
        except Exception as e:
            print(e)
        self.keep_running = False
        self.running = False
        sys.exit(0)

    def _stream_callback(self, indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        self.audio_q.put(bytes(indata))

    def save_to_wave(self, audio: AudioData, text):
        text = '_'.join(text.split(' '))
        with open(os.path.join(self.audio_path, f"{int(time.time())}__{text}.wav"), "wb") as f:
            f.write(audio.get_wav_data())
        sys.exit(0)


def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text


def get_vosk_args():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        '-l', '--list-devices', action='store_true',
        help='show list of audio devices and exit')
    args, remaining = parser.parse_known_args()
    if args.list_devices:
        print(sd.query_devices())
        parser.exit(0)
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[parser])
    parser.add_argument(
        '-f', '--filename', type=str, metavar='FILENAME',
        help='audio file to store recording to')
    parser.add_argument(
        '-m', '--model', type=str, metavar='MODEL_PATH',
        help='Path to the model')

    parser.add_argument(
        '-d', '--device', type=int_or_str,
        help='input device (numeric ID or substring)')
    parser.add_argument(
        '-r', '--samplerate', type=int, help='sampling rate')
    args = parser.parse_args(remaining)
    config = vars(args)
    return config


if __name__ == "__main__":
    config = get_vosk_args()

    recorder = Recorder(**config, save_audio=True)
    recorder.start()
    time.sleep(20)
    recorder.stop()
