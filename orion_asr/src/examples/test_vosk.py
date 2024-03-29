#!/usr/bin/env python3

import vosk     # has to be imported at the top
import argparse
import os
import queue
import sounddevice as sd
import sys

vosk.SetLogLevel(0)


class Recorder:
    def __init__(self, model=None, filename=None, device=None, samplerate=None, blocksize=8000, **kwargs):
        if model is None:
            model = "model"
        if not os.path.exists(model):
            raise Exception("""Please download a model for your language from https://alphacephei.com/vosk/models
            and unpack as 'model' in the current folder.""")
        if samplerate is None:
            device_info = sd.query_devices(device, 'input')
            # soundfile expects an int, sounddevice provides a float:
            samplerate = int(device_info['default_samplerate'])
        self.samplerate = samplerate
        self.device = device
        self.blocksize = blocksize

        self.q = queue.Queue()
        self.model = vosk.Model(model)
        self.dump_fn = open(filename, "wb") if filename else None
        self.run()

    def run(self):
        with sd.RawInputStream(samplerate=self.samplerate, blocksize=self.blocksize, device=self.device, dtype='int16',
                               channels=1, callback=self._callback):
            rec = vosk.KaldiRecognizer(self.model, self.samplerate)
            while True:
                data = self.q.get()
                if rec.AcceptWaveform(data):
                    print(rec.Result())
                else:
                    print(rec.PartialResult())
                if self.dump_fn is not None:
                    self.dump_fn.write(data)

    def _callback(self, indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        self.q.put(bytes(indata))


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

    try:
        print('#' * 80)
        print('Press Ctrl+C to stop the recording')
        print('#' * 80)
        recorder = Recorder(**config)
    except KeyboardInterrupt:
        print('\nStopped recording')
