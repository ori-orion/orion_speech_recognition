import os
import wave
import queue
import argparse
import sounddevice as sd
import vosk
import sys


q = queue.Queue()

def intORstr(text):
    try:
        return int(text)
    except ValueError:
        return text

def callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

def main():

    parser = argparse.ArgumentParser(add_help=False)
    args, remaining = parser.parse_known_args()

    parser.add_argument('-m','--model',type=str,metavar='MODEL_PATH',help='Path to model')
    parser.add_argument('-r','--samplerate',type=int,help='sampling rate')
    parser.add_argument('-d','--device',type=intORstr,help='input device (numeric ID or substring')
    parser.add_argument('-f','--filename',type=str,metavar='FILENAME',help='audiofile to store recording to')

    args = parser.parse_args(remaining)

    try:
        if args.model is None:
            args.model = "vosk_model"
        if not os.path.exists(args.model):
            print("Please download a model for your language from https://alphacephei.com/vosk/models and unpack as 'model' in the current folder.")
            parser.exit(0)
        if args.samplerate is None:
            device_info = sd.query_devices(args.device, 'input')
            args.samplerate = int(device_info['default_samplerate'])


        model = vosk.Model(args.model)

        if args.filename:
            dump_fn = open(args.filename, "wb")
        else:
            dump_fn = None

        with sd.RawInputStream(samplerate=args.samplerate,blocksize = 8000,device=args.device,dtype='int16',channels = 1, callback=callback):
            print('Press Ctrl+C to stop recording')

            rec = vosk.KaldiRecognizer(model, args.samplerate)
            while True:
                wavefile = wave.open("sound.wav","wb")
                wavefile.setnchannels(1)
                wavefile.setsampwidth(2)
                wavefile.setframerate(args.samplerate)
                data = q.get()
                if rec.AcceptWaveform(data):
                    print(rec.Result())
                if dump_fn is not None:
                    dump_fn.write(data)

    except KeyboardInterrupt:
        wavefile.close()
        print('\nDone')
        parser.exit(0)
    except Exception as e:
        parser.exit(type(e).__name__ + ': ' + str(e))


if __name__ == '__main__':
    main()
