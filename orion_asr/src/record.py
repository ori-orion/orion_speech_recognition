import pyaudio, os
import time as Time
import matplotlib.pyplot as plt
import numpy as np


class Recorder(object):

    def __init__(self):

        self.frames = []
        self.pa = pyaudio.PyAudio()
        self.config = {
            'FORMAT': pyaudio.paInt16,
            'CHANNELS': 1,
            'RATE': 16000,
            'CHUNK': 1024,
            'WIDTH': 2
        }

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.pa.terminate()

    def run(self, max_sec=10, window_sec=1.2):

        start_frame = len(self.frames)

        config = self.config
        stream = self.pa.open(format=config['FORMAT'], channels=config['CHANNELS'],
                                 rate=config['RATE'], input=True,
                                 frames_per_buffer=config['CHUNK'])

        max_energy = 0
        frames_per_sec = config['RATE'] / config['CHUNK']

        for _ in range(0, int(frames_per_sec * max_sec)):
            data = stream.read(config['CHUNK'])
            self.frames.append(data)
            window = self.frames[max(0, len(self.frames) - int(frames_per_sec * window_sec)):]
            pxx, freq, time = self.spectrogram(window)
            rms_energy = self.rmsenergy(pxx, freq).mean()
            max_energy = max(max_energy, rms_energy)
            print(rms_energy.mean())
            if rms_energy < max_energy / 4:
                break

        stream.stop_stream()
        stream.close()

        data = b''.join(self.frames[start_frame:])
        return data

    def spectrogram(self, frames):
        data = b''.join(frames)
        npdata = np.fromstring(data, dtype=np.int16)
        pxx, freq, time, _ = plt.specgram(npdata, Fs=self.config['RATE'], cmap="jet")
        return pxx, freq, time

    def rmsenergy(self, pxx, freq):
        pxx = np.array(pxx)
        pxx_sqr = pxx * pxx
        pxx_sqr_sum = pxx_sqr.sum(axis=0)
        rms_energy = np.sqrt(pxx_sqr_sum)
        return rms_energy

    def graph(self, show=True):
        plt.gcf().clear()
        plt.subplot(2, 1, 1)
        pxx, freq, time = self.spectrogram(self.frames)
        plt.subplot(2, 1, 2)
        rms_energy_arr = self.rmsenergy(pxx, freq)
        plt.plot(time, np.log10(rms_energy_arr))
        if show:
            plt.show()


if __name__ == "__main__":
    with Recorder() as rec:
        rec.run()
        rec.graph()