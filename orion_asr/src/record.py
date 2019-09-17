import pyaudio, threading
import time as Time
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal


class Recorder(object):

    def __init__(self, max_sec=10, window_sec=1.2):

        self.data = []
        self.config = {
            'FORMAT': pyaudio.paInt16,
            'CHANNELS': 1,
            'RATE': 16000,
            'CHUNK': 1024,
            'WIDTH': 2
        }
        self.max_sec = max_sec
        self.window_sec = window_sec
        self.is_recording = True
        self.th = threading.Thread(target=self.run, args=())

    def __enter__(self):
        self.th.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.is_recording = False
        self.th.join()

    def run(self):

        pa = pyaudio.PyAudio()
        config = self.config
        stream = pa.open(format=config['FORMAT'], channels=config['CHANNELS'],
                         rate=config['RATE'], input=True,
                         frames_per_buffer=config['CHUNK'])

        while self.is_recording:

            frames = []
            max_energy = 0
            frames_per_sec = config['RATE'] / config['CHUNK']

            for _ in range(0, int(frames_per_sec * self.max_sec)):
                frame = stream.read(config['CHUNK'])
                frames.append(frame)
                window = frames[max(0, len(frames) - int(frames_per_sec * self.window_sec)):]
                pxx, freq, time = self.spectrogram(window)
                rms_energy = self.rmsenergy(pxx).mean()
                max_energy = max(max_energy, rms_energy)
                print(rms_energy.mean())
                if rms_energy < max_energy / 4 or not self.is_recording:
                    break

            self.data.append((frames, max_energy))

        stream.stop_stream()
        stream.close()
        pa.terminate()

    def frames_generator(self):
        i = 0
        while self.is_recording:
            while i >= len(self.data):
                Time.sleep(0.1)
            frames, max_energy = self.data[i]
            data = b''.join(frames)
            yield data, max_energy
            i += 1

    def spectrogram(self, frames, show=False):
        data = b''.join(frames)
        npdata = np.fromstring(data, dtype=np.int16)
        if show:
            pxx, freq, time, _ = plt.specgram(npdata, Fs=self.config['RATE'], cmap="jet")
        else:
            freq, time, pxx = signal.spectrogram(npdata, fs=self.config['RATE'])
        return pxx, freq, time

    def rmsenergy(self, pxx):
        pxx = np.array(pxx)
        pxx_sqr = pxx * pxx
        pxx_sqr_sum = pxx_sqr.sum(axis=0)
        rms_energy = np.sqrt(pxx_sqr_sum)
        return rms_energy

    def graph(self, frames, show=True):
        plt.gcf().clear()
        ax1 = plt.subplot(2, 1, 1)
        pxx, freq, time = self.spectrogram(frames, show=True)
        ax1.title.set_text('Spectrogram')
        ax1.set_ylabel('frequency [Hz]')
        ax1.set_xlabel('time [s]')
        ax2 = plt.subplot(2, 1, 2)
        rms_energy_arr = self.rmsenergy(pxx)
        plt.plot(time, np.log10(rms_energy_arr))
        ax2.title.set_text('Signal RMS energy')
        ax2.set_ylabel('RMS energy')
        ax2.set_xlabel('time [s]')
        plt.subplots_adjust(hspace=0.4)
        if show:
            plt.show()


if __name__ == "__main__":
    with Recorder() as rec:
        gen = rec.frames_generator()
        data, _ = next(gen)
        rec.graph(data)
        data, _ = next(gen)
        rec.graph(data)
