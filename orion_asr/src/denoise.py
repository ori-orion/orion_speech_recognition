from scipy.io import wavfile
import noisereduce as nr
from scipy.io.wavfile import read, write
import io
import numpy as np


def apply_denoise(rate, frame_data, noise_thresh=0.0, volume_amp=10):
    data = np.frombuffer(frame_data, dtype=np.int16)

    data = np.array(nr.reduce_noise(y=data, sr=rate, n_std_thresh_stationary=noise_thresh), dtype=float)
    max_range = np.iinfo(np.int16).max
    data = np.asarray(np.tanh(data * volume_amp / max_range) * max_range, dtype=np.int16)

    byte_io = io.BytesIO(bytes())
    write(byte_io, rate, data)
    frame_data = byte_io.read()

    return frame_data


if __name__ == "__main__":
    print("Reading wave file")
    # load data
    rate, data = wavfile.read("../test.wav")
    print(data)
    # perform noise reduction
    reduced_noise = nr.reduce_noise(y=data, sr=rate)
    print("Reduced noise")
    wavfile.write("../test_reduced_noise.wav", rate, reduced_noise)
