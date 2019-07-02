import numpy as np
import scipy, os, time
from logmmse import logmmse
from record import Recorder

with Recorder() as rec:
    print("Started recording...")
    gen = rec.frames_generator()
    data = next(gen)
    npdata = np.fromstring(data, dtype=np.int16).astype(np.float64)
    print(npdata[:10])
    npdata = np.fromstring(data, dtype=np.int16).astype(np.float64)
    absmax = np.abs(npdata).max()
    npdata = (npdata * 32000.0 / absmax).astype(np.int16)
    print(npdata[:10])
    print(absmax)
    print("Filtering and saving wav...")
    filename = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                            "tmp/%s.wav" % int(time.time()))
    logmmse(npdata, rec.config['RATE'], output_file=filename)
    # scipy.io.wavfile.write(filename, rec.config['RATE'], npdata)