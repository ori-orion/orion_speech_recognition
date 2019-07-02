import numpy as np
import scipy, os, time
from logmmse import logmmse
from record import Recorder

with Recorder() as rec:
    print("Started recording...")
    gen = rec.frames_generator()
    data, max_energy = next(gen)
    npdata = np.fromstring(data, dtype=np.int16).astype(np.float64)
    print(npdata[:10])
    absmax = np.abs(npdata).max()
    npdata = (npdata * 48000 / absmax / np.log(max_energy)).astype(np.int16)
    print(npdata[:10])
    print(absmax, max_energy)
    print("Filtering and saving wav...")
    filename = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                            "tmp/%s.wav" % int(time.time()))
    logmmse(npdata, rec.config['RATE'], output_file=filename)
    # scipy.io.wavfile.write(filename, rec.config['RATE'], npdata)