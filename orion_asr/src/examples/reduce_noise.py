from scipy.io import wavfile
import noisereduce as nr
print("Reading wave file")
# load data
rate, data = wavfile.read("./test.wav")
print(data)
# perform noise reduction
reduced_noise = nr.reduce_noise(y=data, sr=rate)
print("Reduced noise")
wavfile.write("../test_reduced_noise.wav", rate, reduced_noise)
