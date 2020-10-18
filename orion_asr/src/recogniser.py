import os
import sys
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import speech_recognition as sr
# from wavenet.recognize import WaveNet
import Levenshtein, threading, time
import numpy as np
#import rospy
from record import Recorder
import scipy
from logmmse import logmmse
import SS



class ASR(object):

    def __init__(self, recognizer, wavenet, thresh=0.6):
        self.rec = recognizer
        self.wavenet = wavenet
        self.google_available = True
        self.audios = []
        self.transcription = [""]
        self.thresh = thresh
        self.candidates_parsed = []
        self.candidates_params = []

    def set_candidates(self, candidates, params):
        for candidate in candidates:
            if params and "<param>" in candidate:
                for param in params:
                    self.candidates_parsed.append(candidate.replace("<param>", param))
                    self.candidates_params.append(param)
            else:
                self.candidates_parsed.append(candidate)
                self.candidates_params.append("")

    def transcribe(self, filename):
        with sr.AudioFile(filename) as source:
            audio = self.rec.record(source)
            text = ""

            if self.google_available:
                print("Google ASR...")
                try:
                    text = self.rec.recognize_google(audio)
                except sr.UnknownValueError:
                    print("Google Speech Recognition could not understand audio")
                except Exception as e:
                    print(e)
                    self.google_available = False
                    self.transcription = [self.transcription[0] for _ in range(2)]

            if not self.google_available:

                print("Sphinx ASR + WaveNet...")

                def transcribe_sphinx(aud):
                    try:
                        sphinx_text = self.rec.recognize_sphinx(aud)
                        self.transcription[1] += " " + str(sphinx_text)
                    except sr.UnknownValueError:
                        rospy.logerr("Sphinx could not understand audio")
                    except Exception as e:
                        rospy.logerr(e)

                th = threading.Thread(target=transcribe_sphinx, args=(audio,))
                th.start()

                # if tmp_filename:
                #     try:
                #         text = self.wavenet.transcribe(tmp_filename)
                #     except Exception as e:
                #         rospy.logerr(e)

                th.join()

            if text:
                self.audios.append(audio)
                self.transcription[0] += " " + str(text)

    def record(self, audio_source, config,classification_algorithm='synset'):

        try:
            # audio = self.rec.record(audio_source, duration=5.0)
            data, max_energy = next(audio_source)
            npdata = np.fromstring(data, dtype=np.int16).astype(np.float64)
            absmax = np.abs(npdata).max()
            npdata = (npdata * 32000/absmax * 10/np.log(max_energy)).astype(np.int16)
            print("Filtering and saving wav...")
            filename = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                    "tmp/%s.wav" % int(time.time()))
            logmmse(npdata, config['RATE'], output_file=filename)
            # scipy.io.wavfile.write(filename, config['RATE'], npdata)
            self.transcribe(filename)
        except sr.WaitTimeoutError as e:
            pass
        try:
            if classification_algorithm=='synset':
                sentence, confidence, transcription = self.classify_synset(self.candidates_parsed, self.transcription)
            elif classification_algorithm=='levenshtein':
                sentence, confidence, transcription = self.classify_Levenshtein(self.candidates_parsed, self.transcription)
            else:
                raise Exception('Please provide a valid classification algorithm, synset or levenstein')

            if confidence > self.thresh:
                print("Similarity Measure: " + classification_algorithm)
                print("Transcription: " +  self.transcription[0])
                print("Most relevant task: " + sentence)
                print("Confidence [0,1]: " + str(confidence))
            else:
                raise Exception("No valid task was found")
            param = self.candidates_params[self.candidates_parsed.index(sentence)] if sentence else ""
            return sentence, param, confidence, transcription, confidence > self.thresh

        except:
            print("please try again")

            #asr.record(audio_source, config)

    @staticmethod
    def classify_Levenshtein(candidates, transcriptions):
        scores = np.zeros((len(candidates), len(transcriptions)))
        for i_cand, candidate in enumerate(candidates):
            score = 0
            m = int(len(candidate))
            for j, transcription in enumerate(transcriptions):
                for i in range(max(len(transcription) - m + 1, 1)):
                    subtext = transcription[i:i+m]
                    score = max(Levenshtein.ratio(candidate, subtext), score)
                scores[i_cand, j] = score
        i_max = np.argmax(scores)
        i_cand = i_max // len(transcriptions)
        i_trans = i_max % len(transcriptions)
        return candidates[i_cand], scores[i_cand, i_trans], transcriptions[i_trans]

    @staticmethod
    def classify_synset(candidates, transcriptions):
        max_similarity = 0
        for i, candidate in enumerate(candidates):
            for j, transcription in enumerate(transcriptions):
                sim = float(SS.synset(candidate, transcription))
                if sim>max_similarity:
                    c_max = i
                    t_max = j
                    max_similarity = sim
        return candidates[c_max], max_similarity, transcriptions[t_max]



if __name__ == "__main__":
    recognizer = sr.Recognizer()
    # wavenet = WaveNet()

    print("Instantiating ASR...")
    asr = ASR(recognizer, "wavenet")
    candidates = ['search for objects', 'tidy up', 'bring me something', 'learn new object', 'go to start', "bring me a <param>"]
    params = ["banana", "tomato", "peach", "toothbrush", "apple"]

    asr.set_candidates(candidates, params)

    # print(asr.classify(candidates, transcription))
    # with sr.Microphone() as source:

    with Recorder() as rec:
        print("Started recording...")
        gen = rec.frames_generator()


        asr.record(gen, rec.config)
        #print(asr.record(gen, rec.config))
