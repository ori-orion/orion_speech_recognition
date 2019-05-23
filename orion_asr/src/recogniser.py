import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import speech_recognition as sr
# from wavenet.recognize import WaveNet
import Levenshtein, threading, time
import numpy as np
import rospy

class ASR(object):

    def __init__(self, recognizer, wavenet, thresh=0.6):
        self.rec = recognizer
        self.wavenet = wavenet
        self.google_available = True
        self.audios = []
        self.transcription = [""]
        self.thresh = thresh

    def transcribe(self, audio, candidates):
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
            #
            # try:
            #     wav = audio.get_wav_data(16000)
            #     tmp_filename = os.path.join(os.path.dirname(os.path.realpath(__file__)), "tmp/%s.wav" % int(time.time()))
            #     with open(tmp_filename, "wb") as f:
            #         f.write(wav)
            #     text = self.wavenet.transcribe(tmp_filename)
            #     os.remove(tmp_filename)
            # except Exception as e:
            #     rospy.logerr(e)

            th.join()

        if text:
            self.audios.append(audio)
            self.transcription[0] += " " + str(text)

    def record(self, audio_source, candidates):
        try:
            audio = self.rec.listen(audio_source, timeout=5.0, phrase_time_limit=5.0)
            self.transcribe(audio, candidates)
        except sr.WaitTimeoutError as e:
            pass
        sentence, confidence, transcription = self.classify(candidates, self.transcription)
        print(self.transcription, sentence, confidence)
        return sentence, confidence, transcription, confidence > self.thresh

    @staticmethod
    def classify(candidates, transcriptions):
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
        print(scores, i_max, i_cand, i_trans, candidates)
        print(candidates[i_cand], scores[i_cand, i_trans], transcriptions[i_trans])
        return candidates[i_cand], scores[i_cand, i_trans], transcriptions[i_trans]

    def main(self, source, candidates, params):
        candidates_parsed = []
        params_index = []
        for i_cand, candidate in enumerate(candidates):
            if params and "<param>" in candidate:
                for i_param, param in enumerate(params):
                    candidates_parsed.append(candidate.replace("<param>", param))
                    params_index.append(i_param)
            else:
                candidates_parsed.append(candidate)
                params_index.append(-1)

        sentence, confidence, transcription, succeeded = self.record(source, candidates_parsed)
        i_param = params_index[candidates_parsed.index(sentence)] if sentence else -1
        param = params[i_param] if i_param >= 0 else ""
        return sentence, param, confidence, transcription, succeeded
        

if __name__ == "__main__":
    recognizer = sr.Recognizer()
    # wavenet = WaveNet()

    print("Instantiating ASR...")
    asr = ASR(recognizer, "wavenet")

    candidates = ['search for objects', 'tidy up', 'bring me something', 'learn new object', 'go to start', "bring me a <param>"]
    params = ["banana", "tomato", "peach", "toothbrush", "apple"]

    # print(asr.classify(candidates, transcription))
    with sr.Microphone() as source:
        print("Started recording...")
        print(asr.main(source, candidates, params))