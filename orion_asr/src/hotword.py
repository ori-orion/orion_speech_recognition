import os, rospy, threading, pyaudio, time, wave
from snowboy import snowboydecoder


class HotwordDetector(object):
    def __init__(self, hotwords, preempt_callback):

        self.frames = []
        self.config = {
            'FORMAT': pyaudio.paInt16,
            'CHANNELS': 1,
            'RATE': 16000,
            'CHUNK': 1024,
            'WIDTH': 2
        }
        self.MODEL_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "snowboy/resources")

        if not hotwords:
            rospy.logerr("No hotwords passed as arguments")
            hotwords = [hotword.replace(".pmdl", "") for hotword in os.listdir(self.MODEL_PATH) if ".pmdl" in hotword]
            rospy.loginfo(hotwords)

        self.hotwords = []
        self.is_recording = True

        for hotword in hotwords:
            hotword = hotword.lower()
            if hotword + ".pmdl" in os.listdir(self.MODEL_PATH):
                self.hotwords.append(hotword)
            else:
                rospy.logerr("Requested hotword (%s) not trained" % hotword)

        self.detected_hotword = ""
        self.preempt_callback = preempt_callback

        self.threads = [threading.Thread(target=self.detect, args=(ind,)) for ind in range(len(self.hotwords))]

    def run(self):

        record_thread = threading.Thread(target=self.record, args=())
        record_thread.start()

        rospy.loginfo('Started listening to hotwords...')
        for thread in self.threads:
            thread.start()
        for thread in self.threads:
            thread.join()

        rospy.loginfo('Finished listening to hotwords...')
        record_thread.join()

    def interrupt_callback(self):
        if self.detected_hotword != "" or self.preempt_callback():
            self.is_recording = False
            return True
        return False

    def detect(self, ind):
        hotword = self.hotwords[ind]
        model = os.path.join(self.MODEL_PATH, hotword + '.pmdl')
        detector = snowboydecoder.HotwordDetector(model, sensitivity=0.4)

        def detected_callback(i):
            self.detected_hotword = hotword
            rospy.loginfo("Detected hotword: " + hotword)

        # main loop
        detector.start(detected_callback=detected_callback,
                       interrupt_check=self.interrupt_callback,
                       sleep_time=0.03)

        detector.terminate()

    def record(self):

        pa = pyaudio.PyAudio()
        config = self.config
        stream = pa.open(format=config['FORMAT'], channels=config['CHANNELS'],
                         rate=config['RATE'], input=True,
                         frames_per_buffer=config['CHUNK'])

        while self.is_recording:
            frame = stream.read(config['CHUNK'])
            self.frames.append(frame)

        stream.stop_stream()
        stream.close()
        pa.terminate()

        filename = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                "tmp/%s-hotword.wav" % int(time.time()))

        f = wave.open(filename, 'wb')
        f.setnchannels(config['CHANNELS'])
        f.setsampwidth(pa.get_sample_size(config['FORMAT']))
        f.setframerate(config['RATE'])
        f.writeframes(b''.join(self.frames))
        f.close()
