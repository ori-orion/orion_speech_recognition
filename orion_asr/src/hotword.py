import os, rospy, threading
from snowboy import snowboydecoder


class HotwordDetector(object):
    def __init__(self, hotwords, preempt_callback):

        self.MODEL_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "snowboy/resources")

        if not hotwords:
            rospy.logerr("No hotwords passed as arguments")
            hotwords = [hotword.replace(".pmdl", "") for hotword in os.listdir(self.MODEL_PATH) if ".pmdl" in hotword]
            rospy.loginfo(hotwords)

        self.hotwords = []

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
        rospy.loginfo('Started listening to hotwords...')
        i = 0
        for thread in self.threads:
            thread.start()
            print(i)
            i += 1
        i = 0
        for thread in self.threads:
            thread.join()
            print(i)
            i += 1
        rospy.loginfo('Finished listening to hotwords...')

    def interrupt_callback(self):
        return self.detected_hotword != "" or self.preempt_callback()

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
