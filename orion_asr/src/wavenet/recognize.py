# -*- coding: utf-8 -*-
import sugartensor as tf
import numpy as np
import librosa
from model import *
import data, os


class WaveNet(object):

    def __init__(self):
        
        # set log level to debug
        tf.sg_verbosity(10)

        #
        # hyper parameters
        #

        batch_size = 1     # batch size

        #
        # inputs
        #

        # vocabulary size
        voca_size = data.voca_size

        # mfcc feature of audio
        x = tf.placeholder(dtype=tf.sg_floatx, shape=(batch_size, None, 20))

        # sequence length except zero-padding
        seq_len = tf.not_equal(x.sg_sum(axis=2), 0.).sg_int().sg_sum(axis=1)

        # encode audio feature
        logit = get_logit(x, voca_size=voca_size)

        # ctc decoding
        decoded, _ = tf.nn.ctc_beam_search_decoder(logit.sg_transpose(perm=[1, 0, 2]), seq_len, merge_repeated=False)

        # to dense tensor
        self.y = tf.sparse_to_dense(decoded[0].indices, decoded[0].dense_shape, decoded[0].values) + 1
        self.x = x

    def transcribe(self, filename):
        # load wave file
        wav, _ = librosa.load(filename, mono=True, sr=16000)
        
        # get mfcc feature
        mfcc = np.transpose(np.expand_dims(librosa.feature.mfcc(wav, 16000), axis=0), [0, 2, 1])

        # run network
        with tf.Session() as sess:

            # init variables
            tf.sg_init(sess)

            # restore parameters
            saver = tf.train.Saver()
            saver.restore(sess, tf.train.latest_checkpoint(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'asset/train')))
            # run session
            label = sess.run(self.y, feed_dict={self.x: mfcc})

            # print label
            transcription = data.get_index(label)

        return transcription

if __name__ == "__main__":
    wavenet = WaveNet()
    tf.sg_arg_def(file=('', 'speech wave file to recognize.'))
    filename = tf.sg_arg().file
    print(wavenet.transcribe(filename))