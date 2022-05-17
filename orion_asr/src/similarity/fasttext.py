# This is the code for classifying which task the robot is given based on the word input
# The input is transcription, and the given example sentences for tasks are candidates
# The similarity between the sentences is computed using the cosine similarity in 300 dimensional embedding space
# using pre-trained fasttext vectors
# https://fasttext.cc/docs/en/unsupervised-tutorial.html
# Embeddings for sentences are formed as averages of the embeddings of words which are forming them
# More weight is put on verbs in order to classify tasks. Nouns then determine an object of the task
# e.g. Bring me a banana vs Bring me something vs Bring me fruit
import pickle
from typing import List

import numpy as np
from scipy.spatial.distance import cosine
import nltk
from nltk.tokenize import word_tokenize
# nltk.download('punkt')
from spacy.lang.en.stop_words import STOP_WORDS
import string
import os


DEFAULT_VECTORS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../data/fasttext_vectors.p')


def clean(sentence):
    tokens = word_tokenize(sentence)
    stopwords = STOP_WORDS
    return [w.lower() for w in tokens if w not in string.punctuation and w not in stopwords]


class TextVector:
    vectors = None

    def __init__(self, vectors_path=DEFAULT_VECTORS_PATH, verb_weight=1.2):
        if self.vectors is None:
            try:
                with open(vectors_path, 'rb') as f:
                    self.vectors = pickle.load(f)
            except:
                raise Exception(f"Wrong vectors path {vectors_path}")

        self.verbs = ['bring', 'find', 'search', 'go', 'deliver', 'tidy', 'learn', 'take', 'is']
        self.verb_weight = verb_weight

    def sent_to_vec(self, sentence):
        words = clean(sentence)
        vecs = []
        for word in words:
            try:
                vec = np.array(self.vectors[word])
                if word in self.verbs:
                    vec = vec * self.verb_weight
                vecs.append(vec)
            except:
                print('No vector of ', word)
        return np.mean(vecs, axis=0)

    def sents_to_vec(self, sentences: List[str]):
        return np.stack([self.sent_to_vec(sent) for sent in sentences])


def compare(candidates, transcriptions):
    text_vector = TextVector()
    cand_vecs = text_vector.sents_to_vec(candidates)
    trans_vecs = text_vector.sents_to_vec(transcriptions)

    cosine_dist = 1
    c_min, t_min = 0, 0
    for i, cand_vec in enumerate(cand_vecs):
        for j, trans_vec in enumerate(trans_vecs):
            distance = cosine(cand_vec, trans_vec)
            if cosine_dist > distance:
                c_min = i
                t_min = j
                cosine_dist = float(distance)

    return cosine_dist, c_min, t_min


def compare_individual(candidate, transcription, vectors, verbs, weight):
    # Candidate is a sentence
    # Transcriptions is a list of two strings, one from Vosk model and one from Google
    print("I'm comparing", candidate, transcription)
    text_vector = TextVector()
    cand_vec = text_vector.sents_to_vec(candidate)
    trans_vec = text_vector.sents_to_vec(transcription)
    return cosine(cand_vec, trans_vec)


if __name__ == "__main__":
    candidates = ['search for objects', 'tidy up', 'bring me something', 'learn new object', 'go to start',
                  'bring me a piece of fruit']

    transcriptions = ["bring me a piece of fruit", "bring me the cupboard"]

    print(compare(candidates, transcriptions))
