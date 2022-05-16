# This is the code for classifying which task the robot is given based on the word input
# The input is transcription, and the given example sentences for tasks are candidates
# The similarity between the sentences is computed using the cosine similarity in 300 dimensional embedding space
# using pre-trained fasttext vectors
# Embeddings for sentences are formed as averages of the embeddings of words which are forming them
# More weight is put on verbs in order to classify tasks. Nouns then determine an object of the task
# e.g. Bring me a banana vs Bring me something vs Bring me fruit

import pickle
import numpy as np
from scipy.spatial.distance import cosine
# from sklearn.feature_extraction import stop_words
import nltk
from nltk.tokenize import word_tokenize
# nltk.download('punkt')
from spacy.lang.en.stop_words import STOP_WORDS
import string


def clean(sentence):
    tokens = word_tokenize(sentence)
    stopwords = STOP_WORDS
    return [w.lower() for w in tokens if w not in string.punctuation and w not in stopwords]


def getVerbs():
    verbs = ['bring', 'find', 'search', 'go', 'deliver', 'tidy', 'learn', 'take', 'is']
    return verbs


def compare(candidates, transcriptions):
    vectors_path = '../../data/fasttext_vectors.p'  # Change this to the relative path on your machine

    try:
        with open(vectors_path, 'rb') as f:
            vectors = pickle.load(f)
            print("I loaded")
    except:
        print("Wrong vectors path")

    # Get a list of verbs
    verbs = getVerbs()
    weight = 1.2

    print("I made it to past reading vectors")

    vec_sum_cand = np.zeros((len(candidates), len(vectors['find'])))
    vec_sum_trans = np.zeros((len(transcriptions), len(vectors['find'])))

    for i in range(len(candidates)):
        cand = clean(candidates[i])
        for word in cand:
            try:
                if (word in verbs):
                    vec_sum_cand[i] = np.add(vec_sum_cand[i], np.multiply(vectors[word], weight))
                else:
                    vec_sum_cand[i] = np.add(vec_sum_cand[i], vectors[word])
            except:
                print('No vector of ', word)
        vec_sum_cand[i] = vec_sum_cand[i] / len(cand)

    for i in range(len(transcriptions)):
        trans = clean(transcriptions[i])
        for word in trans:
            try:
                if (word in verbs):
                    vec_sum_trans[i] = np.add(vec_sum_trans[i], np.multiply(vectors[word], weight))
                else:
                    vec_sum_trans[i] = np.add(vec_sum_trans[i], vectors[word])
            except:
                print('No vector of ', word)
        vec_sum_trans[i] = vec_sum_trans[i] / len(trans)

    min_similarity = 1
    for i in range(len(candidates)):
        for j in range(len(transcriptions)):
            sim = cosine(vec_sum_cand[i], vec_sum_trans[j])
            if (min_similarity > sim):
                c_min = i
                t_min = j
                min_similarity = float(sim)

    return min_similarity, c_min, t_min


def compareIndividual(candidate, transcription, vectors, verbs, weight):
    # Candidate is a sentence
    # Transcriptions is a list of two strings, one from Vosk model and one from Google
    print("I'm comparing", candidate, transcription)

    cand = clean(candidate)
    trans = clean(transcription)

    vec_sum = np.zeros(len(vectors['find']))
    for word in cand:
        try:
            if (word in verbs):
                vec_sum = np.add(vec_sum, np.multiply(vectors[word], weight))
            else:
                vec_sum = np.add(vec_sum, vectors[word])
        except:
            print('No vector of ', word)
    cand_avg = vec_sum / len(cand)

    vec_sum = np.zeros(len(vectors['find']))
    for word in trans:
        try:
            if (word in verbs):
                vec_sum = np.add(vec_sum, np.multiply(vectors[word], weight))
            else:
                vec_sum = np.add(vec_sum, vectors[word])
        except:
            print('No vector of ', word)
    trans_avg = vec_sum / len(cand)

    return cosine(cand_avg, trans_avg)


if __name__ == "__main__":
    candidates = ['search for objects', 'tidy up', 'bring me something', 'learn new object', 'go to start',
                  'bring me a piece of fruit']

    compare(candidates)
