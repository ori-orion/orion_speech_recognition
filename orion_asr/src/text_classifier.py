import os
from typing import List

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import Levenshtein
import numpy as np
from nlp import fasttext, synset
import time


def parse_candidates(candidates, params):
    parsed_candidates = []
    candidate_params = []
    for candidate in candidates:
        if params and "<param>" in candidate:
            for param in params:
                parsed_candidates.append(candidate.replace("<param>", param))
                candidate_params.append(param)
        else:
            parsed_candidates.append(candidate)
            candidate_params.append("")
    return parsed_candidates, candidate_params


def classify_text(input_texts: List[str], candidates: List[str], candidate_params: List[str], algorithm='fasttext', thresh=0.6):
    classifiers = {"synset": classify_synset, "levenshtein": classify_Levenshtein, "fasttext": classify_fasttext}
    classifier = classifiers[algorithm]
    try:
        sentence, confidence, transcription = classifier(candidates, input_texts)

        if confidence > thresh:
            print("Similarity Measure: " + algorithm)
            print("Transcription: " + transcription)
            print("Most relevant task: " + sentence)
            print(f"Confidence [0,1]: {confidence}")
        else:
            raise Exception("No valid task was found")
        print()
        param = candidate_params[candidates.index(sentence)] if sentence else ""
        return sentence, param, confidence, transcription, confidence > thresh

    except:
        print("please try again")


def classify_Levenshtein(candidates: List[str], transcriptions: List[str]):
    scores = np.zeros((len(candidates), len(transcriptions)))
    for i_cand, candidate in enumerate(candidates):
        score = 0
        m = int(len(candidate))
        for j, transcription in enumerate(transcriptions):
            for i in range(max(len(transcription) - m + 1, 1)):
                subtext = transcription[i:i + m]
                score = max(Levenshtein.ratio(candidate, subtext), score)
            scores[i_cand, j] = score
    i_max = np.argmax(scores)
    i_cand = i_max // len(transcriptions)
    i_trans = i_max % len(transcriptions)
    return candidates[i_cand], scores[i_cand, i_trans], transcriptions[i_trans]


def classify_synset(candidates: List[str], transcriptions: List[str]):
    max_similarity = 0
    c_max, t_max = 0, 0
    for i, candidate in enumerate(candidates):
        for j, transcription in enumerate(transcriptions):
            try:
                sim = float(synset.synset(candidate, transcription))
                if sim > max_similarity:
                    c_max = i
                    t_max = j
                    max_similarity = sim
            except Exception as e:
                print(e)
    return candidates[c_max], max_similarity, transcriptions[t_max]


def classify_fasttext(candidates: List[str], transcriptions: List[str]):
    cosine_dist, c_max, t_max = fasttext.compare(candidates, transcriptions)
    return candidates[c_max], 1 - cosine_dist / 2, transcriptions[t_max]


if __name__ == "__main__":
    candidates = ['search for objects', 'tidy up', 'bring me something', 'learn new object', 'go to start',
                  "bring me a <param>"]
    params = ["banana", "tomato", "peach", "toothbrush", "apple"]

    parsed_candidates, candidate_params = parse_candidates(candidates, params)

    transcriptions = ["bring me a apple", "where is bathroom"]

    start = time.time()
    print(classify_text(transcriptions, parsed_candidates, candidate_params))
    end = time.time()
    print(end - start)
