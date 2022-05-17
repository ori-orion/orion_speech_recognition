import os
from typing import List

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import Levenshtein
import numpy as np
from similarity import fasttext, synset
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


def classify_text(candidates: List[str], candidate_params: List[str], transcriptions: List[str], algorithm='fasttext'):
    """
    Retrieves the best target command, best transcription, most likely variable content and confidence level
    given a list of feasible transcriptions and a set of target candidates and variables.
    Synset, Levenshtein and FastText are available as similarity measures
    """
    classifiers = {"synset": classify_synset, "levenshtein": classify_Levenshtein, "fasttext": classify_fasttext}
    classifier = classifiers[algorithm]

    best_candidate, best_transcription, confidence = classifier(candidates, transcriptions)

    param = candidate_params[candidates.index(best_candidate)]
    return best_candidate, param, best_transcription, confidence


def classify_Levenshtein(candidates: List[str], transcriptions: List[str]):
    """
    Uses the character-level Levenshtein distance as a similarity metric. Useful for bad speech transcribers that output
    a jumble of characters, but doesn't have any semantic understanding.
    """
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
    return candidates[i_cand], transcriptions[i_trans], scores[i_cand, i_trans]


def classify_synset(candidates: List[str], transcriptions: List[str]):
    """
    Uses WordNet to retrieve words with similar semantic meaning and uses those to compare semantic similarity.
    Works well but slower.
    """
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
    return candidates[c_max], transcriptions[t_max], max_similarity


def classify_fasttext(candidates: List[str], transcriptions: List[str]):
    """
    Uses WordNet to retrieve words with similar semantic meaning and uses those to compare semantic similarity.
    Works well but slower.
    """
    cosine_dist, c_max, t_max = fasttext.compare(candidates, transcriptions)
    return candidates[c_max], transcriptions[t_max], 1 - cosine_dist / 2


if __name__ == "__main__":
    candidates = ['search for objects', 'tidy up', 'bring me something', 'learn new object', 'go to start',
                  "bring me a <param>"]
    params = ["banana", "tomato", "peach", "toothbrush", "apple"]

    parsed_candidates, candidate_params = parse_candidates(candidates, params)

    transcriptions = ["bring me a apple", "where is bathroom"]

    start = time.time()
    print(classify_text(parsed_candidates, candidate_params, transcriptions, algorithm="fasttext"))
    end = time.time()
    print(end - start)
