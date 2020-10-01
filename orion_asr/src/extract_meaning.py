

import nltk
from nltk import word_tokenize, pos_tag
from nltk.corpus import wordnet as wn

def penn_to_wn(tag):
    """ Convert between a Penn Treebank tag to a simplified Wordnet tag """
    if tag.startswith('N') or tag.startswith('P'):
        return 'n'

    if tag.startswith('V'):
        return 'v'

    if tag.startswith('J'):
        return 'a'

    if tag.startswith('R'):
        return 'r'

    return None

def tagged_to_synset(word, tag):
    wn_tag = penn_to_wn(tag)

    if wn_tag is None:
        return None
    print(wn.synsets(word, wn_tag))
    try:

        return wn.synsets(word, wn_tag)[0]
    except:
        return None

def sentence_similarity(sentence1, sentence2):
    """ compute the sentence similarity using Wordnet """
    # Tokenize and tag
    sentence1 = pos_tag(word_tokenize(sentence1))
    sentence2 = pos_tag(word_tokenize(sentence2))
    print(sentence1)
    print(sentence2)
    # Get the synsets for the tagged words
    synsets1 = [tagged_to_synset(*tagged_word) for tagged_word in sentence1]
    synsets2 = [tagged_to_synset(*tagged_word) for tagged_word in sentence2]
    print(synsets1)
    print(synsets2)
    # Filter out the Nones
    synsets1 = [ss for ss in synsets1 if ss]
    synsets2 = [ss for ss in synsets2 if ss]
    print(synsets1)
    print(synsets2)
    score, count = 0.0, 0
    # For each word in the first sentence
    for synset in synsets1:
        # Get the similarity value of the most similar word in the other sentence
        l = []
        for ss in synsets2:
            sim = synset.path_similarity(ss)
            if sim != None:
                l.append(sim)
        if l != []:
            best_score = max(l)
            score += best_score
            count += 1

    # Average the values
    score /= count
    return score

sentences = [
    "fetch me animal."
]

focus_sentence = "bring me animal."

for sentence in sentences:
    print("Similarity(\"%s\", \"%s\") = %s" % (focus_sentence, sentence, sentence_similarity(focus_sentence, sentence)))
    print("Similarity(\"%s\", \"%s\") = %s" % (sentence, focus_sentence, sentence_similarity(sentence, focus_sentence)))
    print

# Similarity("Cats are beautiful animals.", "Dogs are awesome.") = 0.511111111111
# Similarity("Dogs are awesome.", "Cats are beautiful animals.") = 0.666666666667

# Similarity("Cats are beautiful animals.", "Some gorgeous creatures are felines.") = 0.833333333333
# Similarity("Some gorgeous creatures are felines.", "Cats are beautiful animals.") = 0.833333333333

# Similarity("Cats are beautiful animals.", "Dolphins are swimming mammals.") = 0.483333333333
# Similarity("Dolphins are swimming mammals.", "Cats are beautiful animals.") = 0.4

# Similarity("Cats are beautiful animals.", "Cats are beautiful animals.") = 1.0
# Similarity("Cats are beautiful animals.", "Cats are beautiful animals.") = 1.0
