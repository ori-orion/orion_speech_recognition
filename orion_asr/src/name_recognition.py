import nltk


def check_for_name(transcripts):
    # The function which checks if the sentence is one if the following and extracts the name:
    # My name is.. I am... I'm ... *Just name*... They call me ... 
    for transcript in transcripts:
        tokens = nltk.word_tokenize(transcript)
        # for I'm it returns [I, 'm] so we need just m
        for i, word in enumerate(tokens):
            if len(word.split("'")) > 1:
                tokens[i] = word.split("'")[1]
        tokens = [word.lower() for word in tokens if word.isalpha()]

        name = ''
        if len(tokens) <= 2:
            # If they only say one word that word is their name
            name = tokens[0].capitalize()
        else:
            for i, word in enumerate(tokens):
                if word == 'is' or word == 'am' or word == "i'm" or word == "m" or word == "me":
                    # My name is NAME
                    try:
                        name = tokens[i + 1].capitalize()
                    except IndexError:
                        pass
        if name:
            return name
    # If none of the transcripts returned a name:
    return ''
