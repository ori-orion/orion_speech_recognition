import nltk


def check_for_name(transcripts):
    # The function which checks if the sentence is one if the following and extracts the name:
    # My name is.. I am... I'm ... *Just name*... They call me ... 
    for transcript in transcripts:
        tokens = nltk.word_tokenize(transcript)
        tokens = [word.lower() if word.isalpha() or word == "'m" else word for word in tokens]

        name = ''
        if len(tokens) <= 2:
            # If they only say one word that word is their name
            name = tokens[0].capitalize()
        else:
            for i, word in enumerate(tokens):
                if word == 'is' or word == 'am' or word == "'m" or word == "me":
                    # My name is NAME
                    try:
                        name = tokens[i + 1].capitalize()
                    except IndexError:
                        pass
        if name:
            return name
    # If none of the transcripts returned a name:
    return ''
