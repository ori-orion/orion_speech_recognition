from typing import List
import os

from constants import DATA_DIR

from name_recognition import check_for_name

NAMES_FILE = os.path.join(DATA_DIR, "names.txt")

with open(NAMES_FILE, "r") as f:
    NAMES = f.read().strip().split("\n")


def recognise_name(transcriptions: List[str]):
    found_names = []
    for trans in transcriptions:
        for word in trans.split(" "):
            word = word.capitalize()
            if word in NAMES:
                found_names.append(word)

    if found_names:
        name = found_names[0]
        confidence = 1.0
    else:
        name = check_for_name(transcriptions)
        confidence = 0.9 if name else 0.0
    return name, confidence


if __name__ == "__main__":
    print(recognise_name(["I'm Shu"]))
