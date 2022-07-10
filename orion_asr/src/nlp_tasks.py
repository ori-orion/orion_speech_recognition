from typing import List
import os

from constants import DATA_DIR

NAMES_FILE = os.path.join(DATA_DIR, "names.txt")

with open(NAMES_FILE, "r") as f:
    NAMES = f.read().strip().split("\n")


def recognise_name(transcriptions: List[str]):
    found_names = []
    for trans in transcriptions:
        for word in trans.split(" "):
            if word.capitalize() in NAMES:
                found_names.append(word.capitalize())

    if found_names:
        name = found_names[-1]
        confidence = 1.0
    else:
        name = ""
        confidence = 0.0
    return name, confidence


if __name__ == "__main__":
    print(recognise_name(["Hi, I am Shawn"]))
    print(recognise_name(["Hi, my name is Shawn"]))     # fails (thinks "My" is a name)
