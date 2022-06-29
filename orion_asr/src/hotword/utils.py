from os.path import dirname, abspath
import os

from pvporcupine import pv_keyword_files_subdir, pv_keyword_paths


def hotword_keyword_paths():
    keyword_files_dir = os.path.join(dirname(dirname(dirname(abspath(__file__)))),
                                     "data", "porcupine", pv_keyword_files_subdir())

    res = dict()
    for x in os.listdir(keyword_files_dir):
        res[str(x).split('_')[0].replace("--", "'").replace("-", " ").lower()] = os.path.join(keyword_files_dir, x)

    return {**pv_keyword_paths(''), **res}


if __name__ == "__main__":
    print(hotword_keyword_paths().keys())
