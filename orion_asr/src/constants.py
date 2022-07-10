import os
from os.path import dirname

ROOT_DIR = dirname((dirname(os.path.abspath(__file__))))
DATA_DIR = os.path.join(ROOT_DIR, "data")
