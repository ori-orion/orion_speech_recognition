#! /usr/bin/env python3

import nltk

if __name__ == "__main__":
  resources = ['punkt', 'wordnet', 'brown', 'omw-1.4']

  print("Starting resource downloads")
  for resource in resources:
    nltk.download(resource) 
  print("\nResource downloads complete")
