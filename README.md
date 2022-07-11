# ORIon Speech Recognition
Repo for speech recognition capabilities for the ORIon robot

## ORIon HRI (discontinued)
Used for 2018 competition.
Implements Speech recognition using Julius, which is built into the HSR. 
Works offline, but requires manual customisation of language model dictionaries for each task.

## ORIon ASR
Used for 2019-2022 competitions.
Allows general speech recognition using Speech to Text models and Snowboy hotword detection. 

`SpeakAndListen` action uses Google Speech to Text API as a primary method for speech to text transcription. When the 
Wi-Fi is unavailable or slow, which is an expected scenario for robotics competitions, we have offline fall-back alternatives,
which uses Vosk. 

`AskPersonName` action specialises at detecting person names.

It also listens to hotwords (e.g. "hey robot" and "I'm ready") and publishes to a topic `/hotword`. It uses a hotword detector called [Porcupine](https://picovoice.ai/docs/quick-start/console-porcupine/) which is a part of the library `picovoice`. 
Custom hotwords can be added by creating an account and downloading a custom model.

Speech to text is suited for common words and phrase, whereas hotword detection is suited for rarer vocabulary, is rapid and works offline.

### Potential future work
Works as it is now, but may be ideal to integrate the two so that ORIon ASR is used when Wi-Fi is available and the fall-back is either using Julius (or Kaldi) when it's trained, or PocketSphinx/WaveNet when it's not.
Basic noise reduction and volume adjustment is performed, but could be improved. 
Some other tasks may require:
- identifying the speech direction using ICA on multiple microphones
- identifying individual speakers
- detection of certain sounds such as door knocking
- natural language processing to allow varying inputs to be classified correctly

Please refer to README files in subdirectories for more information.
