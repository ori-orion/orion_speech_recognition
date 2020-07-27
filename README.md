# ORIon Speech Recognition
Repo for speech recognition capabilities for the ORIon robot

## ORIon HRI
Used for 2018 competition.
Implements Speech recognition using Julius, which is built into the HSR. 
Works offline, but requires manual customisation of language model dictionaries for each task.

## ORIon ASR
Used for 2019 competition.
Allows general speech recognition using Speech to Text models and Snowboy hotword detection. 

`SpeakAndListen` action uses Google Speech to Text API as a primary method for speech to text transcription. When the 
Wi-Fi is unavailable or slow, which is an expected scenario for robotics competitions, we have offline fall-back alternatives,
which uses PocketSphinx and WaveNet. 

`HotwordListen` action uses Snowboy [https://snowboy.kitt.ai], a cloud service that trains models that can be used for offline hotword detection. 
Multiple hotword candidates can be passed to the action so that it would return when any one of them are detected. 

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
