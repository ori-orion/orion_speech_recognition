## Requirements

install with pip:

```
NumPy
PyAudio
SpeechRecognition
python-Levenshtein
tensorflow == 1.0.0
sugartensor == 1.0.0.2
pandas >= 0.19.2
librosa == 0.6.2
scikits.audiolab==0.11.0
```

## WaveNet
Cloned from https://github.com/buriburisuri/speech-to-text-wavenet in /wavenet directory. Modified /wavenet/recognize.py so that the transcriber could be called externally.

## Setup

Create a directory:
```
orion_asr/src/tmp
```

## Run Action Server
You can run the speech recognition server with the following:
```
rosrun orion_asr speech_server.py
```

## Test Action Server

You can test the action server by sending a goal to it, using the axclient GUI tool.  
```
rosrun actionlib axclient.py /speak_and_listen orion_actions/SpeakAndListenAction
```

Example input 1:
```
question: 'What is your name?'
candidates: ["Brian <param>", "Charlie", "Julia"]
params: ["Jobs", "Potter", "Smith"]
timeout: 0.0
```

Example output 1:
```
answer: "Brian Smith"
param: "Smith"
confidence: 0.909090936184
succeeded: True
```

Example input 2:
```
question: 'What do you want me to do?'
candidates: ['search for objects', 'tidy up', 'bring me something', 'learn new object', 'go to start', "bring me a <param>"]
params = ["banana", "tomato", "peach", "toothbrush", "apple"]
timeout: 0.0
```

Example output 2:
```
answer: "Brian Smith"
param: "Smith"
confidence: 0.909090936184
succeeded: True
```