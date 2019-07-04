# General Speech Recognition
Allows general speech recognition using Speech to Text models and Snowboy hotword detection. 

Developed by Shu Ishida - contact shuishida_at_robots.ox.ac.uk for enquiries.

## Actions
There are two actions associated with it - `SpeakAndListen.action` and `HotwordListen.action`. The version used for the 
RoboCup 2019 competition can be found in `ori-orion/orion_actions` repository. A copy of these actions can be found in 
`action` directory.

`SpeakAndListen` action uses Google Speech to Text API (via SpeechRecognition [https://github.com/Uberi/speech_recognition]) as a primary method for speech to text transcription. When the 
Wi-Fi is unavailable or slow, which is an expected scenario for robotics competitions, we have offline fall-back alternatives,
which uses PocketSphinx (via SpeechRecognition) and WaveNet [https://github.com/buriburisuri/speech-to-text-wavenet]. 
Speech to text does not guarantee a perfect transcription due to noise in the arena, other people speaking at the same time, 
and rare vocabulary / names specific to the tasks. Therefore we take a more robust approach of passing candidate sentences that we expect to hear
and formulate the problem as a classification problem. While the performance will be better if we train it end-to-end, 
we take a more naive but flexible approach of first performing speech to text, and then identifying the closest candidate to the generated text. 
The closest candidate is identified by windowing the generated text to the same length as each candidate, and calculating the Levenshtein distance between the
windowed text and the candidate text. This allows classification to work even when the speech to text is imperfect and only parts of the sequence is transcribed correctly.

Speech recognition is performed asynchronously and in parallel to speech recording; the recorded audio is streamed and when a silence 
(determined by the RMS energy of the signal) is detected, the streaming continues but the frames before the silence is sent to the speech recogniser. Recording is done on a separate thread.

`HotwordListen` action uses Snowboy [https://snowboy.kitt.ai], a cloud service that trains models that can be used for offline hotword detection. 
Example code of how to use Snowboy can be found at: https://github.com/Kitt-AI/snowboy/tree/master/examples/Python
Multiple hotword candidates can be passed to the action so that it would return when any one of them are detected. 

### Identified challenges and further work
There were very limited choices to pre-trained speech recognition models that can be used offline. The ones that we currently use (PocketSphinx and WaveNet)
are not very reliable and is trained on limited vocabulary, so they struggle to recognise some basic commands. 
We worked around this problem by using speech to text for more common words and phrase, and using hotword detection for rarer vocabulary. 
However, it may be worth looking more into more robust speech to text methods, integrating Kaldi or Julius as fall-backs, 
whose language models that are easier to customise to specific tasks than training end-to-end models for each task. 

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
Cloned from https://github.com/buriburisuri/speech-to-text-wavenet in /wavenet directory. Modified `/wavenet/recognize.py` so that the transcriber could be called externally. No other changes.

## Setup

Create a directory to store audio recording:
```
orion_asr/src/tmp
```

For hotword detection, create an account for Snowboy [https://snowboy.kitt.ai/], train the hotword model online 
(takes only a couple of minutes for each hotword), download the `.pmdl` file and place them under `snowboy/resources` directory.

## Run HSR simulation (not required when running on robot)
HSR simulation needs to be started for the talk action to work
```roslaunch hsrb_gazebo_launch hsrb_megaweb2015_world.launch```

## Run Action Server
You can run the speech recognition server with the following:
```
rosrun orion_asr speech_server.py
```

## Test Action Server

You can test the action server by sending a goal to it, using the axclient GUI tool.  

### Make the robot speak
`talk_request_action` is an action that is a part of the HSR by default that allows the robot to speak.
 
```
rosrun actionlib axclient.py /talk_request_action tmc_msgs/TalkRequestAction
```

Example input:
```
data:
 interrupting: False
 queueing: False
 language: 1
 sentence: "Hi. My name is HSR."
```

### SpeakAndListen Action

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
params: ["banana", "tomato", "peach", "toothbrush", "apple"]
timeout: 0.0
```

Example output 2:
```
answer: "Brian Smith"
param: "Smith"
confidence: 0.909090936184
succeeded: True
```

### HotwordListen Action

```
rosrun actionlib axclient.py /hotword_listen orion_actions/HotwordListenAction
```

Example input 1:
```
timeout: 0.0
```

Example output 1:
```
answer: "Brian Smith"
param: "Smith"
confidence: 0.909090936184
succeeded: True
```