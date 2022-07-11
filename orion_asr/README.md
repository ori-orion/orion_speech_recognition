# General Speech Recognition
Allows general speech recognition using Speech to Text models and Snowboy hotword detection. 

Developed by Shu Ishida, Mia Mijovic and John Lee - contact ishida_at_robots.ox.ac.uk for enquiries.

## Actions
There are two actions associated with it - `SpeakAndListen.action` and `AskPersonName.action`. The version used for the 
RoboCup 2022 competition can be found in `ori-orion/orion_actions` repository. A copy of these actions can be found in 
`action` directory.

`SpeakAndListen` action uses Google Speech to Text API (via SpeechRecognition [https://github.com/Uberi/speech_recognition]) as a primary method for speech to text transcription. When the 
Wi-Fi is unavailable or slow, which is an expected scenario for robotics competitions, we have offline fall-back alternatives,
which uses [Vosk](https://alphacephei.com/vosk/). 
Speech to text does not guarantee a perfect transcription due to noise in the arena, other people speaking at the same time, 
and rare vocabulary / names specific to the tasks. Therefore we take a more robust approach of passing candidate sentences that we expect to hear
and formulate the problem as a classification problem. While the performance will be better if we train it end-to-end, 
we take a more naive but flexible approach of first performing speech to text, and then identifying the closest candidate to the generated text. 

We have two algorithms for determining the closest candidate; Levenshtein and Sysnet similarity. Levenshtein measures the number of single letter changes that are required to change the candidate into the generated text. Fewer changes mean a closer match. On the other hand, the Synset approach looks at the meaning of the words to find the closest match rather than just the letter arrangement. Synset works better for semantic similarities but Levenshtein works better for phonetic similarity and spelling mistakes. 

Speech recognition is performed asynchronously and in parallel to speech recording; the recorded audio is streamed and when a silence 
(determined by the RMS energy of the signal) is detected, the streaming continues but the frames before the silence is sent to the speech recogniser. Recording is done on a separate thread.

`AskPersonName` action specialises at detecting person names.

It also listens to hotwords (e.g. "hey robot" and "I'm ready") and publishes to a topic `/hotword`. It uses a hotword detector called [Porcupine](https://picovoice.ai/docs/quick-start/console-porcupine/) which is a part of the library `picovoice`. 
Custom hotwords can be added by creating an account and downloading a custom model.

The speech recorder saves outputs to the directory `tmp`.
The speech recorder starts when you either call the `SpeakAndListen` or `AskPersonName`, and stops after completion. 
Alternatively, you can manually start and stop recording by calling `recording_start` and `recording_stop` services with an empty message.

### Identified challenges and further work
Vosk works pretty well for offline speech recognition. 
However, there are better speech recognisers released every year so it is worth looking into alternatives and replacing current methods if need be (after cross-evaluation) or adding these methods as alternative methods.
Noise reduction, detection of door-knocking sounds and sound direction detection (using multiple microphones and applying something like independent component analysis or principal component analysis) may be a good way to do it. 

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
pyyaml
rospkg
matplotlib
logmmse
```

## WaveNet
Cloned from https://github.com/buriburisuri/speech-to-text-wavenet in /wavenet directory. Modified `/wavenet/recognize.py` so that the transcriber could be called externally. No other changes.

## Setup

Please read the `setup.md` for setup instructions for ROS and the catkin workspace.

Create a directory to store audio recording:
```
orion_asr/src/tmp
```

For hotword detection, create an account for Snowboy [https://snowboy.kitt.ai/], train the hotword model online 
(takes only a couple of minutes for each hotword), download the `.pmdl` file and place them under `snowboy/resources` directory.

## Run HSR simulation (not required when running on robot)
HSR simulation needs to be started for the talk action to work
```
roslaunch hsrb_gazebo_launch hsrb_megaweb2015_world.launch
```

## Run Action Server
You can run the speech recognition server with the following:
```
rosrun orion_asr speech_server.py
```

## Test Action Server

You can test the action server by sending a goal to it, using the axclient GUI tool.  

### Make the robot speak
`talk_request_action` is an action that is a part of the HSR by default that allows the robot to speak.
 
 **ROS Noetic**
```
rosrun actionlib_tools axclient.py /talk_request_action tmc_msgs/TalkRequestAction
```
 
** ROS Melodic**
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

### Setting the similarity measure
The default similarity measure has been set to synset, it can be changed to Levenshtein through a parameter in the Record method of the ASR object (found within recogniser.py). For example: 

```
asr.record(gen, rec.config, 'Levenshtein')
```

### SpeakAndListen Action

```
rosrun actionlib_tools axclient.py /speak_and_listen orion_actions/SpeakAndListenAction
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

### AskPersonName Action

```
rosrun actionlib_tools axclient.py /ask_person_name orion_actions/AskPersonNameAction
```

Example input 1:
```
question: "What's your name?"
timeout: 0.0
```

Example output 1:
```
answer: "Brian Smith"
confidence: 0.909090936184
```
