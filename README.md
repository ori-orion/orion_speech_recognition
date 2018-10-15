# orion-speech-recognition
Repo for speech recognition capabilities for the ORIon robot

## Using the simulator or standalone ros master (ignore when using HSR)

When using simulator you need to start several services and action servers

Speech Recognition:
```
roslaunch tmc_rosjulius speech_recognition.launch lm_locale:='en' lm_grammar_mode:=true dic_list:='/etc/opt/tmc/robot/conf.d/dics/wrc_grammar_en/dic_list.txt'
```

Speech Synthesize:
```
roslaunch tmc_talk_action_simulator talk_action_simulator.launch
```

## Setting up the right dictionaries for the grammar

Adding the WRC dictionary:
```
 rosservice call /hsrb/voice/add_dictionary 1 1 `rospack find orion_hri`/dics/wrc_grammar_en/wrc_bring_me '[]' "/etc/opt/tmc/robot/conf.d/dics/wrc_grammar_en"
```



De-activate other dictionaries:
```
rosservice call /hsrb/voice/activate_dictionaries '{names: [grammar_sample], active: False}'
```
Note: adjust name of the dictionary, here: `grammar_sample`

Checking the status:
```
rosservice call /hsrb/voice/list_dictionaries '{with_refresh: False}'
```

## Running the bring-me action server

Starting the server:
```
rosrun orion_hri bring_me_server.py
```

Calling the action server using an action client:
```
rosrun actionlib axclient.py /wait_for_instruction orion_hri/WaitForInstructionAction
```
Note: No arguments are given. The result is a list of objects.


## Running the wait-for-confirmation action server

This is a generic server that asks a question and waits for an answer. 

Starting the server:
```
rosrun orion_hri wait_for_confirmation_server.py
```

Calling the action server using an action client:
```
rosrun actionlib axclient.py /wait_for_confirmation orion_hri/WaitForConfirmationAction
```
Example arguments for the Goal:
```
question: 'Hello, can I help you?'
positive_answers: ['yes please']
negative_answers: ['no thanks']
timeout: 0.0
```
Note that the positive and negative answers _must_ be included in the grammar model. The result is a boolean flag called `is_confirmed`.

## Rebuilding the grammar

```
cd ~/catkin_ws/src/orion_speech_recognition/orion_hri/dics
rosrun tmc_rosjulius mkdfa.sh wrc_grammar
sudo cp -r ~/catkin_ws/src/orion_hri/dics/wrc_grammar_en/ /etc/opt/tmc/robot/conf.d/dics/
```








