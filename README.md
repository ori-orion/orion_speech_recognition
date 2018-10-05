# orion-speech-recognition
Repo for speech recognition capabilities for the ORIon robot

## Using the simulator

When using simulator you need to start several services and action servers

Speech Recognition:
```
roslaunch tmc_rosjulius speech_recognition.launch lm_locale:='en' lm_grammar_mode:=true dict_list:='/etc/opt/tmc/robot/conf.d/dics/wrc_grammar_en/dic_list.txt'
```

Speech Synthesize:
```
roslaunch tmc_talk_action_simulator talk_action_simulator.launch
```

## Setting up the right dictionaries

Adding the WRC dictionary:
```
 rosservice call /hsrb/voice/add_dictionary 1 1 wrc_grammar '[]' "/etc/opt/tmc/robot/conf.d/dics/wrc_grammar_en"
```
Note: adjust the path if necessary.

Removing other dictionaries:
```
rosservice call /hsrb/voice/activate_dictionaries '{names: [grammar_sample], active: False}'
```
Note: adjust name of the dictionary, here: `grammar_sample`

Checking the status:
```
rosservice call /hsrb/voice/list_dictionaries '{with_refresh: False}'
```


## Running the wait-for-instruction action server

Starting the server:
```
rosrun orion_hri wait_for_instruction_server.py
```

Calling the action server using an action client:
```
rosrun actionlib axclient.py /wait_for_instruction orion_hri/WaitForInstructionAction
```
Note: ignore the argument for now.


## Rebuild the grammar

```
cd ~/catkin_ws/src/orion_speech_recognition/orion_hri/dics
rosrun tmc_rosjulius mkdfa.sh wrc_grammar
sudo cp -r ~/catkin_ws/src/orion_hri/dics/wrc_grammar_en/ /etc/opt/tmc/robot/conf.d/dics/
```










