## Installing ROS and HSR simulator

1. Install ROS – currently ros-kinetic-desktop-full - see http://wiki.ros.org/kinetic/Installation/Ubuntu
2. Add sources following the list of commands bellow
```bash
sudo sh -c 'echo "deb https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/tmc.list'
sudo sh -c 'echo "deb https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu `lsb_release -cs` multiverse main" >> /etc/apt/sources.list.d/tmc.list'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -O - | sudo apt-key add -
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-kinetic-tmc-desktop-full
```
3. Load settings
Add the lines below to your `~/.bashrc`, or create a bash file such as `~/catkin_ws/setup.bash` and source it instead. Note that where this example has `eno1`, you will need to find the appropriate network interface for your device. You can use `netstat -i` to list the network interfaces.
```bash
# please set network-interface
network_if=eno1
 
if [ -e /opt/ros/kinetic/setup.bash ] ; then
    source /opt/ros/kinetic/setup.bash
else
    echo "ROS packages are not installed."
fi
 
export TARGET_IP=$(LANG=C /sbin/ifconfig $network_if | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*')
if [ -z "$TARGET_IP" ] ; then
    echo "ROS_IP is not set."
else
    export ROS_IP=$TARGET_IP
fi
 
export ROS_HOME=~/.ros
alias sim_mode='export ROS_MASTER_URI=http://localhost:11311 export PS1="\[\033[44;1;37m\]<local>\[\033[0m\]\w$ "'
alias hsrb_mode='export ROS_MASTER_URI=http://hsrb.local:11311 export PS1="\[\033[41;1;37m\]<hsrb>\[\033[0m\]\w$ "'
```
4. Launch an existing world to test that everything is working
```bash
roslaunch hsrb_gazebo_launch hsrb_megaweb2015_world.launch
```

### Create catkin workspace
1. Create catkin workspace
Assuming you want to create the workspace called `catkin_ws` under your user directory,
```bash
mkdir -p ~/catkin_ws/src
```

2. Clone repositories
Git clone this repository and the orion actions repository in `~/catkin_ws/src`.
```
cd ~/catkin_ws/src
git clone https://github.com/ori-orion/orion_actions
git clone https://github.com/ori-orion/orion_speech_recognition
```
Alternatively, you could clone these repositories in another directory, create symbolic links and place them in the catkin workspace. 

3. Build catkin workspace
```bash
cd ~/catkin_ws
catkin_make
```

4. Source `devel/setup.bash`
Add the following line to `~/.bashrc`
```bash
if [ -e ~/catkin_ws/devel/setup.bash ] ; then
    source ~/catkin_ws/devel/setup.bash
else
    echo "ROS catkin workspace cannot be found or is not built."
fi
```

### Launch speech server
1. Start ros master
You can either launch the HSR simulation as below,
```bash
roslaunch hsrb_gazebo_launch hsrb_megaweb2015_world.launch
```
or if you just want to try the speech recognition functionality, just start
```roscore```

2. Start speech server
While the above is running, open another terminal. Make sure that the bash scripts in the previous sections are sourced. Then run
```
rosrun orion_asr speech_server.py
```
