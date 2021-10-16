# OpenSource4NaturalHRInteraction

## About
Open Source System Integration Towards Natural Interaction with Robots

This repository contains an open source robot interaction architecture to enable natural interacton with a reobot agent. The system is built on a ROS platform and it integrates ESPnet speech recognizer (Watanabe et al. 2018 https://arxiv.org/abs/1804.00015), Rasa dialogue model (https://rasa.com/) and Nao Robot (https://www.softbankrobotics.com/emea/en/nao)

## Setup environment
These programs are run on 
* Ubuntu 18.04
* ROS melodic
* Cuda 10.1
* CUDNN 7.6.5

1. Install ROS melodic
* Please refer to install guide like [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

2. Install necessary modules
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python-catkin-tools python-gobject python-pip python3-pip ros-melodic-jsk-tools ros-melodic-jsk-common
pip install --upgrade pip
pip3 install --upgrade pip
```

3. Install Rasa
```
pip3 install testresources==2.0.1
pip3 install nltk==3.6.2
pip3 install regex==2020.9.27
pip3 install rasa==2.7.1
```

4. Install ESPnet
```
sudo apt-get install cmake sox libsndfile1-dev ffmpeg flac nkf python3-venv
git clone https://github.com/espnet/espnet
cd espnet/tools
./setup_venv.sh $(command -v python3)
make CPU_ONLY=0
echo "export PATH=$PATH:$HOME/espnet/tools/venv/bin" >> ~/.bashrc
```
For more information, please refer to [here](https://espnet.github.io/espnet/installation.html).

5. Install ``Python NAOqi SDK``
* You can download it from [here](https://www.softbankrobotics.com/emea/en/support/nao-6/downloads-softwares/former-versions?os=49&category=39). 
Please change the tab to SDKs. Version < 2.5.5 may cause error.
  * Please unzip the downloaded file.
  * Please create pynaoqi folder in your home directory.
  * Then put the file under your pynaoqi folder.

6. Write environment variables in your ``.bashrc``
```
export PYTHONPATH=$HOME/pynaoqi/pynaoqi-python2.7-2.8.6.23-linux64-20191127_152327/lib/python2.7/site-packages:$PYTHONPATH
export NAO_IP="169.254.XX.YYY"
rossetip
```

## Installation

Follow the below commands.
```
source ~/.bashrc
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
【TODO】git clone https://github.com/ayfujii-aist/robot_interaction.git
bash robot_interaction/setup.bash
cd .. # catkin_ws
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## How to use

If you want to use with Nao v6
* Check your "network_interface" by ```ifconfig```
```
roscore
roslaunch nao_v6 nao_v6.launch network_interface:=wlan0
roslaunch esp_rasa_nao esp_rasa_nao.launch
```

You can also test without Nao.
In this case, you may need to change the roslaunch parameters.
```
roslaunch esp_rasa_pc esp_rasa_pc.launch
```

#### Change the roslaunch parameters
* Check your pc microphone channels, depth, and sample rate by using ```pactl list short sinks```
  * Please change "n_channel", "depth" and "sample_rate" 

* Check your card and device number of microphone by using ```arecord -l```
  * Please change "device" as "hw:[card number],[device number]"

* Change "volume_threshold" for volume detection according to your environment.

#### Not training rasa every roslaunch
* Set "train_nlu" and "train_story" false.

````
roslaunch esp_rasa_nao esp_rasa_nao.launch train_nlu:=false train_story:=false
````
