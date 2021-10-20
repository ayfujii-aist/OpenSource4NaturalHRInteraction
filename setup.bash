#!/bin/bash

cd .. # catkin_ws
wstool init src
wstool merge -t src src/OpenSource4NaturalHRInteraction/install.rosinstall
wstool update -t src
source /opt/ros/melodic/setup.bash
rosdep install -y -r --from-paths src --ignore-src
sudo apt-get install ros-melodic-nao-meshes
cd src
cd naoqi_dashboard
git remote add kochigami https://github.com/kochigami/naoqi_dashboard.git
git fetch kochigami
git checkout -b modify-for-kinetic kochigami/kochigami-develop
cd .. # catkin_ws/src
cd naoqi_bridge
git remote add kochigami https://github.com/kochigami/naoqi_bridge.git
git fetch kochigami
git checkout -b kochigami-develop kochigami/kochigami-develop
cd .. # catkin_ws/src
cd naoqi_bridge_msgs
git remote add kochigami https://github.com/kochigami/naoqi_bridge_msgs.git
git fetch kochigami
git checkout -b kochigami-develop kochigami/kochigami-develop
cd .. # catkin_ws/src
cd naoqi_driver
git remote add kochigami https://github.com/kochigami/naoqi_driver.git
git fetch kochigami
git checkout -b kochigami-develop kochigami/kochigami-develop
cd ../.. # catkin_ws
catkin init
