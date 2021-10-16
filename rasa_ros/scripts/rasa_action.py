#!/usr/bin/env python3
# coding: utf-8
import os
import rospkg

class RasaAction():
    def __init__(self):
        rospack = rospkg.RosPack()
        os.chdir(rospack.get_path("rasa_ros"))
        os.system("rasa run actions --port 5055")

if __name__ == "__main__":
    RasaAction()
