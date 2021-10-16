#!/usr/bin/env python3
# coding: utf-8
#from __future__ import unicode_literals
import requests
import os
import glob
import re

import rospy
from std_msgs.msg import String
from rasa_ros.msg import RasaMsg, RasaOrderMsg
import rospkg

import sys

from rasa.shared.nlu.training_data.loading import load_data
from rasa.nlu.model import Trainer
from rasa.nlu import config
from rasa.nlu.model import Interpreter

from rasa.core.agent import Agent
from rasa.core import config as config2
from rasa.core.channels import UserMessage
import asyncio

from rasa.core.utils import AvailableEndpoints

class RasaNLU():

    def __init__(self):
        data_language = rospy.get_param("~data_language", "en-US")
        train_nlu = rospy.get_param("~train_nlu", False)
        train_story = rospy.get_param("~train_story", False)
        rospack = rospkg.RosPack()
        directory = rospack.get_path('rasa_ros') + '/languages/' + data_language +"/"
        if not os.path.exists(directory+"models/nlu"):
            os.makedirs(directory+"models/nlu")
        if not os.path.exists(directory+"models/dialogue"):
            os.makedirs(directory+"models/dialogue")

        if train_nlu==True:
            print("Loading config files for nlu...")
            training_data = load_data(directory + "data.yml")
            trainer = Trainer(config.load(directory + "config.yml"))
            print("Training the model...")
            trainer.train(training_data)
            print("Saving the model...")
            model_directory = trainer.persist(directory + 'models/nlu')
        else:
            list_of_files = glob.glob(directory + 'models/nlu/*')
            model_directory = max(list_of_files, key=os.path.getctime)

        print("Loading the model...")
        self.interpreter = Interpreter.load(model_directory)
        print("NLU init done.")

        self.loop=asyncio.get_event_loop()
        if train_story==True:
            print("Loading domain files for story...")
            agent = Agent(directory + "domain.yml",
                          policies=config2.load(directory + "config.yml"))
            training_data = self.loop.run_until_complete(agent.load_data(directory + "stories.yml"))
            print("Training the story model...")
            agent.train(training_data)
            print("Saving the story model...")
            story_directory = directory + 'models/dialogue'
            agent.persist(story_directory)
        else:
            story_directory = directory + 'models/dialogue'

        print("Loading the story model...")
        print("story_directory: " + story_directory)
        print("model_directory: " + model_directory)
        _endpoints = AvailableEndpoints.read_endpoints(rospack.get_path('rasa_ros') + '/endpoints.yml')
        self.agent = Agent.load(story_directory, interpreter=model_directory, action_endpoint=_endpoints.action)

        self.pub = rospy.Publisher("/rasa_ros/nlu_detection", RasaMsg, queue_size=1)
        self.pub_agent = rospy.Publisher("/rasa_ros/agent_speech", String, queue_size=1)
        rospy.Subscriber("/speech_content", String, self.topic_cb)
        print("Init all")

    def topic_cb(self, msg):
        question=msg.data
        print("Received sentence : " + question)
        sentence = question.lower()
        print("Calling rasa_nlu...")
        result = RasaMsg()
        response = self.interpreter.parse(question)
        print('*' * 40)
        intent = response.get('intent').get('name')
        print("intent : " + intent)
        confidence = response.get("intent").get("confidence")
        print("confidence : " + str(confidence))
        result.intent = intent
        result.confidence = confidence

        entities = response.get('entities')
        orderList=[]

        try:
            for arg in entities:
                print(arg.get('entity'), ":", arg.get('value'))
                orderList.append(arg.get('value'))
                order = RasaOrderMsg()
                order.entity = str(arg.get("entity"))
                order.value = ascii(arg.get("value")).encode().decode("unicode-escape")
                result.entities.append(order)
            if len(orderList) == 0:
                print("no entities")
        except:
            print("unknown")

        print('*' * 40)
        print()
        self.pub.publish(result)

        UMsg=UserMessage(text=sentence, sender_id="default")
        responses=self.loop.run_until_complete(self.agent.handle_message(UMsg))
        response_text=""
        for response in responses:
            try:
                response_text = response_text + " "+ response["text"]
            except:
                print("No agent answer text")
        if response_text=="":
            response_text="I can not understand."

        response_text = response_text.replace('\n', ' ')
        code_regex = re.compile('[\\"#\$%&\(\)\*\+\-\/:;<=>@\[\]\^_`\{\|\}~「」〔〕“”〈〉『』【】＆＊・（）＄＃＠｀＋￥％]')
        cleaned_text = code_regex.sub('', response_text)
        print("agent response: " + cleaned_text)

        text=String()
        text.data=cleaned_text
        self.pub_agent.publish(text)

if __name__ == "__main__":
    rospy.init_node('rasa_nlu')
    RasaNLU()
    rospy.spin()
