#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import wave
import os.path
import datetime
import array
import numpy as np
import time
import math

import soundfile
from espnet_model_zoo.downloader import ModelDownloader
from espnet2.bin.asr_inference import Speech2Text

class MakeWav4Espnet(object):
    def __init__(self):
        self.savedir = os.path.expanduser('~') + "/wav/espnet/"
        if not os.path.exists(self.savedir):
            os.makedirs(self.savedir)
        d = ModelDownloader()
        self.sample_rate = rospy.get_param("~sample_rate", 48000)
        self.model_sample_rate=16000
        self.language = rospy.get_param("~language", "en-US")
        if self.language=="en-US":
            self.model_name = "kamo-naoyuki/librispeech_asr_train_asr_conformer6_n_fft512_hop_length256_raw_en_bpe5000_scheduler_confwarmup_steps40000_optim_conflr0.0025_sp_valid.acc.ave"
            #self.model_name = "kamo-naoyuki/mini_an4_asr_train_raw_bpe_valid.acc.best"
            #self.model_name = "Shinji Watanabe/spgispeech_asr_train_asr_conformer6_n_fft512_hop_length256_raw_en_unnorm_bpe5000_valid.acc.ave"
        elif self.language=="ja-JP":
            self.model_name = "Shinji Watanabe/laborotv_asr_train_asr_conformer2_latest33_raw_char_sp_valid.acc.ave"
        else:
            print("Please add your language model in make_wav4espnet.py")
            exit(1)
        self.speech2text = Speech2Text(**d.download_and_unpack(self.model_name))
        print("Espnet init")

        rospy.Subscriber("/audio_detection", String, self.topic_cb)
        self.pub_state = rospy.Publisher("/audio_recognition_status", String, queue_size=1)
        # use rasa
        self.pub_content =  rospy.Publisher("/speech_content", String, queue_size=1)
        # for debug
        #self.pub_content =  rospy.Publisher("/rasa_ros/agent_speech", String, queue_size=1)
        self.pub_speech = rospy.Publisher("/rasa_ros/agent_speech", String, queue_size=1)

    def topic_cb(self, msg):
        if msg.data=="Detect":
            print("start espnet detection")
            self.sub=rospy.Subscriber("/audio_for_espnet", AudioData, self.cb_once)
            time.sleep(0.1)

    def cb_once(self, msg):
        # sample_rate x 2 x0.5s
        if len(msg.data) < self.sample_rate:
            print("too short")
            self.pub_state.publish("Restart")
            self.sub.unregister()
            return
        print("Save wav file")
        data = array.array("h", bytes(msg.data)).tolist()
        # down sampling
        num=0
        chan_data=[]
        for i in range (len(data)):
            new_num=i * 1.0 * self.model_sample_rate / self.sample_rate
            if math.floor(new_num) >= num:
                chan_data.append(data[i])
                num=math.floor(new_num)+1
        data2 = array.array("h", chan_data).tobytes()

        now = datetime.datetime.now()
        wavFileName = self.savedir + now.strftime('%Y%m%d_%H%M%S') + ".wav"
        wf = wave.open(wavFileName, "wb")
        wf.setnchannels(1)
        wf.setframerate(self.model_sample_rate)
        wf.setsampwidth(2)
        wf.setnframes(1)
        audio=array.array("b", data2)
        wf.writeframes(audio)
        wf.close()

        speech, _ = soundfile.read(wavFileName)
        nbests = self.speech2text(speech)
        text, *_ = nbests[0]
        print(text)

        if len(text) <=1:
            print("speech recognition failure")
            speech = String()
            if self.language=="en-US":
                speech.data = "I cannot understand"
            elif self.language=="ja-JP":
                speech.data = "ごめんなさい、うまく聞き取れなかったです"
            self.pub_speech.publish(speech)

        else:
            content=String()
            content.data=text
            self.pub_content.publish(content)

        self.sub.unregister()
        print("unregister")
        return

if __name__ == '__main__':
    rospy.init_node('make_wav4espnet')
    MakeWav4Espnet()
    rospy.spin()
