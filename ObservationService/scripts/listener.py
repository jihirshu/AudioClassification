#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import Bool
import pyaudio
import wave
from threading import Lock
import os
import subprocess
class Recorder:
    
    def __init__(self):
        self.lock = Lock()
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 2
        self.RATE = 44100
        self.RECORD_SECONDS = 5
        self.WAVE_OUTPUT_FILENAME = "robot_recording_"
        self.counter = 0
        self.Flag = False



    def enableRecording(self):
        rospy.init_node('listener', anonymous=True)
        
        print("Service is now enabled")
        # print(rospy.get_param("/AudioClassification_Record"))
        recordedSomething=False
        frames = []
        p = pyaudio.PyAudio()
        
        while (True):
            # if (rospy.get_param("/AudioClassification_Stop")):
                # print("Exiting")
                # break;
            if (rospy.get_param("/AudioClassification_Record")):
                stream = p.open(format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.CHUNK)
                while (rospy.get_param("/AudioClassification_Record")):
                    data = stream.read(self.CHUNK)
                    frames.append(data)                
        

                recordedSomething = True
            # if (not rospy.get_param("/AudioClassification_Record")) and (recordedSomething == True):
                print("Recording Finished")
                stream.stop_stream()
                stream.close()
                # p.terminate()
                filename = self.WAVE_OUTPUT_FILENAME + "0.wav"
                self.counter = self.counter + 1
                print(filename)
                wf = wave.open(filename, 'wb')
                wf.setnchannels(self.CHANNELS)
                wf.setsampwidth(p.get_sample_size(self.FORMAT))
                wf.setframerate(self.RATE)
                wf.writeframes(b''.join(frames))
                wf.close()
                print("concluded")
                frames = []
                recordedSomething = False
                # os.system('source ~/workspaces/AudioClassification_Movo/ObservationService/scripts/myenv/bin/activate'
                os.system('~/workspaces/AudioClassification_Movo/ObservationService/scripts/myenv/bin/python ~/workspaces/AudioClassification_Movo/ObservationService/scripts/filter.py')

    def record(self):
        p = pyaudio.PyAudio()

        stream = p.open(format=self.FORMAT,
                        channels=self.CHANNELS,
                        rate=self.RATE,
                        input=True,
                        frames_per_buffer=self.CHUNK)

        print("* recording")

        frames = []

        while(self.Flag):
            data = stream.read(self.CHUNK)
            frames.append(data)

        print("* done recording")

        stream.stop_stream()
        stream.close()
        p.terminate()
        filename = self.WAVE_OUTPUT_FILENAME + str(self.counter) + ".wav"
        self.counter = self.counter + 1
        print(filename)
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.CHANNELS)
        wf.setsampwidth(p.get_sample_size(self.FORMAT))
        wf.setframerate(self.RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        print("concluded")
        os.system('source myenv/bin/activate')
        os.system('python3 filter.py')

    def callback(self,data):
        print(data)
        if (not self.Flag and data.data):
            self.Flag = data.data
            self.record()

        else:
            self.Flag = data.data
            print(self.Flag)


    def listener(self):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
        print("running")
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber('chatter', Bool, self.callback)

    # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    recorder = Recorder()
    recorder.enableRecording()
