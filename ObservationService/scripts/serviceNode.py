#!/usr/bin/env python

import librosa
import librosa.display
from scipy import fft, arange
import numpy as np
from numpy.fft import rfft
from numpy import argmax, log, diff, nonzero
from scipy.signal import correlate, blackmanharris, periodogram, butter, filtfilt
import matplotlib.pyplot as plt
from scipy.io import wavfile
import os
from librosa.core.spectrum import _spectrogram
from librosa import util
from scipy.signal import welch
import pandas as pd
import time
import sys
import rospy
from scipy.signal.signaltools import wiener
from ObservationService.srv import Observation
from subprocess import call
import rospy
import pyaudio
import wave
import soundfile as sf
from python_speech_features import delta
from python_speech_features import logfbank
from python_speech_features import mfcc
import scipy.io.wavfile as wav

class ObservationServer:

	def __init__(self):
		coffee_mug_bang = ['Coffee_mug_bang_1.wav']
		coffee_mug_slide = ['Coffee_mug_drag_1.wav']
		plastic_cup_bang = ['Plastic_cup_bang_1.wav','Plastic_cup_bang_2.wav','Plastic_cup_bang_3.wav']
		plastic_cup_slide = ['Plastic_cup_drag_1.wav','Plastic_cup_drag_2.wav','Plastic_cup_drag_3.wav']

		self.coffee_mug_bang = []
		self.coffee_mug_slide = []
		self.plastic_cup_bang = []
		self.plastic_cup_slide = []
		self.i=0
		self.average_coffee_mug_action1 = np.load("average_coffee_mug_action1.npy")
		self.average_coffee_mug_action2 = np.load("average_coffee_mug_action2.npy")
		self.average_plastic_cup_action1 = np.load("average_plastic_cup_action1.npy")
		self.average_plastic_cup_action2 = np.load("average_plastic_cup_action2.npy")
		self.average_paper_base_action1 = np.load("average_paper_base_action1.npy")
		self.average_paper_base_action2 = np.load("average_paper_base_action2.npy")
		self.average_pringles_action1 = np.load("average_pringles_action1.npy")
		self.average_pringles_action2 = np.load("average_pringles_action2.npy")
		self.action1 = [self.average_pringles_action1, self.average_coffee_mug_action1, 
						self.average_plastic_cup_action1, self.average_paper_base_action1]
		self.action2 = [self.average_pringles_action2, self.average_coffee_mug_action2, 
						self.average_plastic_cup_action2, self.average_paper_base_action2]

		print("Please Wait while service is set up")

		for file in coffee_mug_bang:
			filename = '/home/jihirshu/workspaces/ObjectDetection_ws/src/ObservationService/audio_data/'+file
			centroid = self.get_Centroid(filename)
			observation = None
			if (centroid <= 1200):
				observation = 0
			elif (centroid <= 2200):
				observation = 1
			else:
				observation = 2

			self.coffee_mug_bang.append(observation)

		for file in coffee_mug_slide:
			filename = '/home/jihirshu/workspaces/ObjectDetection_ws/src/ObservationService/audio_data/'+file
			centroid = self.get_Centroid(filename)
			observation = None
			if (centroid <= 1200):
				observation = 0
			elif (centroid <= 2200):
				observation = 1
			else:
				observation = 2

			self.coffee_mug_slide.append(observation)			
		
		for file in plastic_cup_bang:
			filename = '/home/jihirshu/workspaces/ObjectDetection_ws/src/ObservationService/audio_data/'+file
			centroid = self.get_Centroid(filename)
			observation = None
			if (centroid <= 1200):
				observation = 0
			elif (centroid <= 2200):
				observation = 1
			else:
				observation = 2

			self.plastic_cup_bang.append(observation)

		for file in plastic_cup_slide:
			filename = '/home/jihirshu/workspaces/ObjectDetection_ws/src/ObservationService/audio_data/'+file
			centroid = self.get_Centroid(filename)
			observation = None
			if (centroid <= 1200):
				observation = 0
			elif (centroid <= 2200):
				observation = 1
			else:
				observation = 2

			self.plastic_cup_slide.append(observation)

		print("Service is now ready.")

		print(self.coffee_mug_bang)
		print(self.coffee_mug_slide)
		print(self.plastic_cup_bang)
		print(self.plastic_cup_slide)

	def FeatureSpectralFlux(self, X, f_s):

	    # difference spectrum (set first diff to zero)
	    X = np.c_[X[:, 0], X]
	    # X = np.concatenate(X[:,0],X, axis=1)
	    afDeltaX = np.diff(X, 1, axis=1)

	    # flux
	    vsf = np.sqrt((afDeltaX**2).sum(axis=0)) / X.shape[0]

	    return (vsf)

	def parabolic(self, f, x):
	    """Quadratic interpolation for estimating the true position of an
	    inter-sample maximum when nearby samples are known.

	    f is a vector and x is an index for that vector.

	    Returns (vx, vy), the coordinates of the vertex of a parabola that goes
	    through point x and its two neighbors.

	    Example:
	    Defining a vector f with a local maximum at index 3 (= 6), find local
	    maximum if points 2, 3, and 4 actually defined a parabola.

	    In [3]: f = [2, 3, 1, 6, 4, 2, 3, 1]

	    In [4]: parabolic(f, argmax(f))
	    Out[4]: (3.2142857142857144, 6.1607142857142856)

	    """
	    # Requires real division.  Insert float() somewhere to force it?
	    xv = 1 / 2 * (f[x - 1] - f[x + 1]) / (f[x - 1] - 2 * f[x] + f[x + 1]) + x
	    yv = f[x] - 1 / 4 * (f[x - 1] - f[x + 1]) * (xv - x)
	    return (xv, yv)

	def spectral_centroid(self, x, samplerate=44100):
	    magnitudes = np.abs((np.fft.rfft(x))) # magnitudes of positive frequencies
	    length = len(x)
	    freqs = np.abs(np.fft.fftfreq(length, 1.0/samplerate)[:length//2+1]) # positive frequencies
	    return np.sum(magnitudes*freqs) / np.sum(magnitudes)

	def FeatureSpectralRolloff(self, X, f_s, kappa=0.85):
	    X = np.cumsum(X, axis=0) / X.sum(axis=0, keepdims=True)
	    vsr = np.argmax(X >= kappa, axis=0)
	    # convert from index to Hz
	    vsr = vsr / (X.shape[0] - 1) * f_s / 2

	    return (vsr)

	def freq_from_fft(self, sig, fs):
	    """
	    Estimate frequency from peak of FFT
	    """
	    # Compute Fourier transform of windowed signal
	    windowed = sig * blackmanharris(len(sig))
	    f = rfft(windowed)

	    # Find the peak and interpolate to get a more accurate peak
	    i = argmax(abs(f))  # Just use this for less-accurate, naive version
	    true_i = parabolic(log(abs(f)), i)[0]

	    # Convert to equivalent frequency
	    return fs * true_i / len(windowed)

	def TrimSignal(self, y, threshold=0):
	    i=0
	    while(i < len(y)):
	        z = y[i:i+100]
	        if (np.sum(z) > threshold):
	            break
	        i=i+100
	    y = y[i:]

	    # print("Trimmed samples from front : ", i)

	    i = len(y) - 1
	    count=0
	    while(i >= 0):
	        z = y[i-100:i]
	        if (np.sum(z) > threshold):
	            break
	        count=count+100
	        i=i-100
	    y = y[:i]
	    # print("Trimmed samples from end : ", count)

	    return y


	def getcentriodandspread(self, samples):
	    ind = (np.arange(1, len(samples) + 1)) * (400/(2.0 * len(samples)))
	    Xt = samples.copy()
	    Xt = Xt / Xt.max()
	    NUM = np.sum(ind * Xt)
	    DEN = np.sum(Xt) + 13
	    # Centroid:
	    C = (NUM / DEN)
	    # Spread:
	    S = np.sqrt(np.sum(((ind - C) ** 2) * Xt) / DEN)
	    # Normalize:
	    C = C / (220 / 2.0)
	    S = S / (220 / 2.0)
	    print("centroid",C)
	    #print("spread",S)


	def get_Centroid(self, file):
	    y, sr = librosa.load(file)
	    # y = wiener(y, (10))
	    y = self.TrimSignal(y)
	    y = np.asfortranarray(y)
	 
	    centroid = self.spectral_centroid(y, sr)

	    return centroid

	def get_rms(self, file):
		y, sr = librosa.load(file)
		print(len(y))
		y = self.TrimSignal(y)
		y = np.asfortranarray(y)
		# print(len(y))
		# p = pyaudio.PyAudio()
		# wf = wave.open('trimmed', 'wb')
		# wf.setnchannels(4)
		# wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
		# wf.setframerate(sr)
		# wf.writeframes(y)
		# wf.close()
		rms = np.mean(librosa.feature.rms(y))
		return rms

	def classify(self, action):
		rate=None
		sig = None
		fbank_feat = None
		averages = None
		if (action == 0):   #action 1
			print("action 1")
			(rate, sig) = wav.read('/home/jihirshu/filtered_robot_recording_0.wav')
			fbank_feat = logfbank(sig, rate)
			fbank_feat = fbank_feat[-140:, :]
			averages = self.action1
		else: 			    #action 2
			print("action2")
			(rate, sig) = wav.read('/home/jihirshu/robot_recording_0.wav')
			sig = sig[2000:]
			fbank_feat = logfbank(sig, rate)
			fbank_feat = fbank_feat[-300:-150, :]
			averages = self.action2

		distances = []
		for x in averages:
			diff = np.absolute(fbank_feat - x)
			distances.append(np.mean(diff, axis=(0, 1)))

		return np.argmin(distances)


	def callback(self, req):
		state = req.state
		action = req.action
		centroid = None
		rms = None
		# # plastic cup slide 
		# if (state == 0) and (action == 0):
		# 	observation = self.plastic_cup_slide[np.random.randint(len(self.plastic_cup_slide))]
		# # plastic cup bang
		# elif (state == 0) and (action == 1):
		# 	observation = self.plastic_cup_bang[np.random.randint(len(self.plastic_cup_bang))]
		# # coffee mug slide
		# elif (state == 1) and (action == 0):
		# 	observation = self.coffee_mug_slide[np.random.randint(len(self.coffee_mug_slide))]
		# # coffee mug bang
		# elif (state == 1) and (action == 1):
		# 	observation = self.coffee_mug_bang[np.random.randint(len(self.coffee_mug_bang))]
		# else:
		# 	print("Incorrect combination of state and action values")
		# 	sys.exit()
		# print("updated")
		# centroid = self.get_Centroid('/home/jihirshu/robot_recording_0.wav')
		# centroid =  float(centroid)
		# rms = self.get_rms('/home/jihirshu/robot_recording_0.wav')
		# rms = float(rms)

		observation = self.classify(action)

		print(observation, state, action, self.i)
		self.i=self.i+1
		return observation


def main():
	rospy.init_node	('Observation_Server')
	obs_server = ObservationServer()	
	getObservation = rospy.Service('Observations', Observation, obs_server.callback)
	rospy.spin()

if __name__=='__main__':
	main()