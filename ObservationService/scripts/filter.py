#!/home/jihirshu/workspaces/AudioClassification_Movo/ObservationService/scripts/myenv/bin/python3

import librosa
import noisereduce as nr
import soundfile as sf
# audio_data, sr= librosa.load('/home/jihirshu/robot_recording_0.wav')  # Noise reduction
# # y = wiener(audio_data, (10))
# noisy_part = audio_data[0:10000]
# reduced_noise = nr.reduce_noise(audio_clip=audio_data, noise_clip=noisy_part, verbose=False)  # Visualize
# sf.write('/home/jihirshu/filtered_robot_recording_0.wav', reduced_noise, sr, subtype='PCM_16')

audio_data, sr = librosa.load('/home/jihirshu/robot_recording_0.wav')  # Noise reduction
# y = wiener(audio_data, (10))
audio_data = audio_data[2000:]
noisy_part = audio_data[0:5000]
reduced_noise = nr.reduce_noise(audio_clip=audio_data, noise_clip=noisy_part, verbose=False)  # Visualize
sf.write('/home/jihirshu/filtered_robot_recording_0.wav', reduced_noise, sr, subtype='PCM_16')