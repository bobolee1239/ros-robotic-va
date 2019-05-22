# -*- coding: utf-8 -*-
import pyaudio
import librosa
import numpy as np
import sounddevice as sd
from scipy import signal
import os
import platform

if platform.system().lower() == 'windows':
    WAV_DIRECTORY = 'assets\\wav'
else:
    WAV_DIRECTORY = './assets/wav'
	
def load(audio_path):
    y, sr = librosa.load(os.path.join(WAV_DIRECTORY, audio_path),sr = None)
    return y , sr

if __name__ == '__main__':
    a=1
    while(a>0):
        a=int(input("請輸入case number(1~4,0=Exit):"))
        if a==1:
            x , fs=load('JBL.wav')
            g1=x.data
            shape=x.data.shape
            g1=np.reshape(g1,(shape[0],1))*0.2
            x , fs=load('JBL.wav')
            g2=x.data
            shape=x.data.shape
            g2=np.reshape(g2,(shape[0],1))*0.2
            x , fs=load('JBR.wav')
            g3=x.data
            shape=x.data.shape
            g3=np.reshape(g3,(shape[0],1))*0.2
            x , fs=load('JBR.wav')
            g4=x.data
            shape=x.data.shape
            g4=np.reshape(g4,(shape[0],1))*0.2
            zero=np.zeros((shape[0],1))
            o2=np.concatenate((g2,g1),axis=1)
            o3=np.concatenate((g4,g3),axis=1)
            o=np.concatenate((zero,zero),axis=1)
            out1=np.concatenate((o,o),axis=1)
            out2=np.concatenate((o2,o3),axis=1)
            output=np.concatenate((out1,out2),axis=1)
            sd.play(output,fs)
        elif a==2:
            x , fs=load('XTC1.wav')
            g1=x.data
            shape=x.data.shape
            g1=np.reshape(g1,(shape[0],1))*0.8
            x , fs=load('XTC2.wav')
            g2=x.data
            shape=x.data.shape
            g2=np.reshape(g2,(shape[0],1))*0.8
            x , fs=load('XTC3.wav')
            g3=x.data
            shape=x.data.shape
            g3=np.reshape(g3,(shape[0],1))*0.8
            x , fs=load('XTC4.wav')
            g4=x.data
            shape=x.data.shape
            g4=np.reshape(g4,(shape[0],1))*0.8
            zero=np.zeros((shape[0],1))
            o2=np.concatenate((g2,g1),axis=1)
            o3=np.concatenate((g4,g3),axis=1)
            o=np.concatenate((zero,zero),axis=1)
            out1=np.concatenate((o,o),axis=1)
            out2=np.concatenate((o2,o3),axis=1)
            output=np.concatenate((out1,out2),axis=1)
            sd.play(output,fs)
        elif a==3:
            x , fs=load('21.wav')
            g1=x.data
            shape=x.data.shape
            g1=np.reshape(g1,(shape[0],1))*0.6
            x , fs=load('22.wav')
            g2=x.data
            shape=x.data.shape
            g2=np.reshape(g2,(shape[0],1))*0.6
            x , fs=load('23.wav')
            g3=x.data
            shape=x.data.shape
            g3=np.reshape(g3,(shape[0],1))*0.6
            x , fs=load('24.wav')
            g4=x.data
            shape=x.data.shape
            g4=np.reshape(g4,(shape[0],1))*0.6
            zero=np.zeros((shape[0],1))
            o2=np.concatenate((g2,g1),axis=1)
            o3=np.concatenate((g4,g3),axis=1)
            o=np.concatenate((zero,zero),axis=1)
            out1=np.concatenate((o,o),axis=1)
            out2=np.concatenate((o2,o3),axis=1)
            output=np.concatenate((out1,out2),axis=1)
            sd.play(output,fs)
        elif a==4:
            x , fs=load('51.wav')
            g1=x.data
            shape=x.data.shape
            g1=np.reshape(g1,(shape[0],1))
            x , fs=load('52.wav')
            g2=x.data
            shape=x.data.shape
            g2=np.reshape(g2,(shape[0],1))
            x , fs=load('53.wav')
            g3=x.data
            shape=x.data.shape
            g3=np.reshape(g3,(shape[0],1))
            x , fs=load('54.wav')
            g4=x.data
            shape=x.data.shape
            g4=np.reshape(g4,(shape[0],1))
            zero=np.zeros((shape[0],1))
            o2=np.concatenate((g2,g1),axis=1)
            o3=np.concatenate((g4,g3),axis=1)
            o=np.concatenate((zero,zero),axis=1)
            out1=np.concatenate((o,o),axis=1)
            out2=np.concatenate((o2,o3),axis=1)
            output=np.concatenate((out1,out2),axis=1)
            sd.play(output,fs)
        else:
            break
