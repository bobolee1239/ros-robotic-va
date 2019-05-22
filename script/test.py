# -*- coding: utf-8 -*-
import pyaudio
import librosa
import numpy as np
import sounddevice as sd
from scipy import signal

def load(audio_path):
    y, sr = librosa.load(audio_path,sr = None)
    return y , sr

if __name__ == '__main__':
    a=int(input("which speaker(speaker=1,2,5,6,7,8  0=exit!):"))
    while a>0:
        if a==0:
            break;
        x , fs=load('W.wav')
        w=x.data
        shape=x.data.shape
        w=np.reshape(w,(shape[0],1))
        x , fs=load('Q.wav')
        q=x.data
        shape=x.data.shape
        q=np.reshape(q,(shape[0],1))
        #1 2 5 6 7 8#
        o1=np.concatenate((w,q),axis=1)
        o2=np.concatenate((q,q),axis=1)
        z=np.concatenate((o2,o2),axis=1)
        o=np.concatenate((o1,o2),axis=1)
        out=np.concatenate((z,o),axis=1)
        output=out
        temp=out[:,a-1]
        output[:,0]=temp
        output[:,a-1]=out[:,0]
        sd.play(output,fs) 
        a=int(input("which speaker(speaker=1,2,5,6,7,8  0=exit!):"))