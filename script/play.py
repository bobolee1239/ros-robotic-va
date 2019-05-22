# -*- coding: utf-8 -*-
"""
Created on Wed May 15 16:31:00 2019

@author: User
"""

import pyaudio
import librosa
import numpy as np
import sounddevice as sd
from scipy import signal

def load(audio_path):
    y, sr = librosa.load(audio_path,sr = None)
    return y , sr

if __name__ == '__main__':

    '''x , fs=load('XTC1.wav')
    g51=x.data
    shape=x.data.shape
    g51=np.reshape(g51,(shape[0],1))
    x , fs=load('XTC2.wav')
    g52=x.data
    shape=x.data.shape
    g52=np.reshape(g52,(shape[0],1))'''
    x , fs=load('XTC1.wav')
    g53=x.data
    shape=x.data.shape
    g53=np.reshape(g53,(shape[0],1))
    x , fs=load('XTC2.wav')
    g54=x.data
    shape=x.data.shape
    g54=np.reshape(g54,(shape[0],1))
    x , fs=load('XTC3.wav')
    g55=x.data
    shape=x.data.shape
    g55=np.reshape(g55,(shape[0],1))
    x , fs=load('XTC4.wav')
    g56=x.data
    shape=x.data.shape
    g56=np.reshape(g56,(shape[0],1))
    zero=np.zeros((shape[0],1))
    #1 2 5 6 7 8#
    #o1=np.concatenate((g51,g52),axis=1)
    o2=np.concatenate((g54,g53),axis=1)
    o3=np.concatenate((g56,g55),axis=1)
    o=np.concatenate((zero,zero),axis=1)
    out1=np.concatenate((o,o),axis=1)
    out2=np.concatenate((o2,o3),axis=1)
    output=np.concatenate((out1,out2),axis=1)
    sd.play(output,fs) 
