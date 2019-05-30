#!/usr/bin/env python3
#
#  Copyright (c) 2019 Tsung-Han Brian Lee, Yi-Cheng Hsu
## ------------------------------------------------------
#
## ------------------------------------------------------

import os
import sys
import json
import logging
import librosa
import platform
import numpy as np
import sounddevice as sd
from scipy import signal
from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket

if platform.system().lower() == 'windows':
     WAV_DIRECTORY = 'assets\\wav'
else:
     WAV_DIRECTORY = './assets/wav'

logger = logging.getLogger('AudioServer')

def load(audio_path):
    logger.debug('Loading ' + audio_path)
    try:
        y, sr = librosa.load(os.path.join(WAV_DIRECTORY, audio_path),sr = None)
    except:
        logger.error('Unexpected Error: ' + str(sys.exc_info()))
        
    return y , sr

def sdset():
    for idx, device in enumerate(sd.query_devices()):
        if ('Xonar U7 MKII' in device['name']) and (device['max_output_channels'] == 8) :
            sd.default.device[1] = idx
            break

##
#   Audio Server
##
class AudioServer(WebSocket):
    def handleMessage(self):
        logger.debug('received: {}'.format(self.data))
        msg = json.loads(self.data)
        sdset()
        if 'songName' not in msg:
            speech  = msg['content']
            fs      = msg['fs']
            # [TODO...] play speech
            x       = np.array(speech)
            shape   = x.shape
            x       = np.reshape(x,(shape[0],1))
            z       = np.zeros((shape[0],1),np.float32)
            zero    = np.concatenate((z,z),axis=1)
            outzero = np.concatenate((zero,zero),axis=1)
            o1      = np.concatenate((x,z),axis=1)
            o2      = np.concatenate((z,x),axis=1)
            outo    = np.concatenate((o1,o2),axis=1)
            out     = np.concatenate((outzero,outo),axis=1)
            sd.play(out,fs)
            logger.debug('play speech')
            #raise NotImplementedError
        else:
            songName = msg['songName']
            #  [TODO...] play song with corresponding effect
            if msg['effect'] == 'xtalk':
                x1 , fs = load(songName+'_'+msg['effect']+'_1.wav')
                x2 , fs = load(songName+'_'+msg['effect']+'_2.wav')
                x3 , fs = load(songName+'_'+msg['effect']+'_3.wav')
                x4 , fs = load(songName+'_'+msg['effect']+'_4.wav')
                shape   = x1.data.shape
                x1      = np.reshape(x1,(shape[0],1))
                x2      = np.reshape(x2,(shape[0],1))
                x3      = np.reshape(x3,(shape[0],1))
                x4      = np.reshape(x4,(shape[0],1))
                z       = np.zeros((shape[0],1),np.float32)
                zero    = np.concatenate((z,z),axis=1)
                outzero = np.concatenate((zero,zero),axis=1)
                o1      = np.concatenate((x1,x2),axis=1)
                o2      = np.concatenate((x3,x4),axis=1)
                outo    = np.concatenate((o1,o2),axis=1)
                out     = np.concatenate((outzero,outo),axis=1)
                sd.play(out,fs)
                logger.debug('play xtc')
            elif msg['effect'] == 'vsurround':
                x1 , fs = load(songName+'_'+msg['effect']+'_1.wav')
                x2 , fs = load(songName+'_'+msg['effect']+'_2.wav')
                x3 , fs = load(songName+'_'+msg['effect']+'_3.wav')
                x4 , fs = load(songName+'_'+msg['effect']+'_4.wav')
                shape   = x1.data.shape
                x1      = np.reshape(x1,(shape[0],1))
                x2      = np.reshape(x2,(shape[0],1))
                x3      = np.reshape(x3,(shape[0],1))
                x4      = np.reshape(x4,(shape[0],1))
                z       = np.zeros((shape[0],1),np.float32)
                zero    = np.concatenate((z,z),axis=1)
                outzero = np.concatenate((zero,zero),axis=1)
                o1      = np.concatenate((x1,x2),axis=1)
                o2      = np.concatenate((x3,x4),axis=1)
                outo    = np.concatenate((o1,o2),axis=1)
                out     = np.concatenate((outzero,outo),axis=1)
                sd.play(out,fs)
                logger.debug('play vs')
            elif msg['effect'] == 'widening':
                x1 , fs = load(songName+'_'+msg['effect']+'_1.wav')
                x2 , fs = load(songName+'_'+msg['effect']+'_2.wav')
                x3 , fs = load(songName+'_'+msg['effect']+'_3.wav')
                x4 , fs = load(songName+'_'+msg['effect']+'_4.wav')
                shape   = x1.data.shape
                x1      = np.reshape(x1,(shape[0],1))
                x2      = np.reshape(x2,(shape[0],1))
                x3      = np.reshape(x3,(shape[0],1))
                x4      = np.reshape(x4,(shape[0],1))
                z       = np.zeros((shape[0],1),np.float32)
                zero    = np.concatenate((z,z),axis=1)
                outzero = np.concatenate((zero,zero),axis=1)
                o1      = np.concatenate((x1,x2),axis=1)
                o2      = np.concatenate((x3,x4),axis=1)
                outo    = np.concatenate((o1,o2),axis=1)
                out     = np.concatenate((outzero,outo),axis=1)
                sd.play(out,fs)
                logger.debug('play widen')
            elif msg['effect'] == 'none':
                x1 , fs = load(songName+'_'+msg['effect']+'_1.wav')
                x2 , fs = load(songName+'_'+msg['effect']+'_2.wav')
                x3 , fs = load(songName+'_'+msg['effect']+'_3.wav')
                x4 , fs = load(songName+'_'+msg['effect']+'_4.wav')
                shape   = x1.data.shape
                x1      = np.reshape(x1,(shape[0],1))
                x2      = np.reshape(x2,(shape[0],1))
                x3      = np.reshape(x3,(shape[0],1))
                x4      = np.reshape(x4,(shape[0],1))
                z       = np.zeros((shape[0],1),np.float32)
                zero    = np.concatenate((z,z),axis=1)
                outzero = np.concatenate((zero,zero),axis=1)
                o1      = np.concatenate((x1,x2),axis=1)
                o2      = np.concatenate((x3,x4),axis=1)
                outo    = np.concatenate((o1,o2),axis=1)
                out     = np.concatenate((outzero,outo),axis=1)
                sd.play(out,fs)
                logger.debug('play original')
            else:
                raise ValueError('Not Supported Effect! {}'.format(msg['effect']))

        self.sendMessage(json.dumps({'code': 201}))

    def handleConnected(self):
        print(self.address, 'connected!')

    def handleClose(self):
        print(self.address, 'closed')

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: ./audio_server.py <ip_addr> <port_num>')
        exit(1)

    logging.basicConfig()
    logger.setLevel(logging.DEBUG)

    ip_addr  = sys.argv[1]
    port_num = int(sys.argv[2])

    server = SimpleWebSocketServer(ip_addr, port_num, AudioServer)
    logger.info('Audio Server Start ...')
    server.serveforever()
