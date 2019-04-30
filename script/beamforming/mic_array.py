#!/usr/bin/env python3

## FILE: mic_array.py
## 

import pyaudio
import threading
import numpy as np
from gcc_phat import gcc_phat
import math
from beamformer import DAS

try:
    # python2 
    import Queue
except:
    # python3
    import queue as Queue

SOUND_SPEED = 343.2

MIC_DISTANCE_6P1 = 0.064
MAX_TDOA_6P1 = MIC_DISTANCE_6P1 / float(SOUND_SPEED)


class MicArray(object):

    def __init__(self, rate=16000, channels=8, chunk_size=None):
        self.pyaudio_instance = pyaudio.PyAudio()
        self.queue = Queue.Queue()
        self.sep_queue = Queue.Queue()
        self.quit_event = threading.Event()
        self.channels = channels
        self.sample_rate = rate
        self.chunk_size = chunk_size if chunk_size else rate / 100
        
        self.beamformer = DAS(eps=0.01, nfft=1024)

        device_index = None
        for i in range(self.pyaudio_instance.get_device_count()):
            dev = self.pyaudio_instance.get_device_info_by_index(i)
            name = dev['name'].encode('utf-8')
            print(i, name, dev['maxInputChannels'], dev['maxOutputChannels'])
            if dev['maxInputChannels'] == self.channels:
                print('Use {}'.format(name))
                device_index = i
                break

        if device_index is None:
            raise Exception('can not find input device with {} channel(s)'.format(self.channels))

        self.stream = self.pyaudio_instance.open(
            input=True,
            start=False,
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=int(self.sample_rate),
            frames_per_buffer=int(self.chunk_size),
            stream_callback=self._callback,
            input_device_index=device_index,
        )

    def _callback(self, in_data, frame_count, time_info, status):
        self.queue.put(in_data)
        return None, pyaudio.paContinue

    def start(self):
        self.queue.queue.clear()
        self.stream.start_stream()


    def read_chunks(self):
        self.quit_event.clear()
        while not self.quit_event.is_set():
            frames = self.queue.get()
            if not frames:
                break

            yield frames

    def stop(self):
        self.quit_event.set()
        self.stream.stop_stream()
        self.queue.put('')

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, type, value, traceback):
        if value:
            return False
        self.stop()

    def get_direction(self, buf):
        best_guess = None
        if self.channels == 8:
            MIC_GROUP_N = 3
            MIC_GROUP = [[1, 4], [2, 5], [3, 6]]

            tau = [0] * MIC_GROUP_N
            theta = [0] * MIC_GROUP_N

            # buf = np.fromstring(buf, dtype='int16')
            for i, v in enumerate(MIC_GROUP):
                tau[i] = gcc_phat(buf[v[0]::8], buf[v[1]::8], fs=self.sample_rate, max_tau=MAX_TDOA_6P1, interp=1)
                theta[i] = math.asin(tau[i] / MAX_TDOA_6P1) * 180 / math.pi

            min_index = np.argmin(np.abs(tau))
            if (min_index != 0 and theta[min_index - 1] >= 0) or (min_index == 0 and theta[MIC_GROUP_N - 1] < 0):
                best_guess = (theta[min_index] + 360) % 360
            else:
                best_guess = (180 - theta[min_index])

            best_guess = (best_guess + 120 + min_index * 60) % 360

        return best_guess

    def separationAt(self, buff, directions):
        """
        Arg:
        --------------------------------------------------------
        buff      [numpy.ndarray]   : shape(#frames, #channels)
        direction [array of floats] : stores separatino directions

        Return:
        --------------------------------------------------------
        en_speech [numpy.ndarray]   : shape(#frames, len(directions))
        """
        # buff.shape = -1, 8

        self.sep_queue.put(self.beamformer.sound_separation(buff[:, 1:7], directions))



def test_8mic():
    import signal
    import time
    from pixel_ring import pixel_ring

    is_quit = threading.Event()
    

    def signal_handler(sig, num):
        is_quit.set()
        print('Quit')

    signal.signal(signal.SIGINT, signal_handler)
 
    en_speech = np.zeros((1,))
    raw = np.zeros((1, 8))
    with MicArray(16000, 8, 16000 / 8)  as mic:
        for frames in mic.read_chunks():
            chunk = np.fromstring(frames, dtype='int16')
            direction = mic.get_direction(chunk)
            pixel_ring.set_direction(direction)
            print(int(direction))
           #  chunk = chunk / (2**15)
           #  chunk.shape = -1, 8

           #  raw = np.concatenate((raw, chunk), axis=0)
           #  
           #  mic.separationAt(chunk, [direction])
           #  en_speech = np.concatenate((en_speech, mic.sep_queue.get()),axis=0)
            if is_quit.is_set():
                break

    pixel_ring.off()
    return raw, en_speech


if __name__ == '__main__':
    # test_4mic()
    raw, en_speech = test_8mic()
