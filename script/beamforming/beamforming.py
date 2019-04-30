#!/usr/bin/env python3
## FILE: mic_array.py
## 

import threading 
import numpy as np
from pixel_ring import pixel_ring
from mic_array import MicArray
import signal

try:
    # python2
    import Queue
except:
    # python3
    import queue as Queue

class SSS_Thread(threading.Thread):
    """
    """
    def __init__(self, in_queue, out_queue, quit_event):
        threading.Thread.__init__(self)
        self.in_queue = in_queue
        self.out_queue = out_queue
        self.quit_event = quit_event

    def run(self): 
        from beamformer import DAS
        das = DAS(0.01)
        while not self.quit_event.is_set():
            if self.in_queue.empty():  
                continue

            pos, raw = self.in_queue.get()
            raw.shape = -1, 8
            self.out_queue.put(das.sound_separation(raw[:, 1:7], [pos]))
            
class SSL_Thread(threading.Thread):
    """
    """
    def __init__(self, out_queue, quit_event):
        threading.Thread.__init__(self)
        self.out_queue = out_queue
        self.quit_event = quit_event



if __name__ == '__main__':
    raw_sig_queue   = Queue.Queue()
    en_sig_queue    = Queue.Queue()
    quit_event      = threading.Event()


    def sig_handler():
        print('QUIT')
        quit_event.set()
    signal.signal(signal.SIGINT, sig_handler)

    sss = SSS_Thread(raw_sig_queue, en_sig_queue, quit_event)

    # excuting thread
    sss.start()

    with MicArray(16000, 8, 16000/8) as mic:
        for frames in mic.read_chunks():
            chunk = np.fromstring(frames, dtype='int16')
            direction = mic.get_direction(chunk)
            raw_sig_queue.put([direction, (chunk >> 15)])
            pixel_ring.set_direction(direction)
            print(int(direction))
            
            if quit_event.is_set():
                break
        print('breaking')
        pixel_ring.off()
    print('wait for joining')
    sss.join()
    print('joing done')



