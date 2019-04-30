#!/usr/bin/env python3
#
##  Copyright (c) 2019 Tsung-Han Brian Lee, Shincheng Huang
##  ---------------------------------------------------------------------------
##  * AUTHOR: Tsung-Han Brian Lee,
##            Shincheng Huang
##  * FILE        : uca.py
##  * DESCRIPTION : component based design UCA
##  * REFERENCE  :
##       1. https://github.com/respeaker
##  ---------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a
##  copy of this software and associated documentation files (the "Software"),
##  to deal in the Software without restriction, including without limitation
##  the rights to use, copy, modify, merge, publish, distribute, sublicense,
##  and/or sell copies of the Software, and to permit persons to whom the
##  Software is furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in
##  all copies or substantial portions of the Software.

##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
##  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
##  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
##  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
##  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
##  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
##  IN THE SOFTWARE.
##  ---------------------------------------------------------------------------
#

import collections          #  deque
import threading            #  multithreading supporting
import numpy as np          #  science computing
import pyaudio              #  audio I/O
import os                   #  system path utility
import logging              #  diplay information in various level
import math                 #  triangular math

#  import gccphat utility
try:
    from .gcc_phat import gcc_phat
except:
    from gcc_phat import gcc_phat

#  import led contrl unit on respeaker
try:
    from .pixel_ring import pixel_ring
except:
    from pixel_ring import pixel_ring

#  import queue to be the bridge between threads
try:
    # python2 supporting
    import Queue
except:
    # python3
    import queue as Queue

#  voice active detection utility
try:
    from .respeaker.vad import vad
except:
    from respeaker.vad import vad

logger              = logging.getLogger('uca')
collecting_audio    = os.getenv('COLLECTING_AUDIO', 'no')


class UCA(object):
    """
    UCA (Uniform Circular Array)

    Design Based on Respeaker 7 mics array architecture

    It'll fire following events:
        1. ssl_done
        2. horizontal_avg
    """
    SOUND_SPEED = 343.2
    listening_mask   = (1<<0)
    detecting_mask   = (1<<1)
    validation = []

    def __init__(self, fs=16000, nframes=2000, radius=0.032, num_mics=6, quit_event=None, name='respeaker-7'):
        self.radius     = radius
        self.fs         = fs
        self.nframes    = nframes
        self.nchannels  = (num_mics + 2 if name == 'respeaker-7' else num_mic)
        self.num_mics   = num_mics
        self.max_delay  = radius * 2 / UCA.SOUND_SPEED
        self.delays     = None

        self.pyaudio_instance = pyaudio.PyAudio()

        #  tryna get device which meet the user's need
        self.device_idx = None
        for i in range(self.pyaudio_instance.get_device_count()):
            dev  = self.pyaudio_instance.get_device_info_by_index(i)
            name = dev['name'].encode('utf-8')
            print(i, name, dev['maxInputChannels'], dev['maxOutputChannels'])

            #  If found the device
            if name.lower().find(b'respeaker') >= 0 and dev['maxInputChannels'] >= num_mics:
                print('Use {}'.format(name))
                self.device_idx = i
                break

        #  if no device is found
        if not self.device_idx:
            raise ValueError('Wrong #channels of mic array!')

        #  input stream
        self.stream = self.pyaudio_instance.open(
            input       = True,
            start       = False,
            format      = pyaudio.paInt16,
            channels    = self.nchannels,
            rate        = self.fs,
            frames_per_buffer   = int(self.nframes),
            stream_callback     = self._callback,
            input_device_index  = self.device_idx
        )

        self.quit_event = quit_event if quit_event else threading.Event()

        # multi-channels input
        self.listen_queue = Queue.Queue()
        # mono-channel input
        self.detect_queue = Queue.Queue()

        self.active = False
        self.status = 0

        self.listen_history = collections.deque(maxlen=16)
        self.detect_history = collections.deque(maxlen=64)

        # index 0 for listening duration count,
        # index 1 for timeout duration count.
        self.listen_countdown = [0, 0]

        self.decoder = UCA.create_decoder()
        self.decoder.start_utt()

        # build tdoa matrix
        mic_theta           = np.arange(0, 2*np.pi, 2*np.pi/6)
        self.tdoa_matrix    = np.array([np.cos(mic_theta[1:]), np.sin(mic_theta[1:])]).T

        self.tdoa_matrix   -= np.ones((len(mic_theta)-1, 2)) \
                              * np.array([np.cos(mic_theta[0]), np.sin(mic_theta[0])])

        self.tdoa_measures  = np.ones(((len(mic_theta)-1, ))) \
                                * UCA.SOUND_SPEED / self.radius

        # store callback fcn
        self.handlers = dict();

    def on(self, event, handler):
        self.handlers[event] = handler

    def fire(self, event, *argv):
        """The first argument of the handler must be instance itself"""
        if event in self.handlers:
            self.handlers[event](self, *argv)

    def wakeup(self, keyword=None):
        self.decoder.end_utt()
        self.decoder.start_utt()

        # clear detecting queue
        self.detect_history.clear()
        self.detect_queue.queue.clear()

        # flag up detecting
        self.status |= UCA.detecting_mask

        self.stream.start_stream()
        result = None
        logger.info('Start detecting ...')

        while not self.quit_event.is_set():
            if self.detect_queue.qsize() > 4:
                logger.info('Too many delays, {0} in queue'.format(self.detect_queue.qsize()))
            #  ensure something in the queue
            elif self.detect_queue.empty():
                continue

            #  pop data from the queue
            data = self.detect_queue.get()

            self.detect_history.append(data)

            self.decoder.process_raw(data, False, False)

            hypothesis = self.decoder.hyp()
            if hypothesis:
                if collecting_audio != 'no':
                    logger.debug(collecting_audio)
                    # save detect_history as wave ?
                # clear history
                self.detect_history.clear()
                if keyword:
                    if hypothesis.hypstr.find(keyword) >= 0:
                        result = hypothesis.hypstr
                        break
                    else:
                        self.decoder.end_utt()
                        self.decoder.start_utt()
                        self.detect_history.clear()
                else:
                    result = hypothesis.hypstr
                    break

        # flag down detecting
        self.status &= ~UCA.detecting_mask
        self.stop()
        return result

    def DOA(self, buf):
        best_guess = None
        MIC_GROUP_N = self.num_mics - 1
        MIC_GROUP = [[1+i, 1] for i in range(1, MIC_GROUP_N+1)]

        tau = [0] * MIC_GROUP_N

        # estimate each group of delay
        for i, v in enumerate(MIC_GROUP):
            tau[i] = gcc_phat(buf[v[0]::8], buf[v[1]::8], fs=self.fs, max_tau=self.max_delay, interp=10)

        # save delays for separation
        # self.delays = [0] + tau

        # least square solution of (cos, sin)
        sol = np.linalg.pinv(self.tdoa_matrix).dot( \
              (self.tdoa_measures * np.array(tau)).reshape(MIC_GROUP_N, 1))

        # found out theta
        # another 180.0 for positive value, 30.0 for respeaker architecture
        tmp = 1 if (np.sqrt(sol.T.dot(sol)) > 1) else np.sqrt(sol.T.dot(sol))
        return ((math.atan2(sol[1], sol[0])/np.pi*180.0 + 210.0) % 360
                ,math.acos(tmp)/np.pi*180.0, [0] + tau)

    def listen(self, duration=9, timeout=3):
        vad.reset()

        # setting countdown value
        self.listen_countdown[0] = (duration*self.fs + self.nframes -1) / self.nframes
        self.listen_countdown[1] = (timeout*self.fs + self.nframes - 1) / self.nframes

        self.listen_queue.queue.clear()
        self.status |= UCA.listening_mask
        self.start()

        logger.info('Start Listening')

        def _listen():
            """
            Generator for input signals
            """
            try:
                data = self.listen_queue.get(timeout=timeout)
                while data and not self.quit_event.is_set():
                    yield data
                    data = self.listen_queue.get(timeout=timeout)
            except Queue.Empty:
                pass
            self.stop()

        return _listen()

    def start(self):
        if self.stream.is_stopped():
            self.stream.start_stream()

    def stop(self):
        if not self.status and self.stream.is_active():
            self.stream.stop_stream()

    def close(self):
        self.quit()
        self.stream.close()
        pixel_ring.off()

    def quit(self):
        self.status = 0     # flag down everything
        self.quit_event.set()
        self.listen_queue.put('') # put logitical false into queue

    def beamforming(self, chunks):
        delays = [0.0] * (self.num_mics)

        enhanced_speech = []
        avgFrames       = None
        count           = 0
        for chunk in chunks:
            #  decode from binary stream
            raw_sigs = np.fromstring(chunk, dtype='int16')

            count += 1
            count %= 3

            if count == 1:
                avgFrames = raw_sigs
            else:
                avgFrames += raw_sigs

            if count == 0:
                #  tdoa & doa estimation based on planar wavefront
                direction, polar_angle, delays = self.DOA(raw_sigs)
                avgFrames = None
                #  fire event callback function
                self.fire('ssl_done', direction, polar_angle)
                # setting led && logger info
                ## Moving following code to event handler
                ## pixel_ring.set_direction(direction)
                ## logger.debug('@ {:.2f}, @{:.2f}, delays = {}'.format(direction, polar_angle, np.array(delays)*self.fs))


            # *************  apply DAS beamformer  ****************
            int_delays = (np.array(delays)*self.fs).astype('int16')
            int_delays -= int(np.min(int_delays))
            max_delays = np.max(int_delays);

            toAdd = np.zeros((raw_sigs.size//8 + max_delays,
                              self.num_mics), dtype='int16')
            # manupilate integer delays
            for i in range(self.num_mics):
                # 1. Padding zero in the front and back
                # 2. shift 2 bits (devide by 4)
                toAdd[:, i] = np.concatenate((np.zeros(int_delays[i], dtype='int16'),
                                               raw_sigs[i+1::8] >> 2,
                                               np.zeros(max_delays-int_delays[i], dtype='int16')),
                                              axis=0)
            # add them together
            enhanced_speech.append(np.sum(toAdd, axis=1, dtype='int16'))
            # *************************************************

        return np.concatenate(enhanced_speech, axis=0)

    def _callback(self, in_data, frame_count, time_info, status):
        """
        Pyaudio callback function
        """
        # decode bytes stream
        mulChans = np.fromstring(in_data, dtype='int16')
        mono     = mulChans[7::8].tostring()

        if self.status & UCA.detecting_mask:
            # signed 16 bits little endian
            self.detect_queue.put(mono)

        if self.status & UCA.listening_mask:

            active = vad.is_speech(mono)

            if active:
                if not self.active:
                    for d in self.listen_history:
                        self.listen_queue.put(d)
                        self.listen_countdown[0] -= 1  # count down timeout
                    self.listen_history.clear()

                self.listen_queue.put(in_data)
                self.listen_countdown[0] -= 1          # count down listening time
            else:
                if self.active:
                    self.listen_queue.put(in_data)
                else:
                    self.listen_history.append(in_data)
                self.listen_countdown[1] -= 1         # coutn down listening time

            if self.listen_countdown[0] <= 0 or self.listen_countdown[1] <= 0:
                self.listen_queue.put('')
                self.status &= ~self.listening_mask
                logger.info('Stop listening')

            self.active = active

        return None, pyaudio.paContinue




    @staticmethod
    def create_decoder():
        from pocketsphinx.pocketsphinx import Decoder

        path              = os.path.dirname(os.path.realpath(__file__))
        pocketsphinx_dir = os.getenv('POCKETSPHINX_DATA',
                                os.path.join(path, 'assets', 'pocketsphinx-data'))
        hmm = os.getenv('POCKETSPHINX_HMM', os.path.join(pocketsphinx_dir, 'hmm'))
        dic = os.getenv('POCKETSPHINX_DIC', os.path.join(pocketsphinx_dir, 'dictionary.txt'))
        kws = os.getenv('POCKETSPHINX_KWS', os.path.join(pocketsphinx_dir, 'keywords.txt'))

        config = Decoder.default_config()
        config.set_string('-hmm', hmm)
        config.set_string('-dict', dic)
        config.set_string('-kws', kws)
        # config.set_int('-samprate', fs) # uncomment for fs != 16k. use config.set_float() on ubuntu
        config.set_int('-nfft', 512)
        config.set_float('-vad_threshold', 2.7)
        config.set_string('-logfn', os.devnull)

        return Decoder(config)



def sslHandler(firer, direction, polar_angle):
    pixel_ring.set_direction(direction)
    print('In callback: src @ {:.2f}, @{:.2f}'.format(direction,
            polar_angle))


def task(quit_event):
    import time

    uca = UCA(fs=16000, nframes=2000, radius=0.032, num_mics=6, \
                quit_event=quit_event, name='respeaker-7')
    uca.on('ssl_done', sslHandler);

    while not quit_event.is_set():
        if uca.wakeup('hello amber'):
            print('Wake up')
            time.sleep(1.0)
            chunks = uca.listen()
            uca.beamforming(chunks)

    uca.close()

def main():
    import time

    logging.basicConfig(level=logging.DEBUG)

    q = threading.Event()
    t = threading.Thread(target=task, args=(q, ))
    t.start()
    while True:
        try:
            time.sleep(1.0)
        except KeyboardInterrupt:
            print('Quit')
            q.set()
            break
    # wait for the thread
    t.join()

if __name__ == '__main__':
    main()
