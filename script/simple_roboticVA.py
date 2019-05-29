#!/usr/bin/env python3
#
##  Copyright (c) 2019 Tsung-Han Brian Lee
##  ---------------------------------------------------------------------------
##  * AUTHOR     : Tsung-Han Brian Lee
##  * DESCRIPTION:
##       1. It's a robotic voice assistant project which integrated
##          sound source localization(SSL), sound source separation
##          (SSS), cloud-based chatbot(AWS-LEX) and virtual sound
##          field control.
##       2. It's under MIT License.
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
DEBUG = True

import sys                                   # capture cli info
import json                                  # parse json data
from beamforming.uca import UCA, pixel_ring  # audio I/O & beamforming algo
import logging                               # logger to display information
import threading                             # thread
import time                                  # for sleep ...
import boto3                                 # to send AJAX request to AWS LEX
import numpy as np                           # science computation
from scipy import signal                     # resamping signal
from websocket import create_connection      # websocket communication

import rospy                                 # ROS node
from geometry_msgs.msg import Twist          # ROS built-in message type

if DEBUG:
    import sounddevice as sd                 # for debuggin sake

##
#   GLOBAL VARAIBLES TO BE SHARE FOR EACH CALLBACK
#   ----------------------------------------------------------------------
#   1. ROS Topics
#   2. ROBOT POSE
#   3. ROS PUBLISHER TO BE SHARE WITH UCA CALLBACK FCN
#   ----------------------------------------------------------------------
##
logger = logging.getLogger('ROBOTIC_VA')            # for verbosity

ROBOT_POSE_TOPIC   = '/robot_pose'
SUBGOAL_TOPIC      = '/subgoal_position'
COMMAND_TOPIC      = '/cmd_vel'

EFFECT_DICT = {
    'no effect'              : 'none',
    'cross talk cancelation' : 'xtalk',
    'virtual surround'       : 'vsurround',
    'source widening'        : 'widening'
}

##
#   A dictionary to recorde the sound source location
#   and correpsonding frequency.
##
loc_history = {}
pub         = rospy.Publisher(COMMAND_TOPIC, Twist, queue_size=10)
##
#   POINT REPRESENTATION IN THE FOLLOWING
#   ---------------------------------------------
#   1. x, y =>  cardiasion coordinate
#   2. z    =>  robot pose
#   ---------------------------------------------
##
roboticVA_vel = Twist()

##
#   Callback Handler Fcn which will be fired after each SSL job is done!
#   ----------------------------------------------------------------------
#     GOALS:
#       1. light up the led pixel ring
#       2. control the vehicle to face to the user
#     ARGUEMNTS:
#       1. firer : object which fired event, that is, a uca class instance
#       2. direction [double] : horizontal angle of the user w.r.t vehicle
#       2. polar_angle [double] : verticle angle of the user w.r.t vehicle
#   -----------------------------------------------------------------------
##
def sslHandler(firer, direction, polar_angle):
    # user interaction : LED & console
    pixel_ring.set_direction(direction)

    # transform range from 0 ~ 360 -> -180 ~ 180
    if direction > 180:
         direction -= 360

    rospy.loginfo('[UCA Callback] src @ {:.2f}, @{:.2f}'.format(direction,
                 polar_angle))

    # [TODO...] maybe down the resolution
    key = 5*(int(direction) // 5)
    loc_history[int(key)] = loc_history.get(int(key), 0) + 1


##
#   Playing audio back with specific sampling rate
#   --------------------------------------------------------------------------
#     ARGUMENTS:
#       1. in_data [numpy.ndarray] : digital signal points
#       2. fs      [doulbe]        : sampling rate, unit: Hz
#   --------------------------------------------------------------------------
##
def playAudio(in_data, fs, effect=None):
    if DEBUG:
        sd.play(in_data, fs)
    # [TODO] rendering binaural audio || publish to ROS topic


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: ./simple_roboticVA.py <audio_server_ip_addr> <audio_server_port_num>')
        exit(1)
    # setup logger level
    logging.basicConfig(level=logging.INFO)

    ##
    #   Init ROS Node : roboticVA
    #   Note:
    #       In ROS, nodes are uniquely named. If two nodes with the same
    #       node are launched, the previous one is kicked off. The
    #       anonymous=True flag means that rospy will choose a unique
    #       name for our node so that multiple listeners can run simultaneously.
    ##
    rospy.init_node('roboticVA', anonymous=False)

    ##
    #   Init Uniform Circular Array (UCA) for audio Input
    ##
    q = threading.Event()
    uca = UCA(fs=16000, nframes=2000, radius=0.032, num_mics=6, \
                quit_event=q, name='respeaker-7')

    uca.on('ssl_done', sslHandler)          # register callback handler
    enhanced = None                         # To store enhanced signal

    ##
    #   Init LEX-RUNTIME SERVICE to send AJAX request to AWS-LEX
    ##
    lex_client = boto3.client('lex-runtime', region_name="us-west-2")

    ##
    #       Robotic VA Routine
    ##
    isFailed = False
    while not (q.is_set() or rospy.is_shutdown()):
        try:
            #  enable to catch ROS topic callback fcn
            #  rospy.spin()
            #  wake up VA with a keyword
            if uca.wakeup('hello amber'):
                rospy.loginfo('Wake up')
                chunks = uca.listen()
                enhanced = uca.beamforming(chunks)

                rospy.loginfo('sending ajax request to AWS-LEX')
                # sending AJAX request to AWS-LEX
                response = lex_client.post_content(
                    botName = "RoboticVA",
                    botAlias = "roboticVA",
                    userId = "teaLab",
                    sessionAttributes = {
                    },
                    requestAttributes = {
                    },
                    contentType = "audio/lpcm; sample-rate=8000; sample-size-bits=16; channel-count=1; is-big-endian=false",
                    accept = 'audio/pcm',
                    inputStream = (signal.decimate(enhanced/2**15, 2)* 2**15).astype('<i2').tostring()
                )

                if response["dialogState"] == 'ReadyForFulfillment': break
                if response["dialogState"] == 'Fulfilled': break
                elif response["dialogState"] == "Failed": isFailed = True

                if DEBUG:
                    rospy.loginfo('  * Playing back response ...')
                content = np.fromstring(response["audioStream"].read(), dtype="<i2")

                # Play enhanced speech back
                if DEBUG:
                    rospy.loginfo('Playing enhanced speech ...')
                    # playAudio(enhanced / 2**14, 16000)
                    # time.sleep(3.0)

                ##
                #   Playing response back to user
                ##
                playAudio(content / np.max(content), 16000)
                rospy.loginfo('\n-------------------')
                rospy.loginfo(' [RESPONSE]: ' + response["message"])

                if isFailed: break
        except KeyboardInterrupt:
            rospy.loginfo('Quit')
            q.set()
            break
        # not handling other exception
        except Exception as e:
            rospy.logwarn(e)
    uca.close()

    if not isFailed:
        content = np.fromstring(response["audioStream"].read(), dtype="<i2")
        playAudio(content / np.max(content), 16000)
        rospy.loginfo('\n-------------------')
        rospy.loginfo(response["message"])

        print("\n///// Request Information ///// ")
        for keys in response["slots"].keys():
            print("  * " + keys + ": " + response["slots"][keys])
        print("\n\n////////// Conversation END! //////////")

    ##
    #   Rotate the Robotic VA
    #   -------------------------------
    #   1. determine the angle
    #   2. publish command
    #   3. sleep awhile
    #   2. publish command
    ##
    rospy.loginfo('[ROBOTIC VA] loc_history len: {}'.format(len(loc_history)))
    rospy.loginfo('[ROBOTIC VA]\n loc_history: {}'.format(loc_history))

    goal = max(loc_history, key=lambda k: loc_history[k])
    rospy.loginfo('[ROBOTIC VA] goal: {}'.format(goal))

    rotation_time      = 3

    vel_msg = Twist()
    vel_msg.linear.x  = 0.0
    vel_msg.angular.z = float(goal) * 0.01745329251 / rotation_time

    if vel_msg.angular.z < 0.2:
        vel_msg.angular.z = 0.2
        rotation_time -= 1

    rospy.loginfo('[ROBOTIC VA] Publishing command with linear: {}, angular {}'.format(vel_msg.linear.x, vel_msg.angular.z))

    pub.publish(vel_msg)

    time.sleep(rotation_time + 2)

    vel_msg.linear.x  = 0.0
    vel_msg.angular.z = 0.0
    pub.publish(vel_msg)

    # empty the history
    loc_history = {}

    # sending audio request to audio server
    address  = 'ws://192.168.1.115:8888'
    effect   = EFFECT_DICT[response['slots']['effect']]
    songName = response['slots']['songName']

    ws      = create_connection(address)
    toSend  = json.dumps({
        'effect'   : effect,
        'songName' : songName
    })

    logger.info('Sending : ' + toSend)
    ws.send(toSend)
    logger.info('Receiving ...')
    result = ws.recv()
    logger.info('Received : ' + result)

    ws.close()


    # hang the process
    rospy.spin()
