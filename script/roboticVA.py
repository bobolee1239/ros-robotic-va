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

from beamforming.uca import UCA, pixel_ring  # audio I/O & beamforming algo
import logging                               # logger to display information
import threading                             # thread
import time                                  # for sleep ...
import boto3                                 # to send AJAX request to AWS LEX
import numpy as np                           # science computation
from scipy import signal                     # resamping signal

import rospy                                 # ROS node
from geometry_msgs.msg import Point          # ROS built-in message type
from nav_msgs.msg import Odometry            # ROS built-in message type

if DEBUG:
    import sounddevice as sd                 # for debuggin sake

class PlanarInfo(object):
    def __init__(self, x = 0.0, y=0.0, th=0.0):
        self.x  = x
        self.y  = y
        self.th = th
    def move2(x, y, th=None):
        self.x = x
        self.y = y
        if th:
            self.th = th
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

pub = rospy.Publisher(SUBGOAL_TOPIC, Point, queue_size=10)
##
#   POINT REPRESENTATION IN THE FOLLOWING
#   ---------------------------------------------
#   1. x, y =>  cardiasion coordinate
#   2. z    =>  robot pose
#   ---------------------------------------------
##
roboticVA_position = Point(0.0, 0.0, 0.0)
subGoal            = Point(0.0, 0.0, 0.0)

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
    rospy.loginfo('In callback: src @ {:.2f}, @{:.2f}'.format(direction,
                 polar_angle))

    command = Point(roboticVA_position.x,
                    roboticVA_position.y,
                    roboticVA_position.z + direction)

    # transform range from 0 ~ 360 -> -180 ~ 180
    if command.z > 180:
         command.z -= 360
    
    rospy.logdebug('command.z : {}'.format(command.z))

    # transform from degree to rad
    command.z *= (0.01745329252)

    # Publish command to ROS topic
    pub.publish(command)


##
#   Callback Handler Fcn which will be fired when receive a ROS message
#   ----------------------------------------------------------------------
#     GOALS:
#       1. Update Robot Pose
#     ARGUEMNTS:
#       1. recvMsg [nav_msgs::Odometry]
#   -----------------------------------------------------------------------
##
def odometryHandler(loc):
    # update robotic VA position
    roboticVA_position = Point(loc.pose.pose.position.x,
                               loc.pose.pose.position.y,
                               loc.pose.pose.orientation.z)

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
    # setup logger level
    logging.basicConfig(level=logging.INFO)

    ##
    #   Init ROS Node : roboticVA
    ##
    rospy.init_node('roboticVA', anonymous=True)
    rospy.Subscriber(ROBOT_POSE_TOPIC, Odometry, odometryHandler)

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
    while not q.is_set():
        try:
            #  enable to catch ROS topic callback fcn
            #  rospy.spin()
            #  wake up VA with a keyword
            if uca.wakeup('hello amber'):
                rospy.loginfo('Wake up')
                chunks = uca.listen()
                enhanced = uca.beamforming(chunks)

                pub.publish(Point(roboticVA_position.x,
                                  roboticVA_position.y,
                                  roboticVA_position.z))

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
        logger.info('\n-------------------')
        logger.info(response["message"])

        print("\n///// Request Information ///// ")
        for keys in response["slots"].keys():
            print("  * " + keys + ": " + response["slots"][keys])
        print("\n\n////////// Conversation END! //////////")
    rospy.spin()
