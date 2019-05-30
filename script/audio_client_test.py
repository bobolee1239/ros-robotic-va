#!/usr/bin/env python3

##
#   FILE: client.py
##

import sys
import logging
import json
import time
import numpy as np

try:
    import thread
except:
    import _thread as thread

from websocket import create_connection

logger = logging.getLogger('AudioClient-Test')

EFFECT = {
    0 : 'none',
    1 : 'xtalk',
    2 : 'vsurround',
    3 : 'widening'
}

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: ./audio_client_test.py <ip_addr> <port_num>')
        exit(1)

    logging.basicConfig()
    logger.setLevel(logging.DEBUG)

    address = 'ws://' + sys.argv[1] + ':' + sys.argv[2]

    effect = EFFECT[int(input('What kinda effect? (0 : none, 1 : xtalk, 2 : vsurround, 3 : widening)  '))]

    songName = input('which song?  ')

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
