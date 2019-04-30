// Copyright 2019 Tsung-Han Lee
/**************************************
 ** FILE    : hall_sensor_decode.h
 ** AUTHOR  : Tsung-Han Lee
 **************************************/
#ifndef HALL_SENSOR_DECODE_H_
#define HALL_SENSOR_DECODE_H_

#include <wiringPi.h>
//  #include <iostream>  //  DEBUG

#define LEFT_HALL_A     0           //  BCM 17 : WiringPi  0
#define LEFT_HALL_B     2           //  BCM 27 : WiringPi  2
#define RIGHT_HALL_A    21          //  BCM  5 : WiringPi 21
#define RIGHT_HALL_B    30          //  BCM  0 : WiringPi 30

/********************************************************
 *  WheelState should be defined as volatile since it
 *  will be modified in ISR
 ********************************************************/
typedef volatile struct WheelState {
    volatile int numStateChange;
    volatile bool direction;
    volatile unsigned short state;
    volatile unsigned short prestate;
    volatile int hallA;
    volatile int hallB;

    /* constructor for structure */
    WheelState() {
        numStateChange  = 0;
        direction       = false;
        state           = 0;
        prestate        = 0;
        hallA           = 0;
        hallB           = 0;
    }
} WheelState_t;

WheelState_t* const leftWheel = new WheelState_t();
WheelState_t* const rightWheel = new WheelState_t();
//void hallSensor_ISR(void);
//void closeHallSensor();

void hallSensor_ISR(void) {
    /* Dealing with left motor */
    /* read state of GPIO pin */
    leftWheel->hallA = digitalRead(LEFT_HALL_A);
    leftWheel->hallB = digitalRead(LEFT_HALL_B);

    /***********************************************************
     **   I. hallA hallB should be 1, 0 => bitwise xor is fine
     **  II. update state and check if:
     **        1. state increase (direction = forward)
     **        2. state decrease (direction = backward)
     **        3. same state     (nothing happened)
     **
     **  [HALL SENSOR TABLE] :
     **         * FORWARD   : from up to down
     **         * BACKWARD  : from down to up
     **   ---------------------------
     **   | STATE | HALL A | HALL B |
     **   |   1   |   0    |   0    |
     **   |   2   |   0    |   1    |
     **   |   3   |   1    |   1    |
     **   |   4   |   1    |   0    |
     **   |   1   |   0    |   0    |
     **   ---------------------------
     ***********************************************************/
    // update state in [1, 2, 3, 4]
    leftWheel->state = (leftWheel->hallA << 1)
                        + (leftWheel->hallA ^ leftWheel->hallB)
                        + 1;

    if (leftWheel->state == 1) {
        if (leftWheel->prestate == 4) {
            leftWheel->direction = true;
        } else if (leftWheel->prestate != 1) {
            leftWheel->direction = false;
        }
    } else if (leftWheel->state == 4) {
        if (leftWheel->prestate == 1) {
            leftWheel->direction = false;
        } else if (leftWheel->prestate != 4) {
            leftWheel->direction = true;
        }
    } else {
        leftWheel->direction = (leftWheel->state > leftWheel->prestate);
    }

    /* do nothing if state ain't change */
    if (leftWheel->prestate != leftWheel->state) {
        if (leftWheel->direction) {
            ++(leftWheel->numStateChange);
        } else {
            --(leftWheel->numStateChange);
        }
        //  DEBUG
        //  std::cout << leftWheel->prestate << " -> " << leftWheel->state << std::endl;
        // std::cout << leftWheel->numStateChange << std::endl;
    }

    /* update previous state */
    leftWheel->prestate = leftWheel->state;

    /* Dealing with right motor */
    /* read state of GPIO pin */
    rightWheel->hallA = digitalRead(RIGHT_HALL_A);
    rightWheel->hallB = digitalRead(RIGHT_HALL_B);

    /* hallA hallB should be 1, 0 */
    rightWheel->state = (rightWheel->hallA << 2)
                        + (rightWheel->hallA ^ rightWheel->hallB)
                        + 1;

    if (rightWheel->state == 1) {
        if (rightWheel->prestate == 4) {
            rightWheel->direction = true;
        } else if (rightWheel->prestate != 1) {
            rightWheel->direction = false;
        }
    } else if (rightWheel->state == 4) {
        if (rightWheel->prestate == 1) {
            rightWheel->direction = false;
        } else if (rightWheel->prestate != 4) {
            rightWheel->direction = true;
        }
    } else {
        rightWheel->direction = (rightWheel->state > rightWheel->prestate);
    }

    /* do nothing if state ain't change */
    if (rightWheel->prestate != rightWheel->state) {
        if (rightWheel->direction) {
            ++rightWheel->numStateChange;
        } else {
            --rightWheel->numStateChange;
        }
    }

    /* update previous state */
    rightWheel->prestate = rightWheel->state;
}

void closeHallSensor() {
    delete rightWheel;
    delete leftWheel;
}

#endif  // HALL_SENSOR_DECODE_H_
