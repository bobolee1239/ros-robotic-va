//  Copyright (c) 2019 Tsung-Han Lee
/******************************************************************************
 ** FILE       : mobileBot.cpp
 ** AUTHOR     : Tsung-Han Brian Lee
 ** DESCRIPTION:
 ** LICENSE    : MIT
 ** ---------------------------------------------------------------------------
 ** Permission is hereby granted, free of charge, to any person obtaining a
 ** copy of this software and associated documentation files (the "Software"),
 ** to deal in the Software without restriction, including without limitation
 ** the rights to use, copy, modify, merge, publish, distribute, sublicense,
 ** and/or sell copies of the Software, and to permit persons to whom the
 ** Software is furnished to do so, subject to the following conditions:
 **
 ** The above copyright notice and this permission notice shall be included
 ** in all copies or substantial portions of the Software.
 **
 ** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 ** OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 ** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 ** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 ** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 ** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 ** THE SOFTWARE.
 ** ---------------------------------------------------------------------------
 ******************************************************************************/
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>
#include <iostream>
#include <iomanip>
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "ros/console.h"

#include "hall_sensor_decode.h"

/********************* DEFINITION ***************************************/
int initHallSensors();
int initPWM();
int initTimerISR();
void timerISR(int signum);

typedef volatile struct PIController {
    volatile double rpm;
    volatile double piOut;      //  output of PI controller
    volatile double err;        //  error
    volatile double ierr;       //  integration of error
    double kp;
    double ki;
} PIController_t;

typedef volatile struct Vehicle {
    const double r;             //  radius
    const double L;             //  distance between two wheels
    volatile double V;          //  forward speed
    volatile double W;          //  rotation speed
    volatile double refV;       //  reference forward speed
    volatile double refW;       //  reference rotation speed
} Vehicle_t;
/***********************************************************************/

/********************* PARAMETERS ***************************************/
const int leftPWM   =  1;     //  WiringPi  1 : BCM 18
const int rightPWM  = 23;     //  WiringPi 23 : BCM 13

PIController_t leftController  = {0.0, 0.0, 0.0, 0.0, 0.008, 0.02};
PIController_t rightController = {0.0, 0.0, 0.0, 0.0, 0.008, 0.02};

Vehicle_t car = {0.0989, 0.2748, 0.0, 0.0, 0.0, 0.0};

double leftRef  = -15.0;
double rightRef = -30.0;
double Ts       = 0.01;       // sampling interval

/***********************************************************************/

int main(int argc, char* argv[]) {
    /* Setup wiringPi */
    if (wiringPiSetup() < 0) {
        std::cerr << "Unable to setup wiringPi: "
                  << strerror(errno) << std::endl;
        return -1;
    }

    if (initHallSensors() < 0) {
        std::cerr << "Init hall sensors failed!" << std::endl;
    }

    initPWM();
    initTimerISR();

    while (1) {
        /*************************
         **  1. 2pi/60      = 0.1047197551
         **  2. 2pi/60/2    = 0.05235987756
         **************************/
        car.V = (leftController.rpm + rightController.rpm)
                  * car.r * 0.05235987756;
        car.W = -car.r * (rightController.rpm - leftController.rpm)
                / car.L * 0.1047197551;
        //  [TODO] publish to ros topic

        /* output sensor */
        std::cout << "l:" << std::fixed << std::setprecision(2)
                  << leftController.rpm << " , ";
        std::cout << "r:" << std::fixed << std::setprecision(2)
                  << rightController.rpm << std::endl;
        //  [TODO] looping in fixed rate
        sleep(1);
    }

    /* release memory */
    closeHallSensor();
    return 0;
}

void timerISR(int signum) {
    /*********** MOTOR1 **********/
    // measure TODO... translate rpm correctly : gear ratio
    leftController.rpm = leftWheel->numStateChange * 1.04166;
    leftWheel->numStateChange = 0;

    /**************** PI Controller *******************/
    leftController.err = leftRef - leftController.rpm;
    leftController.ierr += Ts*leftController.err;

    std::cout << "err: " << leftController.err << std::endl;
    std::cout << "kp: " << leftController.kp
              << ", ki: " << leftController.ki << std::endl;

    // limit integration output
    if (leftController.ierr > 50.0) {
        leftController.ierr = 50.0;
    } else if (leftController.ierr < -50.0) {
        leftController.ierr = -50.0;
    }
    leftController.piOut = leftController.kp * leftController.err
                           + leftController.ki * leftController.ierr;
    leftController.piOut *= -1;
    // saturation
    if (leftController.piOut > 0.5) {
        leftController.piOut = 0.5;
    } else if (leftController.piOut < -0.5) {
        leftController.piOut = -0.5;
    }
    // output to acuator : complement PWM
    pwmWrite(leftPWM, static_cast<int>(1024*(0.5 + leftController.piOut)));
    /**************************************************/
    /*********** MOTOR2 **********/
    // measure TODO... translate rpm correctly : gear ratio
    rightController.rpm = rightWheel->numStateChange * 1.04166 * 2;
    rightWheel->numStateChange = 0;

    /**************** PI Controller *******************/
    rightController.err = rightRef - rightController.rpm;
    rightController.ierr += Ts*rightController.err;

    // limit integration output
    if (rightController.ierr > 50.0) {
        rightController.ierr = 50.0;
    } else if (rightController.ierr < -50.0) {
        rightController.ierr = -50.0;
    }
    rightController.piOut = rightController.kp * rightController.err
                           + rightController.ki * rightController.ierr;
    // saturation
    if (rightController.piOut > 0.5) {
        rightController.piOut = 0.5;
    } else if (rightController.piOut < -0.5) {
        rightController.piOut = -0.5;
    }
    // output to acuator : complement PWM
    pwmWrite(rightPWM, static_cast<int>(1024*(0.5 + rightController.piOut)));
    /**************************************************/
}

int initTimerISR() {
    struct sigaction sa;
    struct itimerval timer;

    /* Install timer_handler as the signal handler for SIGALRM (Real timer) */
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = &timerISR;
    sigaction(SIGALRM, &sa, NULL);

    /* Configure the timer to be expired after 10 ms */
    timer.it_value.tv_sec  = 0;
    timer.it_value.tv_usec = 10000;
    /* Configure the timer back to 10 ms after expired */
    timer.it_interval.tv_sec  = 0;
    timer.it_interval.tv_usec = 10000;

    /* Start a real timer */
    setitimer(ITIMER_REAL, &timer, NULL);

    return 0;
}

int initHallSensors() {
    /* Register ISR while hall sensor rising and falling */
    if (wiringPiISR(LEFT_HALL_A, INT_EDGE_BOTH, &hallSensor_ISR) < 0) {
        std::cerr << "Unable to setup ISR: "
                  << strerror(errno) << std::endl;
        return -1;
    }
    /* Register ISR while hall sensor rising and falling */
    if (wiringPiISR(LEFT_HALL_B, INT_EDGE_BOTH, &hallSensor_ISR) < 0) {
        std::cerr << "Unable to setup ISR: "
                  << strerror(errno) << std::endl;
        return -1;
    }
    /* Register ISR while hall sensor rising and falling */
    if (wiringPiISR(RIGHT_HALL_A, INT_EDGE_BOTH, &hallSensor_ISR) < 0) {
        std::cerr << "Unable to setup ISR: "
                  << strerror(errno) << std::endl;
        return -1;
    }
    /* Register ISR while hall sensor rising and falling */
    if (wiringPiISR(RIGHT_HALL_B, INT_EDGE_BOTH, &hallSensor_ISR) < 0) {
        std::cerr << "Unable to setup ISR: "
                  << strerror(errno) << std::endl;
        return -1;
    }

    return 0;
}

int initPWM() {
    /* set pwm pin as output */
    pinMode(leftPWM, PWM_OUTPUT);
//    pinMode(leftPWMn, PWM_OUTPUT);
    pinMode(rightPWM, PWM_OUTPUT);
//    pinMode(rightPWMn, PWM_OUTPUT);

    pwmWrite(leftPWM, static_cast<int>(1024*0.5));
    pwmWrite(rightPWM, static_cast<int>(1024*0.5));

    return 0;
}
