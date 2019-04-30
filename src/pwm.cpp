/* file: pwm.cpp */

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

const int PWM_pin1  = 1;  /* GPIO 1 for wiringPi = GPIO 18 for BCM */
const int PWM_pin2  = 23; /* GPIO 23 for wiringPi = GPIO 13 for BCM */
// const int PWM_pin2N = 24; /* GPIO 24 for wiringPi = GPIO 19 for BCM */
// const int PWM_pin1N = 26; /* GPIO 26 for wiringPi = GPIO 12 for BCM */

int main(int argc, char* argv[])
{
    if(wiringPiSetup() == -1){
        exit(-1);
    }

    /* set pwm pin as output */
    pinMode(PWM_pin1, PWM_OUTPUT);
    // pinMode(PWM_pin1N, PWM_OUTPUT);
    pinMode(PWM_pin2, PWM_OUTPUT);
    // pinMode(PWM_pin2N, PWM_OUTPUT);

    while(1){
        double d1, d2;

        std::cout << "input duty ratio 1 & duty ratio 2: ";
        std::cin >> d1 >> d2;

        pwmWrite(PWM_pin1, (int)1024*d1);
        //pwmWrite(PWM_pin1N, 256);
        pwmWrite(PWM_pin2, (int)1024*d2);
        //pwmWrite(PWM_pin2N, 768);
        sleep(3);

/*
        fprintf(stdout, "backward ...\n");
        pwmWrite(PWM_pin1, 768);
        //pwmWrite(PWM_pin1N, 768);
        pwmWrite(PWM_pin2, 256);
        //pwmWrite(PWM_pin2N, 256);
        sleep(3);

        fprintf(stdout, "stop for awhile ...\n");
        pwmWrite(PWM_pin1, 512);
        //pwmWrite(PWM_pin1N, 512);
        pwmWrite(PWM_pin2, 512);
        //pwmWrite(PWM_pin2N, 512);
        sleep(3);
*/
    }

    return 0;
}
