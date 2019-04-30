//  Copyright (c) 2019 Tsung-Han Brian Lee
/*********************************************************
 ** ---------------- roboticVA_pos.cpp -------------------
 **   AUTHOR    : Tsung-Han Brian Lee
 **   REFERENCE : Revised from Dynamic Systems n
 **               Control Lab
 **   LICENSE     : MIT
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
 *********************************************************/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#define PI               3.14159265358979323
#define ROBOT_POSE_TOPIC     "/robot_pose"
#define ROBOT_STATE_TOPIC    "/feedback_vel"

/******************************************
 **     Help Fcn & Structure
 ******************************************/
typedef geometry_msgs::Twist      Twist;
typedef nav_msgs::Odometry        Odometry;
typedef geometry_msgs::Quaternion Quaternion;

typedef struct {
    double x;       // x direction
    double y;       // y direction
    double th;      // angular
} PlanarInfo;

void handleSetVelocity(const Twist& msg);

/******************************************
 ** Global Variable :
 **   => To be caught in callback fcn
 ******************************************/
volatile PlanarInfo vehicle_vel = {0.0, 0.0, 0.0};

int main(int argc, char* argv[]) {
    /* Init ROS node : odemetry_pub */
    ros::init(argc, argv, "odometry_pub");

    ros::NodeHandle nh;

    /* Declaration of ROS Publisher & Subscriber */
    ros::Publisher  odom_pub = nh.advertise<Odometry>(ROBOT_POSE_TOPIC, 50);
    ros::Subscriber sub      = nh.subscribe(ROBOT_STATE_TOPIC,
                                              100, handleSetVelocity);

    PlanarInfo robot_pos = {0.0, 0.0, 0.0};   //  record robot position
    ros::Rate loopRate(100.0);                //  looping rate
    ros::Time curTime;                        //  current Time step
    ros::Time prevTime = ros::Time::now();    //  last Time step

    while (ros::ok()) {
        ros::spinOnce();            //  be able to handle callback fcn
        curTime = ros::Time::now();

        double timeFly = (curTime - prevTime).toSec();

        robot_pos.x  += (vehicle_vel.x * cos(robot_pos.th)
                         - vehicle_vel.y * sin(robot_pos.th)) * timeFly;
        robot_pos.y  += (vehicle_vel.x * sin(robot_pos.th)
                         + vehicle_vel.y * cos(robot_pos.th)) * timeFly;
        robot_pos.th += vehicle_vel.th * timeFly;

        //  Limit the angle in range 0 ~ 2*PI
        if (robot_pos.th >= 2*PI) {
            robot_pos.th -= 2*PI;
        } else if (robot_pos.th < 0) {
            robot_pos.th += 2*PI;
        }

        /**
         ** --- Publish odometry info ove ROS ---
         **  1. Fill in position & velocity info
         **  2. Publish to topic
         **/
        Odometry odom;

        odom.header.stamp     = curTime;
        odom.header.frame_id  = "map";
        odom.child_frame_id   = "base_link";

        odom.pose.pose.position.x    = robot_pos.x;
        odom.pose.pose.position.y    = robot_pos.y;
        odom.pose.pose.position.z    = 0.0;
        odom.pose.pose.orientation.z = robot_pos.th;

        odom.twist.twist.linear.x  = vehicle_vel.x;
        odom.twist.twist.linear.y  = vehicle_vel.y;
        odom.twist.twist.angular.z = vehicle_vel.th;

        odom_pub.publish(odom);

        prevTime = curTime;
        loopRate.sleep();
    }

    return 0;
}

void handleSetVelocity(const Twist& msg) {
    vehicle_vel.x  = msg.linear.x;
    vehicle_vel.y  = msg.linear.y;
    vehicle_vel.th = msg.angular.z;
}
