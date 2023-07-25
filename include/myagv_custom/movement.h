#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include "ros/ros.h" 
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
 
// initialize publisher
void init_myagv(int argc, char *argv[]);
// move robot
// x and y speed in m/s, turn angle in degrees, length in seconds
void move_myagv(float xspeed, float yspeed, float turnangle, float length);
// spin robot
// speed in degrees per second, length in seconds
void spin_myagv(float speed, float length);

// max values of ros messages are all 1.0
extern float MAXSPIN; // 45 deg / s
extern float MAXSPEED; // 0.9 m / s

extern ros::Publisher pub;
extern geometry_msgs::Twist msg;

#endif