/****************************************************************************
 * process_users.h
****************************************************************************/
#ifndef KINECT_PROCESS_USERS
#define KINECT_PROCESS_USERS

#include <XnCppWrapper.h>

// Ros setup
#include "ros/ros.h"
#include "kinect/User.h"

// defines
#define FALSE 0
#define TRUE 1

void estimateVelocity(XnPoint3D pt, kinect::User& user);

#endif
