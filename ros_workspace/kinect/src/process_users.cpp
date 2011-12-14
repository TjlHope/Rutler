/****************************************************************************
 * process_users.cpp
****************************************************************************/

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

//#include <time.h>
//#include <iostream>
//#include <sys/signal.h>
//#include <sys/stat.h>
//#include <sys/types.h>
//#include <sys/time.h>
#include <math.h>
//#include <cmath>

#include "process_users.h"


//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------

#define _POSIX_SOURCE 1 /* POSIX compliant source */

//#define PI 3.14159265

//-- Declaration of estimation values --//
#define HP 5 //4.5
#define HV 6
#define EP 0.1 //0.07


//---------------------------------------------------------------------------
// Function Definitions
//---------------------------------------------------------------------------

void estimateVelocity(XnPoint3D pt, kinect::User& user)
{
	kinect::User userT;
	userT.pos.x = user.pos.x + 0.033 * user.pos.x + ((0.033 / EP) * HP) * (pt.X - user.pos.x);
	userT.pos.y = user.pos.y + 0.033 * user.pos.y + ((0.033 / EP) * HP) * (pt.Y - user.pos.y);
	userT.pos.z = user.pos.z + 0.033 * user.pos.z + ((0.033 / EP) * HP) * (pt.Z - user.pos.z);
	userT.vel.x = user.vel.x + (0.033 / (EP * EP)) * HV * (pt.X - user.pos.x);
	userT.vel.y = user.vel.y + (0.033 / (EP * EP)) * HV * (pt.Y - user.pos.y);
	userT.vel.z = user.vel.z + (0.033 / (EP * EP)) * HV * (pt.Z - user.pos.z);
	user = userT;
	ROS_INFO("Esimatation:\n\t\tX\tY\tZ\nPosition\t%.2f\t%.2f\t%.2f\nVelocity\t%.2f\t%.2f\t%.2f\n\n",
			 user.pos.x, user.pos.y, user.pos.z, user.vel.x, user.vel.y, user.vel.z);
}

