/****************************************************************************
 * process_users.cpp
****************************************************************************/

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <math.h>

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
	kinect::User userT = user;
	userT.pos.x = user.pos.x + 0.033 * user.vel.x + ((0.033 / EP) * HP) * (pt.X - user.pos.x);
	userT.pos.y = user.pos.y + 0.033 * user.vel.y + ((0.033 / EP) * HP) * (pt.Y - user.pos.y);
	userT.pos.z = user.pos.z + 0.033 * user.vel.z + ((0.033 / EP) * HP) * (pt.Z - user.pos.z);
	userT.vel.x = user.vel.x + (0.033 / (EP * EP)) * HV * (pt.X - user.pos.x);
	userT.vel.y = user.vel.y + (0.033 / (EP * EP)) * HV * (pt.Y - user.pos.y);
	userT.vel.z = user.vel.z + (0.033 / (EP * EP)) * HV * (pt.Z - user.pos.z);
	user = userT;
	ROS_INFO("Estimation for User %d:\n\t\tX\tY\tZ\nPosition\t%.3f\t%.3f\t%.3f\nVelocity\t%.3f\t%.3f\t%.3f\n\n",
			  user.id, user.pos.x / 1000, user.pos.y / 1000, user.pos.z / 1000,
			  user.vel.x / 1000, user.vel.y / 1000, user.vel.z / 1000);
}

