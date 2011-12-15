/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

// Try and hack roslib to work with make
#define __AVR_ATmega1280__
#include <HardwareSerial.h>
extern HardwareSerial Serial = Serial1;

#include <WProgram.h>
#include <Servo.h> 

#include <ros.h>
#include <std_msgs/Int8.h>

ros::NodeHandle  nh;

Servo servo;

int angle = 90;
int target = angle;

std_msgs::Int8 position;
std_msgs::Int8 increment;

ros::Publisher pos_pub("servo_position", &position);
ros::Publisher inc_pub("servo_increment", &increment);

void servo_cb(const std_msgs::Int8& cmd_msg)
{
	target = int(cmd_msg.data) + 90; // msg is with center as 0
	angle = servo.read();
	int offset = target - angle;
	increment.data = offset;
	inc_pub.publish(&increment);
	digitalWrite(13, HIGH-digitalRead(13));
}

ros::Subscriber<std_msgs::Int8> sub("servo", servo_cb);

void setup()
{
	pinMode(13, OUTPUT);

	nh.initNode();
	nh.advertise(pos_pub);
	nh.advertise(inc_pub);
	nh.subscribe(sub);

	servo.attach(9); //attach it to pin 9
	servo.write(angle);
}

void loop()
{
	nh.spinOnce();
    while(angle != target)
    {
		angle = servo.read();
		if(angle < target)
			angle += 1;
		else if(angle > target)
			angle -= 1;
		servo.write(angle);
		delay(50);
    }
	position.data = angle - 90;	// align center as 0
	pos_pub.publish(&position);
	delay(50);
}

