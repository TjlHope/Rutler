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

#include <WProgram.h>

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle  nh;

Servo servo;

std_msgs::UInt8 position;
int angle = 90;

void servo_cb(const std_msgs::UInt8& cmd_msg)
{
	angle = servo.read();
	angle += int(cmd_msg.data) - 90;
	servo.write(angle);
	digitalWrite(13, HIGH-digitalRead(13));
}


ros::Subscriber<std_msgs::UInt8> sub("servo", servo_cb);
ros::Publisher pub("servo_position", &position);

void setup()
{
	pinMode(13, OUTPUT);

	nh.initNode();
	nh.subscribe(sub);

	servo.attach(9); //attach it to pin 9
}

void loop()
{
	nh.spinOnce();
	position.data = angle;
	pub.publish(&position);
	delay(1000);
}

