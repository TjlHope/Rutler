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
#include <std_msgs/Int8.h>

ros::NodeHandle  nh;

Servo servo;

std_msgs::Int8 position;
int angle = 90;

ros::Publisher pub("servo_position", &position);

void servo_cb(const std_msgs::Int8& cmd_msg)
{
	angle = servo.read();
	int offset = int(cmd_msg.data);
        angle += offset;
        position.data = offset;
        pub.publish(&position);
	servo.write(angle);
	digitalWrite(13, HIGH-digitalRead(13));
}


ros::Subscriber<std_msgs::Int8> sub("servo", servo_cb);

void setup()
{
	pinMode(13, OUTPUT);

	nh.initNode();
        nh.advertise(pub);
	nh.subscribe(sub);

	servo.attach(9); //attach it to pin 9
        servo.write(angle);
}

void loop()
{
	nh.spinOnce();
	position.data = angle;
	pub.publish(&position);
	delay(1000);
}

