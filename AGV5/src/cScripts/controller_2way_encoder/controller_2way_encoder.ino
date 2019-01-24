#include "MotorController.h"
#include "ros.h"
#include "AGV5/target.h"

#define encoder0PinA  2 //right
#define encoder0PinB  3
#define encoder1PinA 19 //left
#define encoder1PinB 18

float kp = 1;
float kd = 0.0;
float ki = 0.0;

const int SAMPLE_DELAY = 10;

float linTarget = 0.0;
float angTarget = 0.0;

ros::NodeHandle nh;

void messageCb(const AGV5::target& data){
	linTarget = data.linvel;
	angTarget = data.angvel;
}

AGV5::target velocity_msg;

ros::Subscriber<AGV5::target> sub("/some/topic", &messageCb );
ros::Publisher vel_pub("/arduino/vel", &velocity_msg);
MotorController driver(&kp, &kd, &ki, &nh);

unsigned int timer = (unsigned int)millis();
unsigned int t = (unsigned int)millis();


/***	Setup    ***/
void setup(){
Serial.begin(57600);

attachInterrupt(0, doEncoder0A, CHANGE); // Encoder right pin 2
attachInterrupt(1, doEncoder0B, CHANGE); // pin 3
attachInterrupt(5, doEncoder1A, CHANGE); // Encoder left pin 19
attachInterrupt(4, doEncoder1B, CHANGE); // pin 18

nh.initNode();
nh.subscribe(sub);
nh.advertise(vel_pub);
}


/***	Loop    ***/
void loop(){
	if ((unsigned int)millis() - timer > SAMPLE_DELAY){
		timer = (unsigned int)millis();
		driver.get_velocity(&t);
		vel_pub.publish(&velocity_msg);
		driver.driver(linTarget, angTarget);
	}
	nh.spinOnce();
}


/****	Ugly as hell Encoder handle    ***/
void doEncoder0A() {
	// look for a low-to-high on channel A
	if (digitalRead(encoder0PinA) == HIGH) {

		// check channel B to see which way encoder is turning
		if (digitalRead(encoder0PinB) == LOW) {
			driver.isrR(-1);         // CW
		}
		else {
			driver.isrR(1);         // CCW
		}
	}

	else   // must be a high-to-low edge on channel A
	{
		// check channel B to see which way encoder is turning
		if (digitalRead(encoder0PinB) == HIGH) {
			driver.isrR(-1);          // CW
		}
		else {
			driver.isrR(1);          // CCW
		}
	}
}

void doEncoder0B() {
	// look for a low-to-high on channel B
	if (digitalRead(encoder0PinB) == HIGH) {

		// check channel A to see which way encoder is turning
		if (digitalRead(encoder0PinA) == HIGH) {
			driver.isrR(-1);         // CW
		}
		else {
			driver.isrR(1);         // CCW
		}
	}
	// Look for a high-to-low on channel B
	else {
		// check channel B to see which way encoder is turning
		if (digitalRead(encoder0PinA) == LOW) {
			driver.isrR(-1);          // CW
		}
		else {
			driver.isrR(1);          // CCW
		}
	}
}

void doEncoder1A() {
	// look for a low-to-high on channel A
	if (digitalRead(encoder1PinA) == HIGH) {

		// check channel B to see which way encoder is turning
		if (digitalRead(encoder1PinB) == LOW) {
			driver.isrL(1);         // CW
		}
		else {
			driver.isrL(-1);         // CCW
		}
	}

	else   // must be a high-to-low edge on channel A
	{
		// check channel B to see which way encoder is turning
		if (digitalRead(encoder1PinB) == HIGH) {
			driver.isrL(1);          // CW
		}
		else {
			driver.isrL(-1);          // CCW
		}
	}
}

void doEncoder1B() {
	// look for a low-to-high on channel B
	if (digitalRead(encoder1PinB) == HIGH) {

		// check channel A to see which way encoder is turning
		if (digitalRead(encoder1PinA) == HIGH) {
			driver.isrL(1);         // CW
		}
		else {
			driver.isrL(-1);         // CCW
		}
	}
	// Look for a high-to-low on channel B
	else {
		// check channel B to see which way encoder is turning
		if (digitalRead(encoder1PinA) == LOW) {
			driver.isrL(1);          // CW
		}
		else {
			driver.isrL(-1);          // CCW
		}
	}
}
