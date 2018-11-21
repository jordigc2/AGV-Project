/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

void messageCb( const std_msgs::Int16& toggle_msg){
	if(toggle_msg == 0)
  		digitalWrite(13, LOW);   // Turn Off the led
  	else
  		digitalWrite(13, HIGH);   // Turn On the led
}

ros::Subscriber<std_msgs::Int16> sub("LEDaction", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}