/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

void messageCb( const std_msgs::Int16& toggle_msg){
        Serial.println(toggle_msg.data);
	if(toggle_msg.data == 0)
  		digitalWrite(2, LOW);   // Turn Off the led
  	else
  		digitalWrite(2, HIGH);   // Turn On the led
}

ros::Subscriber<std_msgs::Int16> sub("LEDaction", &messageCb );

void setup()
{
  pinMode(2, OUTPUT);
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
