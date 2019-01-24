#include "MotorController.h"
#include "ros.h"
#include "AGV5/target.h"

void isrDispatchR();
void isrDispatchL();

float kp = 5;
float kd = 6.0;
float ki = 1.0;

const int SAMPLE_DELAY = 20;

float linTarget = 0;
float angTarget = 0;

ros::NodeHandle nh;

void messageCb(const AGV5::target& data){
  linTarget = data.linvel;
  angTarget = data.angvel;
  //Serial.println("HAHHAHHAHAHAHAHAHHHAHAHHAHAHAHAHAHH");  
}

ros::Subscriber<AGV5::target> sub("/some/topic", &messageCb );

MotorController driver(&kp, &kd, &ki, &nh);

unsigned int timer = (unsigned int)millis();


unsigned int t = (unsigned int)millis();

void setup(){

Serial.begin(57600);

attachInterrupt(0, isrDispatchR, CHANGE);
attachInterrupt(5, isrDispatchL, CHANGE);

nh.initNode();
nh.subscribe(sub);

}

void loop(){
 // (lin_vel*, ang_vel*) 

if ((unsigned int)millis() - timer > SAMPLE_DELAY){
    driver.driver(linTarget, angTarget, t);
    timer = (unsigned int)millis();
}

 nh.spinOnce();
}

void isrDispatchR(){
  driver.isrR();
}
void isrDispatchL(){
  driver.isrL();
}


  
