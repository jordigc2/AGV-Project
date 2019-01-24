#include <Arduino.h>
#include "MotorController.h"
#include <ros.h>
#include <AGV5/target.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

MotorController::MotorController(float* k_p, float* k_d, float* k_i, ros::NodeHandle* nodehandle) : mr_opins{13, 12, 11},mr_ipins{2, 3},ml_opins{5, 6, 7},ml_ipins{18, 19}, nh_(*nodehandle)
{
		stndby = 9;
		

		PULSE_PR_TURN = 6;
		
		for(int i = 0; i < 3; i++){
			pinMode(mr_opins[i], OUTPUT);
			pinMode(ml_opins[i], OUTPUT);
			if(i < 2){
				pinMode(mr_ipins[i], INPUT);
				pinMode(ml_ipins[i], INPUT);
			}
		}
		
		pinMode(stndby, OUTPUT);

                kp = *k_p;
                kd = *k_d;
                ki = *k_i;
                
                AGV5::target msg;
                
                //ros::Publisher pub = n.advertise<AGV5::target>"wheels/velocity", &msg);
                

}

void cb(AGV5::target& msg){
  
}

void MotorController::isrR() {
	r_state ++;
}

void MotorController::isrL() {
	l_state ++;
}

void MotorController::driveRight(int vel, bool dir){
	
	if (vel > 245) vel = 245;
        if (vel < 0) vel = 0;

	digitalWrite(stndby, HIGH);
	if (dir == true){
		digitalWrite(mr_opins[1], HIGH);
		digitalWrite(mr_opins[2], LOW);
	}
	else{
		digitalWrite(mr_opins[1], LOW);
		digitalWrite(mr_opins[2], HIGH);
	}
	analogWrite(mr_opins[0], vel);  
}



void MotorController::driveLeft(int vel, bool dir){
	
	if (vel > 245) vel = 245;
        if (vel < 0) vel = 0;

	digitalWrite(stndby, HIGH);
	if (dir == true){
		digitalWrite(ml_opins[1], LOW);
		digitalWrite(ml_opins[2], HIGH);
	}
	else{
		digitalWrite(ml_opins[1], HIGH);
		digitalWrite(ml_opins[2], LOW);
	}
	analogWrite(ml_opins[0], vel);  

}

void MotorController::getRpms(int count_r, int count_l, unsigned long prevTime){
	float rps_l, rps_r;
	
	rps_r = ( (count_r * 1000.f / ((unsigned int)millis() - prevTime)) ) / (PULSE_PR_TURN * 150.58);
	rps_l = ( (count_l * 1000.f / ((unsigned int)millis() - prevTime)) ) / (PULSE_PR_TURN * 150.58);

	vright = 2*PI * rps_r;	// m/s
	vleft = 2*PI * rps_l; // m/s
    
}

void MotorController::pidWheels(float target, float rpm, bool mot, int& sig, float& last_e, float& sum_e) {

	bool direct;
  
        if (target < 0) target = abs(target), direct = false;
        else direct = true;
        
        float error = target - rpm;

	sum_e = sum_e + error;

	if (rpm == 0) {
		  sig = 0;
		  sum_e = 0; 
		  last_e = 0; 
	}

	sig = sig + int(error * kp + (error - last_e) * kd + sum_e * ki);

	if (mot == 1) driveRight(sig, direct);
	else driveLeft(sig, direct); 
        //Serial.println(sig);
	last_e = error;
}

void MotorController::reset(){
    
  sig_r = 0;
  sig_l = 0;
  prevE_r, eSum_r = 0;
  prevE_l, eSum_l = 0;  
  
  driveRight(0, 0);
  driveLeft(0, 0);
}


void MotorController::driver(float target_linvel, float target_angvel, unsigned int& t){
	
        
        if (target_linvel == 0 && target_angvel == 0){
          reset();
        }  
        else {
	      float right_veltarget = (2*target_linvel+target_angvel*0.125) / (2*0.02);
	      float left_veltarget = (2*target_linvel-target_angvel*0.125) / (2*0.02);

        //Serial.println(right_veltarget);
        //Serial.println(left_veltarget);
	
              

			noInterrupts();
			getRpms(r_state, l_state, t);
			pTime = (unsigned int)millis();
                        r_state = 0;
                        l_state = 0;
			interrupts();
			t = (unsigned int)millis();

			pidWheels(right_veltarget, vright, bool(1), sig_r, prevE_r, eSum_r);
			pidWheels(left_veltarget, vleft, bool(0), sig_l, prevE_l, eSum_l);
                        //Serial.print("Right RPM : ");
                        //Serial.println(vright);
                        //Serial.print("Left RPM :");
                        //Serial.println(vleft);
                        //Serial.print("Right Signal : ");
                        //Serial.println(sig_r);
                        //Serial.print("Left Signal : ");
                        //Serial.println(sig_l);
	}	
}
