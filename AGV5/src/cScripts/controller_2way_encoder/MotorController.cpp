#include <Arduino.h>
#include "MotorController.h"
#include <ros.h>
#include <AGV5/target.h>


MotorController::MotorController(float* k_p, float* k_d, float* k_i, ros::NodeHandle* nodehandle) : mr_opins{13, 12, 11},mr_ipins{2, 3},ml_opins{5, 6, 7},ml_ipins{18, 19}, nh_(*nodehandle)
{
		stndby = 9;
		

		PULSE_PR_TURN = 12;
		
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
}

void MotorController::isrR(int ct) {
	r_state += ct;
}

void MotorController::isrL(int ct) {
	l_state += ct;
}

void MotorController::resetState() {
	r_state = 0;
	l_state = 0;
}

int MotorController::getR() {
		return r_state; 
}
int MotorController::getL() {
		return l_state; 
}

void MotorController::get_velocity(unsigned int* t){
	noInterrupts();
	getRpms(getR(), getL(), *t);
	resetState();
	interrupts();
	*t = (unsigned int)millis();
}

void MotorController::driveWheels(bool isRight, float sig){
        volatile int vel = int(sig / 0.5 * 245);
	// Saturate
	if (vel > 245) vel = 245;
	if (vel < -245) vel = -245;

	digitalWrite(stndby, HIGH);
	if (isRight){
		if (vel > 0){
			digitalWrite(mr_opins[1], HIGH);
			digitalWrite(mr_opins[2], LOW);
		}
		else{
			digitalWrite(mr_opins[1], LOW);
			digitalWrite(mr_opins[2], HIGH);
		}
		analogWrite(mr_opins[0], abs(vel));  
	}
	else{
		if (vel < 0){
			digitalWrite(ml_opins[1], HIGH);
			digitalWrite(ml_opins[2], LOW);
		}
		else{
			digitalWrite(ml_opins[1], LOW);
			digitalWrite(ml_opins[2], HIGH);
		}
		analogWrite(ml_opins[0], abs(vel)); 
	}

}

void MotorController::getRpms(int count_r, int count_l, unsigned int prevTime){
        float rps_r = ( (count_r * 1000.f / ((unsigned int)millis() - prevTime)) ) / (PULSE_PR_TURN * 150.58);
	float rps_l = ( (count_l * 1000.f / ((unsigned int)millis() - prevTime)) ) / (PULSE_PR_TURN * 150.58);

	vright = 2*PI * rps_r / 50;	// m/s
	vleft = 2*PI * rps_l / 50; // m/s

	float wheel_base = 0.125;
	float tolerance = 0.01;
	if (abs(vleft - vright) < tolerance){
		velocity_msg.linvel = (vleft + vright) / 2;
		velocity_msg.angvel = 0; 
	} 
	else if (abs(vleft + vright) < tolerance){
		velocity_msg.linvel = 0;
		velocity_msg.angvel = (vright - vleft) / wheel_base;
	}
	else {
	  float ICC_r = (wheel_base / 2) * (vleft + vright) / (vright - vleft);
	  float omega = (vright - vleft) / wheel_base;
	  velocity_msg.linvel = omega * ICC_r;
	  velocity_msg.angvel = omega;        
	}

          //velocity_msg.linvel = vright;
	  //velocity_msg.angvel = vleft;

}

void MotorController::pidWheels(float target, float rpm, bool isRight, float& sig, float& last_e, float& sum_e) {
	float error = target - rpm;
	sum_e = sum_e + error;

	if (rpm == 0) {
		sig = 0;
		sum_e = 0; 
		last_e = 0; 
	}

	sig = sig + (error * kp + (error - last_e) * kd + sum_e * ki);
        Serial.println(target);
        Serial.println(rpm);
        Serial.println(sig);
	driveWheels(isRight, sig);
	last_e = error;
}

void MotorController::reset(){
	sig_r = 0;
	sig_l = 0;
	prevE_r, eSum_r = 0;
	prevE_l, eSum_l = 0;  
	driveWheels(1, 0);
	driveWheels(0, 0);
}

void MotorController::driver(float target_linvel, float target_angvel){
	if (target_linvel == 0 && target_angvel == 0){
	  reset();
	}  
	else {
		float right_veltarget = (2*target_linvel+target_angvel*0.125) / 2;
		float left_veltarget = (2*target_linvel-target_angvel*0.125) / 2;
		pidWheels(right_veltarget, vright, bool(1), sig_r, prevE_r, eSum_r);
		pidWheels(left_veltarget, vleft, bool(0), sig_l, prevE_l, eSum_l);
	}	
}
