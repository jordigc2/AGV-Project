#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"


// Arduino -> Motor Driver pins
// Right motor drive pins
const int mr_pwm = 13;
const int mr_in1 = 12;
const int mr_in2 = 11;

// left motor drive pins
const int ml_pwm = 5;
const int ml_in1 = 6;
const int ml_in2 = 7;


// needs to be pulled high before motors can drive
const int stndby = 9;

// Encoder -> Arduino pins
const int mr_A = 2;   // right works  interrupt 0
const int mr_B = 3;  // right works  interrupt 1

const int ml_A = 18;  // left works interrupt 5
const int ml_B = 19;  // left works interrupt 4

// Counting rotations
volatile unsigned int r_state = 0;
volatile unsigned int l_state = 0;

int SAMPLE_DELAY = 20;
int PULSE_PR_TURN = 12;
const int sample_duration = 200;

unsigned int pTime;
unsigned int timeline;
float rpm; 
float rads;

float s = 0;
int sig = 0; 

float time[sample_duration];
float rpms[sample_duration];
int count = 0; 

float maxspeed = 28.83;
float last_e;
ros::NodeHandle nh;

std_msgs::Float32 rpm_val;
ros::Publisher rpm_pub("/arduino/rpm", &rpm_val);


void setup()
{ 

  
  pinMode(mr_pwm, OUTPUT);
  pinMode(mr_in1, OUTPUT);
  pinMode(mr_in2, OUTPUT);
  
  pinMode(mr_A, INPUT);  
  pinMode(mr_B, INPUT);
  
// ---------------------------------------------------------------------------------  
  
  pinMode(ml_pwm, OUTPUT);
  pinMode(ml_in1, OUTPUT);
  pinMode(ml_in2, OUTPUT);

  pinMode(ml_A, INPUT);  
  pinMode(ml_B, INPUT);

  pinMode(stndby, OUTPUT);

// --------------------------------------------------------------------------------- 
  // Interrupt init. Parameters are: 
  // interrupt pin (not all pins can be interrupts), ISR function, mode
  // attachInterrupt(digitalPinToInterrupt(mr_A), isr_r, CHANGE);
  attachInterrupt(0, isr_r, CHANGE);
  attachInterrupt(1, isr_r, CHANGE);
  
  attachInterrupt(5, isr_l, CHANGE);
  attachInterrupt(4, isr_l, CHANGE);

  Serial.begin(57600);



  nh.initNode();
 // nh.subscribe(sub);
  nh.advertise(rpm_pub);
  
  

  //drive_left(195, true);
  //drive_right(120, true); 
    
    
  timeline = (unsigned int)millis();

  pTime = (unsigned int)millis();
  
}

int target = 0;

void loop()
{
  
    
    if ((unsigned int)millis() - timeline > 1000 && (unsigned int)millis() - timeline < 1050){
       target = 15;
       //drive_right(255, true); 
    }

    if ((unsigned int)millis() - pTime >= SAMPLE_DELAY && count < sample_duration)
      {

        unsigned int pulses;
        noInterrupts();
        pulses = r_state;
        r_state = 0;
        interrupts();

        
        rpm = ( (pulses * 1000.f / ((unsigned int)millis() - pTime)) ) / (PULSE_PR_TURN * 150.58);
        rads = 2 * 3.14 * rpm;
        pTime = (unsigned int)millis();
       
        rpms[count] = rads;
        count ++; 
        
        pid(target, rads, (2), sig); 
        //pi(target, rads, 5, 6, 1, s, last_e);
      }

    if (count == sample_duration){
      drive_right(0,false);

      for (int i=0;i<sample_duration;i++){
        rpm_val.data = rpms[i];
        rpm_pub.publish(&rpm_val);
        delay(10);
        nh.spinOnce();
      }    

    delay(2000);
    count = 0; 
    r_state = 0; 
    timeline = (unsigned int)millis();
    pTime = (unsigned int)millis();
    target = 0;
  }
  
 nh.spinOnce();

} 


/*    
Potentially look into cli() and sei(), which turns off and on interrupts:
It seems they fuck up Serial.print on tinkercad ... maybe more things. 
More importantly, since cli() might fuck up Serial.print, it might mess whith
the arduino <-> rpi serial connection. (I'm assuming that must work as some sort of interrupt)

*/ 

// --------------- DRIVE FUNCTIONS -----------------------------------------------------------

void drive_right(int vel, bool dir){    // True: Forward

  if (vel > 255) vel = 255;
      
  digitalWrite(stndby, HIGH);
  if (dir == true){
    digitalWrite(mr_in1, HIGH);
    digitalWrite(mr_in2, LOW);
  }
  else{
    digitalWrite(mr_in1, LOW);
    digitalWrite(mr_in2, HIGH);
  }
  analogWrite(mr_pwm, vel);
}

void drive_left(int vel, bool dir){     // True: Forward 

  if (vel > 255) vel = 255;
      
  digitalWrite(stndby, HIGH);
  if (dir == true){
    digitalWrite(ml_in1, LOW);
    digitalWrite(ml_in2, HIGH);
  }
  else{
    digitalWrite(ml_in1, HIGH);
    digitalWrite(ml_in2, LOW);
  }
  analogWrite(ml_pwm, vel);  
}

// --------------- ISR FUNCTIONS -----------------------------------------------------------
void isr_r()
{

      r_state ++;
      //Serial.println("wow   RIGHT");
}

void isr_l()
{

      l_state ++;
      //Serial.println("wow   LEFT");
}


float getRps(int count, unsigned long prevTime, int cpr) {
    float rotations;
    float rps;
    

    rotations = count / (150 * cpr); 
    rps = rotations / ((micros() - prevTime) * 1000000);
    return rotations;
}


void pid(int target, float rad, float kp, int& sig){
   // Serial.print("bef  ");
   // Serial.println(sig);
    float error = target - rad;
    
    if (rad == 0 || target == 0) {
     sig = 0; 
    }
  
  
  sig = sig + int(error * kp);
  drive_right(sig, true);
 // Serial.println(sig);
       // rpm_val.data = rad;
       // rpm_pub.publish(&rpm_val);
  
}

void pi(int target, float rad, float kp, float ki, float kd, float& sum, float& last_e){
   // Serial.print("bef  ");
   // Serial.println(sig);
    int sig;

    float error = target - rad;
    sum = sum + error;
    
    if (rad == 0 || target == 0) {
     sum = 0; 
    }
  
  
  sig = int(error * kp + sum * ki + (error - last_e) * kd);
  drive_right(sig, true);

  last_e = error;
 // Serial.println(sig);
       // rpm_val.data = rad;
       // rpm_pub.publish(&rpm_val);
  
}
