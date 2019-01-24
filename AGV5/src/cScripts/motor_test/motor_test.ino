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
const int mr_A = 2;
const int mr_B = 3;

const int ml_A = 18;
const int ml_B = 19;



// Encoder pulse counters
volatile unsigned int r_state = 0;
volatile unsigned int l_state = 0;

// RPM calculation parameters 
int SAMPLE_DELAY = 1000;    // Frequency of control update
int PULSE_PR_TURN = 6;     


unsigned int pTime;

int sig_r;
int sig_l;

float prevE_r;
float prevE_l;

float eSum_r;
float eSum_l;

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
  attachInterrupt(0, isr_r, CHANGE);
  attachInterrupt(5, isr_l, CHANGE);
  
  Serial.begin(9600);


 // drive_left(255, true);
  drive_right(255, true);

  
} 
void loop()
{

  if ((unsigned int)millis() - pTime >= SAMPLE_DELAY)
    {
      float rpm_r, rpm_l;
      float vel_r, vel_l;
      unsigned int pulses_r, pulses_l;

      noInterrupts();
      pulses_r = r_state;
      r_state = 0;

      pulses_l = l_state;
      l_state = 0;
      interrupts();

      getRpms(pulses_r, pulses_l, pTime, rpm_r, rpm_l, vel_r, vel_l); 
      pTime = (unsigned int)millis();
    
      
      Serial.print("RPM :");     // There is a fairly decent chance that all these Serial.print statements impacts the performance of the code negatively 
      Serial.println(rpm_r);
      Serial.print("Vel (rad/s) : ");
      Serial.println(vel_r); 
    
    }
}


// --------------- DRIVE FUNCTIONS -----------------------------------------------------------
             
void drive_right(int vel, bool dir){    // True: Forward

 // if (vel > 200) vel = 200; 
  if (vel < 0) vel = 0;
      
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

 // if (vel > 200) vel = 200;
  if (vel < 0) vel = 0;
      
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
}

void isr_l()
{
      l_state ++;
}

void getRpms(int count_r, int count_l, unsigned long prevTime, float& rpm1, float& rpm2, float& vel1, float& vel2) {

    rpm1 = ( (count_r * 60000.f / ((unsigned int)millis() - prevTime)) ) / (PULSE_PR_TURN * 150.58);
    rpm2 = ( (count_l * 60000.f / ((unsigned int)millis() - prevTime)) ) / (PULSE_PR_TURN * 150.58);   

    vel1 = 2*PI * (rpm1/60.0);  // rad/s
    vel2 = 2*PI * (rpm2/60.0); // rad/s

  
}

void pid_controller(float target, float rpm, bool mot, int& sig, float& le, float& se)
{

/* PID-parameters needs to be tuned properly.
 * PID can be changes to PD, PI & P by setting the respective parameters to 0. 
 * i.e. for a PD controller, ki is set to 0 
 * Maybe round() would be a better choice than int(). 
 */
 

// Right motor == 1, left motor == 0. 
float kp = 0.5;
float kd = 0.5;
float ki = 0.15;
float error = target - rpm;

se = se + error;

if (rpm == 0) {
  sig = 0;
  se = 0; 
}

sig = sig + int(error * kp + (error - le) * kd + se * ki);

if (mot == 1) drive_right(sig, true);
else drive_left(sig,true); 

le = error;
}

