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



// Counting rotations
volatile unsigned int r_state = 0;
volatile unsigned int l_state = 0;
unsigned int prev_state = 0; 

int SAMPLE_DELAY = 1000;
int PULSE_PR_TURN = 6;


unsigned long ISRinterval; // Time between to interrupts in micro s
unsigned long prev_ISRinterval; 
bool newISR = false; // not used 

unsigned int pTime;
float rpm; 


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
  attachInterrupt(ml_A, isr_l, CHANGE);
  
  Serial.begin(9600);


  drive_left(100, true);
  drive_right(50, true);

  
}
void loop()
{

  if ((unsigned int)millis() - pTime >= SAMPLE_DELAY)
    {
      unsigned int pulses;
      noInterrupts();
      pulses = l_state;
      l_state = 0;
      interrupts();

      rpm = ( (pulses * 60000.f / ((unsigned int)millis() - pTime)) ) / (PULSE_PR_TURN * 150.58);
      pTime = (unsigned int)millis();
      Serial.println(rpm); 
    }

  
  /*
  if (l_state == 1000)
  { // If an interrupt has occured since last time
      rpm = 1.105 / (millis() - pTime) * 1000;
      Serial.println(rpm*60);
      l_state = 0;
      pTime = millis();
      
  }

*/
 // prev_ISRinterval = ISRinterval ;
 // prev_state = l_state; 
/*

Potentially look into cli() and sei(), which turns off and on interrupts:
It seems they fuck up Serial.print on tinkercad ... maybe more things. 
More importantly, since cli() might fuck up Serial.print, it might mess whith
the arduino <-> rpi serial connection. (I'm assuming that must work as some sort of interrupt)

*/ 

// --------------- DRIVE FUNCTIONS -----------------------------------------------------------

}             
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
//    current = millis();
      r_state ++;


  /*
    unsigned long ISRmicros;
    static unsigned long prevISRmicros;
    ISRmicros = micros();
    ISRinterval = (ISRmicros - prevISRmicros);

    prevISRmicros = ISRmicros;
    
    newISR = true; 

  */
  // micros() is used since millis() doesn't work in ISR functions
}

void isr_l()
{
//    current = millis();
      l_state ++;


  /*
    unsigned long ISRmicros;
    static unsigned long prevISRmicros;
    ISRmicros = micros();
    ISRinterval = (ISRmicros - prevISRmicros);

    prevISRmicros = ISRmicros;
    
    newISR = true; 

  */
  // micros() is used since millis() doesn't work in ISR functions
}


float getRps(int count, unsigned long prevTime, int cpr) {
    float rotations;
    float rps;

    rotations = count / (150 * cpr); 
    rps = rotations / ((micros() - prevTime) * 1000000);
    return rotations;
}


