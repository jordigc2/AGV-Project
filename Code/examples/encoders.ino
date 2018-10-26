// Arduino -> Motor Driver pins
const int m1_pwm = 9;
const int m1_in1 = 11;
const int m1_in2 = 12;
const int stndby = 10;
// Encoder -> Arduino pins
const int m1_A = 2;
const int m1_B = 3;

// Counting rotations
volatile unsigned int state = 0;
unsigned int prev_state = 0; 

unsigned long ISRinterval; // Time between to interrupts in micro s
unsigned long prev_ISRinterval; 
bool newISR = false; // not used 
  
float rpm; 
// For some reason in tinkercad float value goes to 0 if it is below 1. 
// Most likely tinkercad issue


void setup()
{ 

  
  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(stndby, OUTPUT);
  
  pinMode(m1_A, INPUT);  // Tinkercad rotary encoder only work with capacitors
  pinMode(m1_B, INPUT);
  
  // Interrupt init. Parameters are: 
  // interrupt pin (not all pins can be interrupts), ISR function, mode
  attachInterrupt(digitalPinToInterrupt(m1_A), service_rout, CHANGE);
  
  Serial.begin(9600);
/*
  digitalWrite(stndby, HIGH);
  digitalWrite(m1_in1, HIGH);
  digitalWrite(m1_in2, LOW);
  analogWrite(m1_pwm, 125);
*/

}
void loop()
{


  
  if (state != state){ // If an interrupt has occured since last time
    
    // 690 comes from tinkercad ... apparently it is aprox. how many pulses there is pr. revolution
    // according to polulu, encoders should have 12 counts pr. revolution for both channels,
    // counting both sides (low -> high and high -> low)
    // meaning from 1 channel counting one side it should be 3 pr. revolution
     //rpm = ( (1 * 1000000 * 60) /ISRinterval) / 3;
    Serial.println(state);
  }

  
 // prev_ISRinterval = ISRinterval ;
    prev_state = state; 
    
  

/*

Potentially look into cli() and sei(), which turns off and on interrupts:
It seems they fuck up Serial.print on tinkercad ... maybe more things. 
More importantly, since cli() might fuck up Serial.print, it might mess whith
the arduino <-> rpi serial connection. (I'm assuming that must work as some sort of interrupt)

*/ 

}             

void service_rout()
{
//    current = millis();
      state ++;


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



