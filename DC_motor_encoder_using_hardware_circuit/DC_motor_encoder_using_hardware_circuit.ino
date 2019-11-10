//pcf8575 library: https://github.com/skywodd/pcf8574_arduino_library/tree/master/PCF8575
// Thanks to Brett Beauregard for his nice PID library http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

#include <PID_v1.h>  // Required to compute PID
#include <Wire.h>    // Required for I2C communication
#include "PCF8575.h" // Required for PCF8575
#define M1              9                       
#define M2              10

PCF8575 expander;

String inString = "";
double kp =140, ki =0 , kd =0.1;          // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
long temp;
volatile int encoderPos = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

void setup() {
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  expander.begin(0x20);
  Serial.begin (115200);                              // for debugging
  setpoint= 0.0;
}

void loop() {
   encoderPos= (expander.read());  // Read the encoder value
  
   input = (encoderPos*360.0)/2200.0 ;  // Calculate the current position of the joint.

   if (encoderPos > 2250)
       pwmOut(-255);
   else if (encoderPos < -2250)
       pwmOut(255);
   else
   {
      if (Serial.available() > 0) {
        int inChar = Serial.read();
        if (isDigit(inChar)) {
        // convert the incoming byte to a char and add it to the string:
        inString += (char)inChar;
      }
      // if you get a newline, print the string, then the string's value:
      if (inChar == '\n') {
        setpoint= (double)inString.toInt();

        // clear the string for new input:
        inString = "";
      }
      } 
      Serial.print("C: "); 
      Serial.println(input);                      // monitor motor position
      Serial.print("T "); 
      Serial.println((int)setpoint); 
  
      myPID.Compute();                                    // calculate new output
      //Serial.print("O "); 
      //Serial.println((int)output); 
      pwmOut(output);                                     // drive L298N H-Bridge module
 }   
 Serial.print("E "); 
 Serial.println(encoderPos); 
}

void pwmOut(int out) {                                // to H-Bridge board
  if (out > 0) {
    analogWrite(M2, out);                             // drive motor CW
    analogWrite(M1, 0);
  }
  else {
    analogWrite(M2, 0);
    analogWrite(M1, abs(out));                        // drive motor CCW
  }
}
