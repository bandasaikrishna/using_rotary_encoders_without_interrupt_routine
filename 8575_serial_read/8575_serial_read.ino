//pcf8575 library: https://github.com/skywodd/pcf8574_arduino_library/tree/master/PCF8575

#include <Wire.h>    // Required for I2C communication
#include "PCF8575.h" // Required for PCF8575

PCF8575 expander;
uint16_t value=0,preval=10;

void setup() {
  // put your setup code here, to run once:
  /* Setup serial for debug */
  Serial.begin(115200);
   
  /* Start I2C bus and PCF8575 instance */
  expander.begin(0x20);
}

void loop() {
  value= expander.read();
  value &= 0x00FF;
  if (value != preval)
  {
    Serial.println(value,DEC);
  }
  delay(100); 
}
