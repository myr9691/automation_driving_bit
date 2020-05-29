// Example 52.4 PCF8591 ADC demo
// http://tronixstuff.com/tutorials Chapter 52
// John Boxall June 2013
#include "Wire.h"
#define PCF8591 (0x90 >> 1) // I2C bus address
byte value0, value1, value2, value3;
int Ain = 0;
float a0value;

void setup()
{
     Wire.begin();
     Serial.begin(9600);
}

void loop()
{
    for (int i = 0; i < 256; i++)
    {    
        Wire.beginTransmission(PCF8591);
        Wire.write(0x40);                   // sets the PCF8591 into a DA mode
        Wire.write(i);                      // sets the outputn
        Wire.endTransmission();
     
        Wire.beginTransmission(PCF8591);    // wake up PCF8591
        Wire.write(0x04);                   // control byte - read ADC0 then auto-increment
        Wire.endTransmission(); // end tranmission
        Wire.requestFrom(PCF8591, 5);
        value0=Wire.read();
        value0=Wire.read();
        value1=Wire.read();
        value2=Wire.read();
        value3=Wire.read();
        Serial.print(value0); Serial.print(" ");
        Serial.print(value1); Serial.print(" ");
        Serial.print(value2); Serial.print(" ");
        Serial.print(value3); Serial.print(" ");       
        Serial.println();
        delay(300);
    }
}
