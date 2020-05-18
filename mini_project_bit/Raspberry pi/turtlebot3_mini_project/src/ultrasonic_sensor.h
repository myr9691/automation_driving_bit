#ifndef _ULTRASONIC_SENSOR_H_
#define _ULTRASONIC_SENSOR_H_

#define LOW 0
#define HIGH 1

#define echo 27	
#define trig 28

void ultrasonic_sensor_setup()
{
	pinMode(trig, OUTPUT);
	pinMode(echo, INPUT);
}	

int ultrasonic_sensor_start()
{
	digitalWrite(trig, LOW);
	digitalWrite(echo, LOW);
	delayMicroseconds(2); 
  	digitalWrite(trig, HIGH); 
  	delayMicroseconds(10); 
  	digitalWrite(trig, LOW);
	while(digitalRead(echo) == LOW);
	long startTime = micros();
	while(digitalRead(echo) == HIGH);
	long travelTime = micros() - startTime;
	return (int)(travelTime / 58);
}	
#endif
