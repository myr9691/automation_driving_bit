#ifndef _ULTRASONIC_SENSOR_H_
#define _ULTRASONIC_SENSOR_H_

#define LOW 0
#define HIGH 1

#define echo 5 
#define trig 4

void back_laser_sensor_setup()
{
	pinMode(trig, OUTPUT);
	pinMode(echo, INPUT);
}

int back_laser_sensor_start()
{
	digitalWrite(trig, LOW);
	digitalWrite(echo, LOW);
	delayMicroseconds(2);
	digitalWrite(trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig, LOW);
	while(digitalRead(echo) == LOW);
	long startTime = micros();
	printf("%ld", startTime);
	while(digitalRead(echo) == HIGH);
	long travelTime = micros() - startTime;
	printf("%ld", travelTime);
	return (int)(travelTime/58);
}
#endif
