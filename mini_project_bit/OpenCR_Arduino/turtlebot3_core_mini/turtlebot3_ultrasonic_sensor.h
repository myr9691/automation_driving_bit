#ifndef _TURTLEBOT3_ULTRASONIC_SENSOR_H_
#define _TURTLEBOT3_ULTRASONIC_SENSOR_H_

// #include <std_msgs/Int32.h> turtlebot3_core_config.h 파일에 선언되어 있음

#define trig 53                                         // OpenCR GPIO pin 6
#define echo 51                                         // OpenCR GPIO pin 4

std_msgs::Int32 distance_right;                         // 초음파 센서로 측정한 거리값 std_msgs 로 pub하기 위한 변수 선언
ros::Publisher pub_cr_ultrasonic("distance_right", &distance_right);

void ultrasonic_sensor_setup()
{
  pinMode(trig, OUTPUT);                                // 초음파 송신부 output
  pinMode(echo, INPUT);                                 // 초음파 수신부 input
}

void ultrasonic_sensor_start()
{
  digitalWrite(trig, LOW);                              // trig 처음 상태 LOW
  digitalWrite(echo, LOW);                              // echo 처음 상태 LOW
  delayMicroseconds(2);                                 // 2us delay
  digitalWrite(trig, HIGH);                             // 초음파 trig 신호 발생
  delayMicroseconds(10);                                // 10us 만큼 초음파 발생
  digitalWrite(trig, LOW); 

  unsigned long duration = pulseIn(echo, HIGH);         // echo핀이 high신호를 유지하고 있는 동안의 시간을 측정

  // 초음파의 속도는 초당 340미터를 이동하거나, 29마이크로초 당 1센치를 이동합니다.
  // 따라서, 초음파의 이동 거리 = duration(왕복에 걸린시간) / 29 / 2 입니다. 
  // distance_right std_msgs에 측정한 거리 대입
  distance_right.data = (int)(duration / 29.0 / 2.0); 
  
  pub_cr_ultrasonic.publish(&distance_right);           // 측정한 거리값 publish
}

#endif
