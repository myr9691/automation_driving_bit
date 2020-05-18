#ifndef _TURTLEBOT3_BUTTON_SENSOR_H_
#define _TURTLEBOT3_BUTTON_SENSOR_H_

// #include <std_msgs/Int32.h> turtlebot3_core_config.h 파일에 선언되어 있음

#define button 67                                // OpenCR GPIO pin 20

std_msgs::Int32 button_state;                    // 버튼 센서의 상태값 std_msgs 로 pub하기 위한 변수 선언
ros::Publisher pub_cr_button("speed_button", &button_state);

void button_sensor_setup()
{
  pinMode(button, INPUT);                       // 버튼 핀 모드 input
}

void button_sensor_start()
{
  button_state.data = digitalRead(button);      // 버튼 센서값 읽고 저장 
  
  pub_cr_button.publish(&button_state);         // 버튼 센서값 publish
}

#endif
