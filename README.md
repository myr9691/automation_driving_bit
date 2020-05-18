## Project Configuration
<img src="https://user-images.githubusercontent.com/49478077/81807607-5b067800-9559-11ea-8612-df083484dfb9.PNG" width="80%">


* * *
ROS melodic   
Ubuntu 18.04 LTS (PC)   
OpenCV 4.2.0   
* * *


#### 1) Raspberry pi (C++)
* OpenCV로 차선 인식 후 자율 주행 (진행 중)
* VL53L0X(레이저 거리 측정 센서)를 이용한 주차 알고리즘 (or Bezier Curve와 ROS odometry값으로 turtlebot제어)
* Pi CAM v2. 실시간 영상 처리 및 Jetson nano에 영상 전달 (진행 중)
* Publish : /cmd_vel

#### 2) Jetson nano (Python)
* Tensorflow Lite 모델 학습으로 객체 인식(신호등, 표지판, 사람, 차) (진행 중)
* CORAL을 붙여서 frame 및 GPU 성능 향상

#### 3) OpenCR Board (Arduino)
* motor 연결
* Subscribe : /cmd_vel
* Publish : /imu, /odom, /tf

#### 4) PC
* roscore 실행
* QT 프로그램으로 센서값, 영상 처리 화면, 객체 인식 화면 등 화면 구성 (진행 중)


* * *
