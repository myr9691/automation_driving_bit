## Git 사용법   
$ git clone https://github.com/karlema/bit_automation_driving/   
$ cd bit_automation_driving   
$ cd Image_processing   
$ 업로드하려는 파일을 복사해 가져오기   
$ git init   
$ git config --global user.name "이름" (최초 한번만)   
$ git config --global user.email "이메일" (최초 한번만)   
$ git remote add origin https://github.com/karlema/bit_automation_driving/   
$ git add .   
$ git status (잘 추가되었는지 확인)   
$ git commit -m "수정된 내용 간략히"   
$ git push origin master   
  -> push에서 오류가 난다면   
  -> git pull origin master --allow-unrelated-histories   
  -> 편집기 나오면 Ctrl + X로 나간 후   
  -> 다시 git push origin master   

#### 2020-05-18
***
###### 문예리
* ~~차선인식 Python -> C++~~
* ~~영상처리한 영상 ROS 송수신 확인~~
* ~~소스 코드 업로드~~
* ~~sd 카드 백업~~
* 차선인식 코드 + ROS center publish code
###### 이명상
* ~~Transfer Learning Test(TensorFlow SSD v2 / Darknet Yolov3)~~
* 신호등 인식 (OpenCV)
* ~~ROS OpenCV 테스트~~
###### 김주성
* ~~조도센서를 이용한 LED 자동제어 (ROS통합버젼)~~
* ~~Gstreamer를 이용한 해상도 조절~~
* Gstreamer를 이용한 압축
* ~~세트 만들기 도움~~
***
#### 2020-05-25
***
###### 문예리
* ~~차선인식 코드 + ROS center publish code~~
* ~~코스 영상 새로 촬영 -> 차선 인식 코드 수정~~
* turtlebot3 자율 주행 코드 실행
###### 김주성
* <h1> I2C 2개이상 디바이스 연결
* QT공부
* LED 납땜
