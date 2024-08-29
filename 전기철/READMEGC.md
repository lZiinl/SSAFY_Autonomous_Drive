## 학습 내용 정리
- src 폴더 내에 beginner_tutorials 폴더 배치 후 scripts 폴더 내에 실행시킬 파일 배치
- chmod +x 파일명.py로 실행권한 부여
- rosrun beginner_tutorials 파일명.py로 실행

- 센서 부착 후 ROS 연결시 rosbridge_websocket에서 
2024-08-29 17:37:53+0900 [-] [INFO] [1724920673.123288]: Client connected.  34 clients total.
위 로그와 같이 클라이언트 숫자가 늘어나는 것으로 연결을 확인할 수 있음.


## 편의 세팅
#### catkin_make  //  source devel/setup.bash 치기 귀찮을 때

1. terminal에 gedit ~/.bashrc

2. 맨 밑에
alias cm='catkin_make'
alias sd='source devel/setup.bash'
추가 후 저장

3. source ~/.bashrc
4. 안되면 터미널 다시 실행시 cm , sd로 catkin_make, source devel/setup.bash 사용가능