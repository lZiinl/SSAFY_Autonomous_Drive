
1. Ubuntu 20.04 버전에서 ROS MELODIC을 설치한다.
2. MORAI SIM 24.R2.0 버전을 설치한다.
3. MORAI SIM에 3개의 ROS launch에 대한 json 파일을 load한다.
4. Ubuntu에서 두 개의 터미널 창을 실행한 후 아래 명령어를 실행한다.

- roscore
- roslaunch rosbridge_server rosbridge_websocket.launch

5. ros.zip 파일을 압축해제 한 후 launch 파일 실행에 필요한 코드를 다운로드한다.
6. 1,2,3에 해당하는 json 파일을 MORAI SIM에서 세팅하고 각 번호에 맞는 launch 파일을 ubuntu에서 실행한다.
7. react 코드를 실행하고 로컬 환경에서 정상 동작하는지 확인한다.
