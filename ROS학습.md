
# Robot Operating System (ROS)

ROS는 로봇 소프트웨어 개발을 위한 오픈소스 프레임워크로, 노드(Node)라고 불리는 프로세스의 분산 프레임워크를 사용해 실행 프로그램을 독립적으로 설계하고, 실행 시 프로세스 간 결합도를 낮추어 전체 시스템을 구동할 수 있도록 합니다. ROS는 C++와 Python을 지원하며, 다양한 라이브러리와 툴을 제공합니다.

- **주요 기능**:
    - 라이다, 카메라 등의 센서를 시각화하는 도구 제공
    - 메시지 기록 및 재생 기능으로 반복적인 실험 가능
    - 알고리즘 개발에 용이

## ROS Tools

- **Rviz**: 센서 데이터 시각화 도구
- **RQT**: QT 기반 GUI 응용 개발 도구
- **Gazebo**: 물리 엔진 기반의 3차원 시뮬레이터

## ROS Architecture

- **노드↔마스터**: 노드와 마스터는 노드 정보를 주고받습니다.
- **노드↔노드**: 노드끼리 접속 정보를 주고받습니다.
- **노드↔노드**: 노드끼리 메시지 통신(토픽, 서비스)을 합니다.

## ROS 용어 정리

- **ROS Master**:
    - 노드와 노드 사이의 연결과 통신을 위한 서버
    - 마스터가 없으면 ROS 노드 간 메시지, 토픽 등의 통신을 할 수 없습니다.
    - 실행 명령어: `roscore`
- **ROS Node**:
    - ROS에서 실행되는 최소 단위 프로세스(프로그램)
    - 하나의 목적에 하나의 노드를 개발하는 것을 권장합니다.
- **ROS Message**:
    - 노드 간 데이터 전송을 위한 양식
- **ROS Package**:
    - ROS 소프트웨어의 기본 단위
    - 패키지는 노드, 라이브러리, 환경설정 파일 등을 포함하는 빌드 및 배포 단위입니다.
- **ROS Topic**:
    - 단방향의 연속적인 메시지 송수신 방식
    - 메시지를 송신하기 위해 토픽으로 마스터에 등록하여 메시지를 보냅니다.
- **ROS Service**:
    - 양방향의 일회성 송수신 방식
- **ROS Publish**:
    - Topic에 메시지를 담아 송신하는 것
- **ROS Publisher**:
    - Publish를 수행하기 위해 Topic을 포함한 정보를 마스터에 등록하고, Subscriber Node에 메시지를 보냅니다.
- **ROS Subscribe**:
    - Topic의 내용을 수신하는 것
- **ROS Subscriber**:
    - Subscribe를 수행하기 위해 Topic을 포함한 정보를 마스터에 등록하고, 수신할 Topic 정보를 받아옵니다.

| Topic          | Service         |
|----------------|-----------------|
| 단방향, 비동기 | 양방향, 동기     |
| Publisher : Message 송신 | Service Client : Service 요청 |
| Subscriber : Message 수신 | Service Server : Service 응답 |
| 지속적인 송수신에 적합 | 요청 후 서버에서 응답 |

### ROS 패키지 구조

- **ROS Workspace**
    - `src`: Catkin 패키지의 소스 코드를 포함하는 공간
    - `build`: Catkin 패키지를 빌드하기 위해 Cmake가 호출되는 공간
    - `devel`: 패키지를 시스템에 설치하기 전, 개발 과정에 쓰이는 실행 파일과 라이브러리가 저장되는 공간
    - `src`
        - `CMakeList.txt`
        - `Package name`
            - `launch`: `.launch` 파일
            - `scripts`: `.cpp` 파일 (.py)
            - `include`
            - `CMakeList.txt`

## ROS 쉘 명령어

- `roscd` (cd)
- `rosls` (ls)

## ROS 실행 명령어

- `roscore`: 마스터 노드 실행
- `rosrun`: 노드 실행
- `roslaunch`: 여러 노드 실행 및 실행 옵션 설정
- `rosclean`: ROS log file 검사 및 삭제

## ROS catkin 명령어

- `catkin_create_pkg`: Catkin 빌드 시스템으로 패키지 자동 생성
- `catkin_make`: Catkin 빌드 시스템에 기반을 둔 빌드
- `catkin_init_workspace`: Catkin 빌드 시스템 작업 폴더 초기화

---

## 센서

자율주행에 사용되는 주요 센서는 다음과 같습니다:

### 카메라

- **장점**: 표지판 내용, 문자 읽기 가능
- **단점**: 정확한 거리 측정 어려움, 외부 환경에 영향

### 라이다 (LIDAR)

- **장점**: 정밀한 3D 이미지 제공, 형태 인식 가능
- **단점**: 높은 에너지 소비, 악천후에 약함

### 레이더 (Radar)

- **장점**: 날씨 영향 적음, 소형화 가능
- **단점**: 작은 물체 식별 어려움, 정밀한 이미지 제공 불가

### GPS (Global Positioning System)

- **장점**: 저렴한 가격, 정확한 위치 정보 제공
- **단점**: 고정밀 GPS는 고가, 단가 문제

### IMU (Inertial Measurement Unit)

- **장점**: 3축 가속도 및 회전 속도 측정 가능
- **단점**: 노이즈 및 진동에 취약

---

# MORAI 시뮬레이터

## 네트워크 설정

- **Ego Network**
    - Cmd Control
    - Publisher, Subscriber, Service
- **Simulator Network**
    - Publisher, Subscriber, Service

### ROSBridge 실행

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
ifconfig

### 네트워크 연결

1. 원하는 네트워크 Controller를 선택하고 Ubuntu(VM) 터미널에서의 IP 주소를 입력한 뒤, **Connect** 버튼을 클릭합니다.
2. 시뮬레이터 실행 후, `F4`를 눌러 네트워크 세팅 화면을 실행합니다.

### Cmd Control

![Cmd Control](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/dd5e7755-1fdb-4bce-8103-cf6d2f70f4ca/image.png)

### Publisher, Subscriber, Service

- **Collision Data**는 `morai_msgs/CollisionData` 메시지 타입을 가집니다.
- 동일한 토픽명 `/CollisionData`를 사용합니다.

![Collision Data](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4bce-8103-cf6d2f70f4ca/image.png)

### Simulator Network

- **메시지 타입**: `morai_msgs/VehicleCollisionData`
- **토픽 명**: `/VehicleCollisionData`

![Simulator Network](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/9e435ed7-5c27-4d01-9fbc-3658f1a5e751/image.png)

---

## 센서 설정

### 센서 배치

- **위치**: Edit → Sensor → Sensor Edit Mode
- **작업**: 
  - `Shift + 드래그`로 센서를 배치할 수 있습니다.
  - `Delete` 키를 사용해 센서를 삭제할 수 있습니다.

### 센서 옵션 (공통)

- **Sensor Properties**에서 센서 관련 옵션을 변경할 수 있습니다.
- **Transform**: 센서의 위치 및 각도를 지정합니다.
- **Network Setting**: 원하는 센서 네트워크를 선택하고, IP 주소를 입력한 뒤 **Connect** 버튼을 클릭하여 적용합니다.

![Sensor Properties](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/d6fae05e-707f-474f-92e3-05d5489bf823/image.png)

### 센서 설정 - 카메라

![Camera Settings](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/57671166-20bc-4725-8fe8-9f5def0d73be/image.png)

### 센서 설정 - 라이다 (LIDAR)

- 라이다 센서는 Velodyne Driver를 사용해 UDP로 호스트 IP 주소와 Ubuntu IP 주소를 입력하여 토픽을 전송합니다.
- **Host Sensor IP**: 센서 기준의 주소를 의미합니다.
- **Destination IP**: Velodyne Driver 기준의 주소를 의미합니다.
- ROS가 실행 중인 Virtual Machine의 IP 주소를 입력합니다.

![Lidar Settings](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/2b7ffe99-a55b-4109-9c25-69136fec06e8/image.png)

#### Lidar Transform

- **Lidar Transform**: 라이다의 좌표 및 자세를 설정합니다.
- **Lidar Model**: 라이다 모델 및 채널을 설정합니다.
- **Intensity Type**: Intensity 타입을 설정할 수 있습니다 (Intensity, Semantic, Instance 설정 가능).

#### Gaussian Noise

- **Gaussian Noise** 옵션을 활성화하여 노이즈를 생성할 수 있습니다.
- 노이즈의 정도는 **Mean (m)**, **Stdev (%)** 값을 입력하여 조절할 수 있습니다.

#### Lidar Viz Point Cloud

- **Viz Point Cloud** 옵션을 활성화하여 시뮬레이터에서 라이다의 Point Cloud를 시각화할 수 있습니다.

![Lidar Point Cloud](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/7efb8d5f-072a-4ddd-bad1-71452f06f08c/image.png)

### 센서 설정 - GPS

#### GPS 옵션

- **GPS 데이터 전송 주기**: 10Hz 또는 50Hz로 설정할 수 있습니다.
- **Info 버튼**을 클릭하여 시뮬레이터 내에서 GPS 센서 데이터 값을 확인할 수 있습니다 (네트워크 연결 시).

#### Gaussian Noise

- **Gaussian Noise** 옵션을 활성화하여 노이즈를 생성할 수 있습니다.
- 노이즈의 정도는 **Mean (m)**, **Stdev (%)** 값을 입력하여 조절할 수 있습니다.

![GPS Settings](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/69053dec-591a-4549-a1d8-d2d53fd16707/image.png)

### 센서 설정 - IMU

#### IMU 옵션

- **IMU 데이터 전송 주기**: 30Hz 또는 50Hz로 설정할 수 있습니다.
- **Info 버튼**을 클릭하여 시뮬레이터 내에서 IMU 센서 데이터 값을 확인할 수 있습니다.

#### IMU Noise

- **Noise Model** 옵션을 활성화하여 다양한 노이즈 모델을 활성화할 수 있습니다.
  - **Bias-Instability Noise**
  - **Acceleration (m/s²)**
  - **Gyroscope (rad/s)**
  - **White Gaussian Noise**
  - **Acceleration (m/s/√h)**
  - **Gyroscope (rad/√h)**
  - **Random Walk Noise**
  - **Acceleration (m/s/h^(3/2))**
  - **Gyroscope (rad/h^(3/2))**

---

## 기타 사용법 - List View

![List View](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/a9306ab5-eebe-45cd-be38-f277e7400604/image.png)

### List View

- **Sensor List**: 시뮬레이터에 배치된 센서들을 리스트 형태로 확인할 수 있습니다.
- **Active Objects**: 시뮬레이터에 배치된 모든 오브젝트를 리스트 형태로 확인할 수 있습니다.
- **Network List**: 시뮬레이터의 모든 네트워크 상태를 리스트로 보여줍니다. 연결된 네트워크는 초록색으로 표시됩니다.
- **Mini Map**: Ego 차량을 기준으로 한 시뮬레이터의 미니 맵을 보여줍니다.
- **MGEO Map**: 현재 Map의 MGEO 링크 리스트를 보여줍니다. **Line View** 옵션을 활성화하면 시뮬레이터에서 MGEO 링크를 시각화할 수 있습니다.