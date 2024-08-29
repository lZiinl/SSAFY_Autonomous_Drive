
# Robot Operating System (ROS)

ROS�� �κ� ����Ʈ���� ������ ���� ���¼ҽ� �����ӿ�ũ��, ���(Node)��� �Ҹ��� ���μ����� �л� �����ӿ�ũ�� ����� ���� ���α׷��� ���������� �����ϰ�, ���� �� ���μ��� �� ���յ��� ���߾� ��ü �ý����� ������ �� �ֵ��� �մϴ�. ROS�� C++�� Python�� �����ϸ�, �پ��� ���̺귯���� ���� �����մϴ�.

- **�ֿ� ���**:
    - ���̴�, ī�޶� ���� ������ �ð�ȭ�ϴ� ���� ����
    - �޽��� ��� �� ��� ������� �ݺ����� ���� ����
    - �˰��� ���߿� ����

## ROS Tools

- **Rviz**: ���� ������ �ð�ȭ ����
- **RQT**: QT ��� GUI ���� ���� ����
- **Gazebo**: ���� ���� ����� 3���� �ùķ�����

## ROS Architecture

- **���긶����**: ���� �����ʹ� ��� ������ �ְ�޽��ϴ�.
- **������**: ��峢�� ���� ������ �ְ�޽��ϴ�.
- **������**: ��峢�� �޽��� ���(����, ����)�� �մϴ�.

## ROS ��� ����

- **ROS Master**:
    - ���� ��� ������ ����� ����� ���� ����
    - �����Ͱ� ������ ROS ��� �� �޽���, ���� ���� ����� �� �� �����ϴ�.
    - ���� ��ɾ�: `roscore`
- **ROS Node**:
    - ROS���� ����Ǵ� �ּ� ���� ���μ���(���α׷�)
    - �ϳ��� ������ �ϳ��� ��带 �����ϴ� ���� �����մϴ�.
- **ROS Message**:
    - ��� �� ������ ������ ���� ���
- **ROS Package**:
    - ROS ����Ʈ������ �⺻ ����
    - ��Ű���� ���, ���̺귯��, ȯ�漳�� ���� ���� �����ϴ� ���� �� ���� �����Դϴ�.
- **ROS Topic**:
    - �ܹ����� �������� �޽��� �ۼ��� ���
    - �޽����� �۽��ϱ� ���� �������� �����Ϳ� ����Ͽ� �޽����� �����ϴ�.
- **ROS Service**:
    - ������� ��ȸ�� �ۼ��� ���
- **ROS Publish**:
    - Topic�� �޽����� ��� �۽��ϴ� ��
- **ROS Publisher**:
    - Publish�� �����ϱ� ���� Topic�� ������ ������ �����Ϳ� ����ϰ�, Subscriber Node�� �޽����� �����ϴ�.
- **ROS Subscribe**:
    - Topic�� ������ �����ϴ� ��
- **ROS Subscriber**:
    - Subscribe�� �����ϱ� ���� Topic�� ������ ������ �����Ϳ� ����ϰ�, ������ Topic ������ �޾ƿɴϴ�.

| Topic          | Service         |
|----------------|-----------------|
| �ܹ���, �񵿱� | �����, ����     |
| Publisher : Message �۽� | Service Client : Service ��û |
| Subscriber : Message ���� | Service Server : Service ���� |
| �������� �ۼ��ſ� ���� | ��û �� �������� ���� |

### ROS ��Ű�� ����

- **ROS Workspace**
    - `src`: Catkin ��Ű���� �ҽ� �ڵ带 �����ϴ� ����
    - `build`: Catkin ��Ű���� �����ϱ� ���� Cmake�� ȣ��Ǵ� ����
    - `devel`: ��Ű���� �ý��ۿ� ��ġ�ϱ� ��, ���� ������ ���̴� ���� ���ϰ� ���̺귯���� ����Ǵ� ����
    - `src`
        - `CMakeList.txt`
        - `Package name`
            - `launch`: `.launch` ����
            - `scripts`: `.cpp` ���� (.py)
            - `include`
            - `CMakeList.txt`

## ROS �� ��ɾ�

- `roscd` (cd)
- `rosls` (ls)

## ROS ���� ��ɾ�

- `roscore`: ������ ��� ����
- `rosrun`: ��� ����
- `roslaunch`: ���� ��� ���� �� ���� �ɼ� ����
- `rosclean`: ROS log file �˻� �� ����

## ROS catkin ��ɾ�

- `catkin_create_pkg`: Catkin ���� �ý������� ��Ű�� �ڵ� ����
- `catkin_make`: Catkin ���� �ý��ۿ� ����� �� ����
- `catkin_init_workspace`: Catkin ���� �ý��� �۾� ���� �ʱ�ȭ

---

## ����

�������࿡ ���Ǵ� �ֿ� ������ ������ �����ϴ�:

### ī�޶�

- **����**: ǥ���� ����, ���� �б� ����
- **����**: ��Ȯ�� �Ÿ� ���� �����, �ܺ� ȯ�濡 ����

### ���̴� (LIDAR)

- **����**: ������ 3D �̹��� ����, ���� �ν� ����
- **����**: ���� ������ �Һ�, ��õ�Ŀ� ����

### ���̴� (Radar)

- **����**: ���� ���� ����, ����ȭ ����
- **����**: ���� ��ü �ĺ� �����, ������ �̹��� ���� �Ұ�

### GPS (Global Positioning System)

- **����**: ������ ����, ��Ȯ�� ��ġ ���� ����
- **����**: ������ GPS�� ��, �ܰ� ����

### IMU (Inertial Measurement Unit)

- **����**: 3�� ���ӵ� �� ȸ�� �ӵ� ���� ����
- **����**: ������ �� ������ ���

---

# MORAI �ùķ�����

## ��Ʈ��ũ ����

- **Ego Network**
    - Cmd Control
    - Publisher, Subscriber, Service
- **Simulator Network**
    - Publisher, Subscriber, Service

### ROSBridge ����

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
ifconfig

### ��Ʈ��ũ ����

1. ���ϴ� ��Ʈ��ũ Controller�� �����ϰ� Ubuntu(VM) �͹̳ο����� IP �ּҸ� �Է��� ��, **Connect** ��ư�� Ŭ���մϴ�.
2. �ùķ����� ���� ��, `F4`�� ���� ��Ʈ��ũ ���� ȭ���� �����մϴ�.

### Cmd Control

![Cmd Control](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/dd5e7755-1fdb-4bce-8103-cf6d2f70f4ca/image.png)

### Publisher, Subscriber, Service

- **Collision Data**�� `morai_msgs/CollisionData` �޽��� Ÿ���� �����ϴ�.
- ������ ���ȸ� `/CollisionData`�� ����մϴ�.

![Collision Data](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4bce-8103-cf6d2f70f4ca/image.png)

### Simulator Network

- **�޽��� Ÿ��**: `morai_msgs/VehicleCollisionData`
- **���� ��**: `/VehicleCollisionData`

![Simulator Network](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/9e435ed7-5c27-4d01-9fbc-3658f1a5e751/image.png)

---

## ���� ����

### ���� ��ġ

- **��ġ**: Edit �� Sensor �� Sensor Edit Mode
- **�۾�**: 
  - `Shift + �巡��`�� ������ ��ġ�� �� �ֽ��ϴ�.
  - `Delete` Ű�� ����� ������ ������ �� �ֽ��ϴ�.

### ���� �ɼ� (����)

- **Sensor Properties**���� ���� ���� �ɼ��� ������ �� �ֽ��ϴ�.
- **Transform**: ������ ��ġ �� ������ �����մϴ�.
- **Network Setting**: ���ϴ� ���� ��Ʈ��ũ�� �����ϰ�, IP �ּҸ� �Է��� �� **Connect** ��ư�� Ŭ���Ͽ� �����մϴ�.

![Sensor Properties](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/d6fae05e-707f-474f-92e3-05d5489bf823/image.png)

### ���� ���� - ī�޶�

![Camera Settings](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/57671166-20bc-4725-8fe8-9f5def0d73be/image.png)

### ���� ���� - ���̴� (LIDAR)

- ���̴� ������ Velodyne Driver�� ����� UDP�� ȣ��Ʈ IP �ּҿ� Ubuntu IP �ּҸ� �Է��Ͽ� ������ �����մϴ�.
- **Host Sensor IP**: ���� ������ �ּҸ� �ǹ��մϴ�.
- **Destination IP**: Velodyne Driver ������ �ּҸ� �ǹ��մϴ�.
- ROS�� ���� ���� Virtual Machine�� IP �ּҸ� �Է��մϴ�.

![Lidar Settings](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/2b7ffe99-a55b-4109-9c25-69136fec06e8/image.png)

#### Lidar Transform

- **Lidar Transform**: ���̴��� ��ǥ �� �ڼ��� �����մϴ�.
- **Lidar Model**: ���̴� �� �� ä���� �����մϴ�.
- **Intensity Type**: Intensity Ÿ���� ������ �� �ֽ��ϴ� (Intensity, Semantic, Instance ���� ����).

#### Gaussian Noise

- **Gaussian Noise** �ɼ��� Ȱ��ȭ�Ͽ� ����� ������ �� �ֽ��ϴ�.
- �������� ������ **Mean (m)**, **Stdev (%)** ���� �Է��Ͽ� ������ �� �ֽ��ϴ�.

#### Lidar Viz Point Cloud

- **Viz Point Cloud** �ɼ��� Ȱ��ȭ�Ͽ� �ùķ����Ϳ��� ���̴��� Point Cloud�� �ð�ȭ�� �� �ֽ��ϴ�.

![Lidar Point Cloud](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/7efb8d5f-072a-4ddd-bad1-71452f06f08c/image.png)

### ���� ���� - GPS

#### GPS �ɼ�

- **GPS ������ ���� �ֱ�**: 10Hz �Ǵ� 50Hz�� ������ �� �ֽ��ϴ�.
- **Info ��ư**�� Ŭ���Ͽ� �ùķ����� ������ GPS ���� ������ ���� Ȯ���� �� �ֽ��ϴ� (��Ʈ��ũ ���� ��).

#### Gaussian Noise

- **Gaussian Noise** �ɼ��� Ȱ��ȭ�Ͽ� ����� ������ �� �ֽ��ϴ�.
- �������� ������ **Mean (m)**, **Stdev (%)** ���� �Է��Ͽ� ������ �� �ֽ��ϴ�.

![GPS Settings](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/69053dec-591a-4549-a1d8-d2d53fd16707/image.png)

### ���� ���� - IMU

#### IMU �ɼ�

- **IMU ������ ���� �ֱ�**: 30Hz �Ǵ� 50Hz�� ������ �� �ֽ��ϴ�.
- **Info ��ư**�� Ŭ���Ͽ� �ùķ����� ������ IMU ���� ������ ���� Ȯ���� �� �ֽ��ϴ�.

#### IMU Noise

- **Noise Model** �ɼ��� Ȱ��ȭ�Ͽ� �پ��� ������ ���� Ȱ��ȭ�� �� �ֽ��ϴ�.
  - **Bias-Instability Noise**
  - **Acceleration (m/s��)**
  - **Gyroscope (rad/s)**
  - **White Gaussian Noise**
  - **Acceleration (m/s/��h)**
  - **Gyroscope (rad/��h)**
  - **Random Walk Noise**
  - **Acceleration (m/s/h^(3/2))**
  - **Gyroscope (rad/h^(3/2))**

---

## ��Ÿ ���� - List View

![List View](https://prod-files-secure.s3.us-west-2.amazonaws.com/bcd274b8-88f2-4da2-b976-27ae079909fb/a9306ab5-eebe-45cd-be38-f277e7400604/image.png)

### List View

- **Sensor List**: �ùķ����Ϳ� ��ġ�� �������� ����Ʈ ���·� Ȯ���� �� �ֽ��ϴ�.
- **Active Objects**: �ùķ����Ϳ� ��ġ�� ��� ������Ʈ�� ����Ʈ ���·� Ȯ���� �� �ֽ��ϴ�.
- **Network List**: �ùķ������� ��� ��Ʈ��ũ ���¸� ����Ʈ�� �����ݴϴ�. ����� ��Ʈ��ũ�� �ʷϻ����� ǥ�õ˴ϴ�.
- **Mini Map**: Ego ������ �������� �� �ùķ������� �̴� ���� �����ݴϴ�.
- **MGEO Map**: ���� Map�� MGEO ��ũ ����Ʈ�� �����ݴϴ�. **Line View** �ɼ��� Ȱ��ȭ�ϸ� �ùķ����Ϳ��� MGEO ��ũ�� �ð�ȭ�� �� �ֽ��ϴ�.