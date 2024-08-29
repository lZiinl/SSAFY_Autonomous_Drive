#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from sensor_msgs.msg import Imu

# imu_data_listener는 시뮬레이터에서 송신하는 IMU 센서 정보를 Subscriber 하는 예제입니다.
# IMU 센서 정보인 /imu라는 메시지를 Subscribe 합니다.

def imu_callback(data):
    '''
    시뮬레이터의 IMU 데이터를 아래와 같은 형식으로 터미널 창에 출력합니다.
    '''
    os.system('clear')
    rospy.loginfo('------------------ IMU Sensor Status ------------------')
    
    # 방향(Orientation) 출력
    rospy.loginfo("orientation:")
    rospy.loginfo("x : {} y : {} z : {} w : {}".format(
        data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))

    # 각속도(Angular Velocity) 출력
    rospy.loginfo("angular_velocity:")
    rospy.loginfo("x : {} y : {} z : {}".format(
        data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))

    # 선형 가속도(Linear Acceleration) 출력
    rospy.loginfo("linear_acceleration:")
    rospy.loginfo("x : {} y : {} z : {}".format(
        data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))

def listener():
    # ROS 노드 초기화
    rospy.init_node('imu_data_listener', anonymous=True)

    # Subscriber 생성 (토픽 이름과 메시지 타입 확인 필요)
    rospy.Subscriber('/imu', Imu, imu_callback)

    # 콜백 함수를 계속해서 실행
    rospy.spin()

if __name__ == '__main__':
    listener()
