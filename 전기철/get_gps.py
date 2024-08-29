#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from morai_msgs.msg import GPSMessage

# gps_data_listener는 시뮬레이터에서 송신하는 GPS 센서 정보를 Subscriber 하는 예제입니다.
# GPS 센서 정보인 /gps라는 메세지를 Subscribe 합니다.

# Callback 함수 생성 및 데이터 출력
def gps_callback(data):
    '''
    # 시뮬레이터의 GPS 데이터를 아래와 같은 형식으로 터미널 창에 출력한다.
    # GPS 센서의 위도 경도 고도, Offset 값을 확인할 수 있다.
    # Offset 값은 시뮬레이터 좌표계를 계산하는데 사용된다.
    '''
    os.system('clear')
    print("\n ------------------------------------ \n")
    rospy.loginfo("latitude: {}".format(data.latitude))
    rospy.loginfo("longitude: {}".format(data.longitude))
    rospy.loginfo("altitude: {}".format(data.altitude))
    rospy.loginfo("eastOffset: {}".format(data.eastOffset))
    rospy.loginfo("northOffset: {}".format(data.northOffset))

def listener():
    # ROS 노드 초기화
    rospy.init_node('gps_data_listener', anonymous=True)

    # Subscriber 생성 (토픽 이름과 메시지 타입 확인 필요)
    rospy.Subscriber('/gps', GPSMessage, gps_callback)

    # 콜백 함수를 계속해서 실행
    rospy.spin()

if __name__ == '__main__':
    listener()
