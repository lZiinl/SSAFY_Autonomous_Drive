#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

# camera_img는 시뮬레이터에서 송신하는 Camera 센서 정보를 Subscriber 하는 예제입니다.
# Camera 센서 정보인 /image_jpeg/compressed라는 메시지를 Subscribe 합니다.
# Subscribe 한 데이터를 OpenCV를 이용하여 Image로 출력합니다.

def Camera_callback(data):
    # ROS 메시지를 numpy 배열로 변환
    np_arr = np.frombuffer(data.data, np.uint8)
    
    # 이미지 디코딩
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    # 이미지 출력
    cv2.imshow("Image window", img_bgr)
    cv2.waitKey(1)

def listener():
    # ROS 노드 초기화
    rospy.init_node('camera_img', anonymous=True)

    # Subscriber 생성
    rospy.Subscriber('/image_jpeg/compressed', CompressedImage, Camera_callback)

    # 콜백 함수를 계속해서 실행
    rospy.spin()

if __name__ == '__main__':
    listener()
