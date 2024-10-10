import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';

const CameraFeed = () => {
  const [imageSrc, setImageSrc] = useState(''); // 카메라 이미지 데이터
  const canvasRef = useRef(null); // 캔버스 참조

  useEffect(() => {
    // WebSocket 연결 설정
    const ros = new ROSLIB.Ros({
      url: 'ws://192.168.56.101:9090',  // WebSocket 서버 주소 확인 필요
    });

    ros.on('connection', () => {
      console.log('Connected to ROS WebSocket server.');
    });

    ros.on('error', (error) => {
      console.error('Error connecting to ROS WebSocket server:', error);
    });

    ros.on('close', () => {
      console.log('Connection to ROS WebSocket server closed.');
      setTimeout(() => {
        ros.connect('ws://192.168.56.101:9090');  // 연결 끊어졌을 때 다시 시도
      }, 3000);
    });

    // 카메라 이미지 데이터 구독
    const imageTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/camera6/image_jpeg/compressed',
      messageType: 'sensor_msgs/CompressedImage',
    });

    imageTopic.subscribe((message) => {
      console.log('Camera image received.');
      const imageBase64 = `data:image/jpeg;base64,${message.data}`;
      setImageSrc(imageBase64);

      const img = new Image();
      img.src = imageBase64;
      img.onload = () => {
        drawImageToCanvas(img);  // 이미지 그리기
      };
    });

    return () => {
      imageTopic.unsubscribe();
      ros.close();
    };
  }, []);

  // 이미지 캔버스에 그리기 함수 (비율 유지 및 중앙 크롭)
  const drawImageToCanvas = (img) => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');

    // 캔버스와 이미지 비율 계산
    const imgAspectRatio = img.width / img.height;
    const canvasAspectRatio = canvas.width / canvas.height;

    let drawWidth, drawHeight, offsetX, offsetY;

    if (imgAspectRatio > canvasAspectRatio) {
      // 이미지가 더 넓은 경우: 세로에 맞추고 좌우 크롭
      drawHeight = canvas.height;
      drawWidth = img.width * (canvas.height / img.height);
      offsetX = -(drawWidth - canvas.width) / 2; // 좌우 크롭
      offsetY = 0; // 상하 크롭 없음
    } else {
      // 이미지가 더 좁은 경우: 가로에 맞추고 위아래 크롭
      drawWidth = canvas.width;
      drawHeight = img.height * (canvas.width / img.width);
      offsetX = 0; // 좌우 크롭 없음
      offsetY = -(drawHeight - canvas.height) / 2; // 상하 크롭
    }

    // 이전 캔버스 지우기 및 이미지 그리기
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(img, offsetX, offsetY, drawWidth, drawHeight);
  };

  return (
    <div className="camera-feed" style={{ position: 'relative', width: '100%', height: '100%' }}>
      {/* 카메라 이미지 캔버스 */}
      <canvas ref={canvasRef} width="730" height="980"></canvas>
    </div>
  );
};

export default CameraFeed;
