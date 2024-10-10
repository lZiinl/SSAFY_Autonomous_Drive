import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';

const VelocityDisplay = () => {
  const [velocityKmph, setVelocityKmph] = useState(null);
  const [rosConnected, setRosConnected] = useState(false); // 연결 상태를 추가

  useEffect(() => {
    // ROSBridge WebSocket 연결 설정
    const ros = new ROSLIB.Ros({
      url: 'ws://192.168.56.101:9090',  // ROSBridge WebSocket URL
    });

    // 연결 성공 시
    ros.on('connection', () => {
      console.log('Connected to websocket server.');
      setRosConnected(true);  // 연결 상태 업데이트
    });

    // 연결 실패 시
    ros.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
      setRosConnected(false);  // 연결 실패 시 연결 상태 false로 설정
    });

    // 연결 종료 시
    ros.on('close', () => {
      console.log('Connection to websocket server closed.');
      setRosConnected(false);  // 연결 상태 false로 설정
    });

    // /vehicle_speed_kmph 토픽 구독
    const vehicleSpeedTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/vehicle_speed_kmph',
      messageType: 'std_msgs/Float32',
    });

    // 토픽에서 메시지 수신 시 속도 업데이트
    vehicleSpeedTopic.subscribe((message) => {
      const roundedVelocity = Math.floor(message.data);  // 정수로 변환
      setVelocityKmph(roundedVelocity);  // 정수 값으로 저장
    });

    // 컴포넌트 언마운트 시 구독 해제 및 연결 종료
    return () => {
      vehicleSpeedTopic.unsubscribe();
      ros.close();
    };
  }, []);

  return (
    <div className="absolute top-5 right-5 w-[200px] p-5 bg-black shadow-lg rounded-lg z-50 text-center opacity-90">
      <h2 className="text-[#FF5733] text-xl font-bold">속도 (km/h)</h2>
      {rosConnected ? (
        velocityKmph !== null ? (
          <p className="text-[#28A745] text-lg">{velocityKmph} km/h</p>
        ) : (
          <p className="text-white text-lg">Loading...</p>
        )
      ) : (
        <p className="text-red-500 text-lg">Disconnected</p>
      )}
    </div>
  );
};

export default VelocityDisplay;