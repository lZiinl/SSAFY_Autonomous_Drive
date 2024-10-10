import React, { useEffect, useState } from 'react';
import io from 'socket.io-client';

const ReverseView = () => {
  const [image, setImage] = useState('');
  const [lastUpdateTime, setLastUpdateTime] = useState(Date.now());

  useEffect(() => {
    const frameInterval = 1000 / 15;  // 1초에 15프레임 = 66.67ms
    const socket = io('http://192.168.56.101:5124', { 
      forceNew: true, 
      transports: ['polling']  // 폴링 방식으로만 연결 강제
    });

    socket.on('connect', () => {
      console.log('Connected to Socket.IO server on port 5124');
    });

    socket.on('image', (data) => {
      const now = Date.now();
      console.log("Received image at: ", now);  // 수신 시간 로깅
      if (now - lastUpdateTime > frameInterval) {  // 1초에 15프레임으로 갱신
        setImage(`data:image/jpeg;base64,${data}`);
        setLastUpdateTime(now);
      }
    });

    socket.on('disconnect', () => {
      console.log('Disconnected from server');
    });

    socket.on('connect_error', (error) => {
      console.error('Connection Error:', error);
    });

    socket.on('reconnect_attempt', () => {
      console.log('Trying to reconnect...');
    });

    // 클린업 함수: 소켓 연결 해제
    return () => {
      socket.off('image');
      socket.disconnect();
    };
  }, []);  // 의존성 배열에서 lastUpdateTime을 제거

  return (
    <div>
      {image ? (
        <img 
          src={image} 
          alt="Around View" 
          style={{ width: '100%' }} 
          onError={(e) => console.error("Image loading error", e)}
        />
      ) : (
        <p>Loading...</p>
      )}
    </div>
  );
};

export default ReverseView;