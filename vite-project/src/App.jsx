import React, { useState } from 'react';
import CameraFeed from './CameraFeed';
import './App.css';

const App = () => {
  const [selectedContent, setSelectedContent] = useState('camera');

  const handleIconClick = (content) => {
    setSelectedContent(content);
  };

  return (
    <div className="app-container">
      <div className="navigation-screen">
        <div className="top-section">
          <div className="left-panel">
            {selectedContent === 'camera' && <CameraFeed />}
          </div>
          <div className="right-panel">
            {selectedContent === 'music' && (
              <div className="music-content">
                <h2>유튜브 뮤직</h2>
                <iframe
                  width="100%"
                  height="400px"
                  src="https://open.spotify.com/embed/playlist/37i9dQZF1DXcBWIGoYBM5M/"
                  title="YouTube Music"
                  frameBorder="0"
                  allow="autoplay; encrypted-media"
                  allowFullScreen
                ></iframe>
              </div>
            )}
            {selectedContent === 'car' && <h2>차량 상태 화면</h2>}
            {selectedContent === 'ac' && <h2>에어컨 조절 화면</h2>}
            {selectedContent === 'sound' && <h2>음향 설정 화면</h2>}
            {selectedContent === 'phone' && <h2>전화 기능 화면</h2>}
          </div>
        </div>

        <div className="bottom-section">
          <div className="control-icon" onClick={() => handleIconClick('camera')}>
            📷
          </div>
          <div className="control-icon" onClick={() => handleIconClick('car')}>
            🚗
          </div>
          <div className="control-icon" onClick={() => handleIconClick('ac')}>
            ❄️
          </div>
          <div className="control-icon" onClick={() => handleIconClick('sound')}>
            🔊
          </div>
          <div className="control-icon" onClick={() => handleIconClick('phone')}>
            📱
          </div>
          <div className="control-icon" onClick={() => handleIconClick('music')}>
            🎵
          </div>
        </div>
      </div>
    </div>
  );
};

export default App;
// import React, { useRef, useEffect, useState } from 'react';
// import ROSLIB from 'roslib';
// import { Canvas, useFrame } from '@react-three/fiber';
// import { Box } from '@react-three/drei';

// // 회전하는 박스 컴포넌트 (NPC와 보행자 모두 사용)
// const RotatingBox = ({ position, size, color }) => {
//   const boxRef = useRef();

//   // 박스 회전 애니메이션
//   useFrame(() => {
//     if (boxRef.current) {
//       boxRef.current.rotation.x += 0.01;
//       boxRef.current.rotation.y += 0.01;
//     }
//   });

//   return (
//     <mesh ref={boxRef} position={position}>
//       {/* 박스의 크기와 색상 설정 */}
//       <boxGeometry args={size} />
//       <meshStandardMaterial color={color} />
//     </mesh>
//   );
// };

// const CameraFeed = () => {
//   const [egoPosition, setEgoPosition] = useState({ x: 0, y: 0, z: 0 });
//   const [objects, setObjects] = useState({ npc_list: [], pedestrian_list: [] });

//   useEffect(() => {
//     // WebSocket 연결 설정
//     const ros = new ROSLIB.Ros({
//       url: 'ws://192.168.56.101:9090',  // WebSocket 서버 주소 확인 필요
//     });

//     ros.on('connection', () => {
//       console.log('Connected to ROS WebSocket server.');
//     });

//     ros.on('error', (error) => {
//       console.error('Error connecting to ROS WebSocket server:', error);
//     });

//     ros.on('close', () => {
//       console.log('Connection to ROS WebSocket server closed.');
//       setTimeout(() => {
//         ros.connect('ws://192.168.56.101:9090');  // 연결 끊어졌을 때 다시 시도
//       }, 3000);
//     });

//     // Ego 차량의 상태 구독 (EgoVehicleStatus)
//     const egoStatusTopic = new ROSLIB.Topic({
//       ros: ros,
//       name: '/Ego_topic',
//       messageType: 'morai_msgs/EgoVehicleStatus',
//     });

//     egoStatusTopic.subscribe((message) => {
//       // Ego 차량의 위치 정보 저장
//       setEgoPosition({
//         x: message.position.x,
//         y: message.position.y,
//         z: message.position.z,
//       });

//       console.log(`Ego Position: x=${message.position.x}, y=${message.position.y}, z=${message.position.z}`);
//     });

//     // Object 데이터 구독 (NPC 차량 및 보행자)
//     const objectTopic = new ROSLIB.Topic({
//       ros: ros,
//       name: '/Object_topic',
//       messageType: 'morai_msgs/ObjectStatusList',
//     });

//     objectTopic.subscribe((message) => {
//       const scaleFactor = 0.1;  // 좌표 축소 스케일링 적용

//       // NPC 차량의 상대 좌표 계산
//       const detectedNPCs = message.npc_list.map((npc) => ({
//         x: (npc.position.x - egoPosition.x) * scaleFactor,
//         y: (npc.position.y - egoPosition.y) * scaleFactor,
//         z: (npc.position.z - egoPosition.z) * scaleFactor,
//         width: npc.size.x || 3,
//         height: npc.size.y || 3,
//         depth: npc.size.z || 3,
//       }));

//       // 보행자의 상대 좌표 계산
//       const detectedPedestrians = message.pedestrian_list.map((pedestrian) => ({
//         x: (pedestrian.position.x - egoPosition.x) * scaleFactor,
//         y: (pedestrian.position.y - egoPosition.y) * scaleFactor,
//         z: (pedestrian.position.z - egoPosition.z) * scaleFactor,
//         width: pedestrian.size.x || 1,
//         height: pedestrian.size.y || 2,
//         depth: pedestrian.size.z || 1,
//       }));

//       // 상대 좌표로 변환된 객체 목록 업데이트
//       setObjects({ npc_list: detectedNPCs, pedestrian_list: detectedPedestrians });
//     });

//     return () => {
//       egoStatusTopic.unsubscribe();
//       objectTopic.unsubscribe();
//       ros.close();
//     };
//   }, [egoPosition]);

//   return (
//     <div className="camera-feed" style={{ position: 'relative', width: '100%', height: '100%' }}>
//       <Canvas camera={{ position: [0, 10, 20], fov: 50 }}>
//         <ambientLight intensity={0.5} />
//         <pointLight position={[10, 10, 10]} />

//         {/* NPC 차량 렌더링 */}
//         {objects.npc_list.map((npc, index) => (
//           <RotatingBox
//             key={`npc-${index}`}
//             position={[npc.x, npc.y, npc.z]}  // 상대 좌표로 렌더링
//             size={[npc.width, npc.height, npc.depth]}  // 크기 설정
//             color="red"  // NPC 차량은 빨간색으로 렌더링
//           />
//         ))}

//         {/* 보행자 렌더링 */}
//         {objects.pedestrian_list.map((pedestrian, index) => (
//           <RotatingBox
//             key={`pedestrian-${index}`}
//             position={[pedestrian.x, pedestrian.y, pedestrian.z]}  // 상대 좌표로 렌더링
//             size={[pedestrian.width, pedestrian.height, pedestrian.depth]}  // 크기 설정
//             color="blue"  // 보행자는 파란색으로 렌더링
//           />
//         ))}
//       </Canvas>
//     </div>
//   );
// };

// export default CameraFeed;
