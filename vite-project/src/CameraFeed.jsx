// import React, { useEffect, useRef, useState } from 'react';
// import ROSLIB from 'roslib';
// import { Canvas } from '@react-three/fiber';
// import { Box } from '@react-three/drei';

// const CameraFeed = () => {
//   const [imageSrc, setImageSrc] = useState('');
//   const [objects, setObjects] = useState({ npc_list: [], pedestrian_list: [] });
//   const [egoPosition, setEgoPosition] = useState({ x: 0, y: 0, z: 0 });
//   const canvasRef = useRef(null);

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

//     // 카메라 이미지 데이터 구독
//     const imageTopic = new ROSLIB.Topic({
//       ros: ros,
//       name: '/camera1/image_jpeg/compressed',
//       messageType: 'sensor_msgs/CompressedImage',
//     });

//     imageTopic.subscribe((message) => {
//       console.log('Camera image received.');
//       const imageBase64 = `data:image/jpeg;base64,${message.data}`;
//       setImageSrc(imageBase64);
      
//       const img = new Image();
//       img.src = imageBase64;
//       img.onload = () => {
//         processImage(img);  // 이미지 처리
//       };
//     });

//     // Object 데이터 구독 (NPC 차량 및 보행자)
//     const objectTopic = new ROSLIB.Topic({
//       ros: ros,
//       name: '/Object_topic',
//       messageType: 'morai_msgs/ObjectStatusList',
//     });

//     objectTopic.subscribe((message) => {
//       console.log('Object data received.');
//       const scaleFactor = 0.01;  // 좌표 축소 스케일링 적용
//       const detectedNPCs = message.npc_list.map((npc) => ({
//         type: 'Car',
//         x: (npc.position.x - egoPosition.x) * scaleFactor,
//         y: (npc.position.y - egoPosition.y) * scaleFactor,
//         z: (npc.position.z - egoPosition.z) * scaleFactor,
//         width: npc.size.x || 1,
//         height: npc.size.y || 1,
//         depth: npc.size.z || 1,
//       }));

//       const detectedPedestrians = message.pedestrian_list.map((pedestrian) => ({
//         type: 'Person',
//         x: (pedestrian.position.x - egoPosition.x) * scaleFactor,
//         y: (pedestrian.position.y - egoPosition.y) * scaleFactor,
//         z: (pedestrian.position.z - egoPosition.z) * scaleFactor,
//         width: pedestrian.size.x || 0.5,
//         height: pedestrian.size.y || 1.8,
//         depth: pedestrian.size.z || 0.5,
//       }));

//       setObjects({ npc_list: detectedNPCs, pedestrian_list: detectedPedestrians });
//     });

//     return () => {
//       imageTopic.unsubscribe();
//       objectTopic.unsubscribe();
//       ros.close();
//     };
//   }, [egoPosition]);

//   // Canny Edge 처리 함수
//   const processImage = (img) => {
//     const canvas = canvasRef.current;
//     const ctx = canvas.getContext('2d');
//     ctx.drawImage(img, 0, 0, canvas.width, canvas.height);

//     if (!window.cv || !window.cv.imread) {
//       console.error('OpenCV is not ready yet.');
//       return;
//     }

//     const src = window.cv.imread(canvas);
//     const gray = new window.cv.Mat();
//     const edges = new window.cv.Mat();
    
//     window.cv.cvtColor(src, gray, window.cv.COLOR_RGBA2GRAY, 0);
//     window.cv.Canny(gray, edges, 50, 150, 3, false);

//     const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
//     for (let i = 0; i < imageData.data.length; i += 4) {
//       const edgeValue = edges.data[i / 4];
//       imageData.data[i] = edgeValue;
//       imageData.data[i + 1] = edgeValue;
//       imageData.data[i + 2] = edgeValue;
//     }
//     ctx.putImageData(imageData, 0, 0);

//     src.delete();
//     gray.delete();
//     edges.delete();
//   };

//   return (
//     <div className="camera-feed" style={{ position: 'relative', width: '100%', height: '100%' }}>
//       {/* Canny Edge 캔버스 */}
//       <canvas ref={canvasRef} width="1920" height="960"></canvas>

//       {/* Three.js를 사용한 3D 객체 렌더링 */}
//       <Canvas className="threeDCanvas" camera={{ position: [0, 10, 30], fov: 50 }}>
//         <ambientLight intensity={0.5} />
//         <pointLight position={[10, 10, 10]} />
        
//         {/* NPC 차량 렌더링 */}
//         {objects.npc_list.map((obj, index) => (
//           <Box
//             key={`npc-${index}`}
//             position={[obj.x, obj.y, obj.z]}
//             args={[obj.width, obj.height, obj.depth]}
//             material={{ color: 'red' }}
//           />
//         ))}

//         {/* 보행자 렌더링 */}
//         {objects.pedestrian_list.map((obj, index) => (
//           <Box
//             key={`pedestrian-${index}`}
//             position={[obj.x, obj.y, obj.z]}
//             args={[obj.width, obj.height, obj.depth]}
//             material={{ color: 'blue' }}
//           />
//         ))}
//       </Canvas>
//     </div>
//   );
// };

// export default CameraFeed;

import React, { useRef, useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { Canvas, useFrame } from '@react-three/fiber';
import { Box } from '@react-three/drei';
import './CameraFeed.css';  // CameraFeed.css 파일 임포트

const RotatingBox = ({ position, size, color }) => {
  const boxRef = useRef();

  useFrame(() => {
    if (boxRef.current) {
      boxRef.current.rotation.x += 0.01;
      boxRef.current.rotation.y += 0.01;
    }
  });

  return (
    <mesh ref={boxRef} position={position}>
      <boxGeometry args={size} />
      <meshStandardMaterial color={color} />
    </mesh>
  );
};

const CameraFeed = () => {
  const [imageSrc, setImageSrc] = useState('');
  const [objects, setObjects] = useState({ npc_list: [], pedestrian_list: [] });
  const [egoPosition, setEgoPosition] = useState({ x: 0, y: 0, z: 0 });
  const canvasRef = useRef(null);
  const ros = useRef(null); // ros 객체를 유지하기 위해 useRef 사용

  useEffect(() => {
    // ROS 연결 설정 (한 번만 연결)
    ros.current = new ROSLIB.Ros({
      url: 'ws://192.168.56.101:9090',
    });

    ros.current.on('connection', () => {
      console.log('Connected to ROS WebSocket server.');
    });

    ros.current.on('error', (error) => {
      console.error('Error connecting to ROS WebSocket server:', error);
    });

    ros.current.on('close', () => {
      console.log('Connection to ROS WebSocket server closed.');
      setTimeout(() => {
        ros.current.connect('ws://192.168.56.101:9090');
      }, 3000);
    });

    // 카메라 이미지, Ego 차량 상태, Object 데이터 구독 설정
    const imageTopic = new ROSLIB.Topic({
      ros: ros.current,
      name: '/camera1/image_jpeg/compressed',
      messageType: 'sensor_msgs/CompressedImage',
    });

    const egoStatusTopic = new ROSLIB.Topic({
      ros: ros.current,
      name: '/Ego_topic',
      messageType: 'morai_msgs/EgoVehicleStatus',
    });

    const objectTopic = new ROSLIB.Topic({
      ros: ros.current,
      name: '/Object_topic',
      messageType: 'morai_msgs/ObjectStatusList',
    });

    // 데이터 수신 설정
    const startDataSubscriptions = () => {
      // 1초마다 카메라 이미지 수신
      const cameraInterval = setInterval(() => {
        imageTopic.subscribe((message) => {
          const imageBase64 = `data:image/jpeg;base64,${message.data}`;
          setImageSrc(imageBase64);

          const img = new Image();
          img.src = imageBase64;
          img.onload = () => {
            processImage(img); // Canny Edge 처리
          };
        });
      }, 1000); // 1초마다 실행

      // 1초마다 Ego 차량의 상태 수신
      const egoInterval = setInterval(() => {
        egoStatusTopic.subscribe((message) => {
          setEgoPosition({
            x: message.position.x,
            y: message.position.y,
            z: message.position.z,
          });
          console.log(`Ego Position: x=${message.position.x}, y=${message.position.y}, z=${message.position.z}`);
        });
      }, 1000); // 1초마다 실행

      // 1초마다 Object 데이터 수신 (NPC 차량 및 보행자)
      const objectInterval = setInterval(() => {
        objectTopic.subscribe((message) => {
          const scaleFactor = 0.1;

          const detectedNPCs = message.npc_list.map((npc) => ({
            x: (npc.position.x - egoPosition.x) * scaleFactor,
            y: (npc.position.y - egoPosition.y) * scaleFactor,
            z: (npc.position.z - egoPosition.z) * scaleFactor,
            width: npc.size.x || 3,
            height: npc.size.y || 3,
            depth: npc.size.z || 3,
          }));

          const detectedPedestrians = message.pedestrian_list.map((pedestrian) => ({
            x: (pedestrian.position.x - egoPosition.x) * scaleFactor,
            y: (pedestrian.position.y - egoPosition.y) * scaleFactor,
            z: (pedestrian.position.z - egoPosition.z) * scaleFactor,
            width: pedestrian.size.x || 1,
            height: pedestrian.size.y || 2,
            depth: pedestrian.size.z || 1,
          }));

          setObjects({ npc_list: detectedNPCs, pedestrian_list: detectedPedestrians });
        });
      }, 1000); // 1초마다 실행

      // 컴포넌트가 언마운트 될 때 인터벌 해제
      return () => {
        clearInterval(cameraInterval);
        clearInterval(egoInterval);
        clearInterval(objectInterval);
      };
    };

    const stopDataSubscriptions = startDataSubscriptions(); // 구독 시작

    // 컴포넌트 언마운트 시 구독 해제
    return () => {
      imageTopic.unsubscribe();
      egoStatusTopic.unsubscribe();
      objectTopic.unsubscribe();
      ros.current.close();
      stopDataSubscriptions(); // 인터벌 해제
    };
  }, [egoPosition]); // 의존성 배열에 egoPosition 추가

  // Canny Edge 처리 함수
  const processImage = (img) => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    ctx.drawImage(img, 0, 0, canvas.width, canvas.height);

    if (!window.cv || !window.cv.imread) {
      console.error('OpenCV is not ready yet.');
      return;
    }

    const src = window.cv.imread(canvas);
    const gray = new window.cv.Mat();
    const edges = new window.cv.Mat();

    window.cv.cvtColor(src, gray, window.cv.COLOR_RGBA2GRAY, 0);
    window.cv.Canny(gray, edges, 50, 150, 3, false);

    const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
    const mintColor = { r: 62, g: 243, b: 232 }; // 민트 RGB
    for (let i = 0; i < imageData.data.length; i += 4) {
      const edgeValue = edges.data[i / 4];
      if (edgeValue > 0) {
        imageData.data[i] = mintColor.r;  // R 값 설정
        imageData.data[i + 1] = mintColor.g;  // G 값 설정
        imageData.data[i + 2] = mintColor.b;  // B 값 설정
      } else {
        imageData.data[i] = 0;  // 배경은 검정색으로 설정
        imageData.data[i + 1] = 0;
        imageData.data[i + 2] = 0;
      }
    }
    ctx.putImageData(imageData, 0, 0);

    src.delete();
    gray.delete();
    edges.delete();
  };

  return (
    <div className="container">
      {/* Canny Edge Detection 이미지 캔버스 */}
      <canvas ref={canvasRef} width="640" height="480" className="imageCanvas"></canvas>

      {/* Three.js로 3D 객체 렌더링 */}
      <Canvas className="threeDCanvas" camera={{ position: [0, 10, 20], fov: 50 }}>
        <ambientLight intensity={0.5} />
        <pointLight position={[10, 10, 10]} />

        {/* NPC 차량 렌더링 */}
        {objects.npc_list.map((npc, index) => (
          <RotatingBox
            key={`npc-${index}`}
            position={[npc.x, npc.y, npc.z]}
            size={[npc.width, npc.height, npc.depth]}
            color="red"
          />
        ))}

        {/* 보행자 렌더링 */}
        {objects.pedestrian_list.map((pedestrian, index) => (
          <RotatingBox
            key={`pedestrian-${index}`}
            position={[pedestrian.x, pedestrian.y, pedestrian.z]}
            size={[pedestrian.width, pedestrian.height, pedestrian.depth]}
            color="blue"
          />
        ))}
      </Canvas>
    </div>
  );
};

export default CameraFeed;
