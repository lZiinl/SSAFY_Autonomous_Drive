import React, { useState, useEffect } from 'react';

// WebSocket 연결 정보
const ROS_WEBSOCKET_URL = 'ws://192.168.56.101:9090'; // ROS WebSocket URL

const RouteFinder = () => {
  const [localEndPoint, setLocalEndPoint] = useState(''); // 도착지 상태
  const [selectedOption, setSelectedOption] = useState('distance'); // 기본값은 '최단 거리'
  const [result, setResult] = useState(null);
  const [ws, setWs] = useState(null); // WebSocket 상태 관리

  // WebSocket 연결 설정
  useEffect(() => {
    const socket = new WebSocket(ROS_WEBSOCKET_URL);

    socket.onopen = () => {
      console.log('WebSocket 연결됨');
    };

    socket.onclose = () => {
      console.log('WebSocket 연결 끊김');
    };

    socket.onerror = (error) => {
      console.error('WebSocket 오류:', error);
    };

    setWs(socket); // WebSocket 객체 저장

    return () => {
      if (socket) {
        socket.close(); // 컴포넌트 언마운트 시 소켓 종료
      }
    };
  }, []);

  // 경로 탐색 버튼 클릭 시 경로 계산 결과 출력 및 ROS 토픽 발행
  const handleFindRoute = () => {
    if (localEndPoint) {
      // 경로 결과 임의로 설정 (더미값)
      setResult({
        distance: '10 km',
        time: '15분',
        cost: '1,250원',
        arrival: '오후 12:27 도착',
      });

      // 선택한 옵션에 따라 ROS 토픽 메시지 발행
      let optionValue;

      if (selectedOption === 'distance') {
        optionValue = 1;
      } else if (selectedOption === 'cost') {
        optionValue = 2;
      } else if (selectedOption === 'safeZone') {
        optionValue = 3;
      }

      // 도착지를 숫자로 변환
      const destination = parseInt(localEndPoint);

      // ROS 메시지 형식
      const message = {
        op: 'publish',
        topic: '/goal_data',
        msg: {
          data: [optionValue, destination]
        }
      };

      // WebSocket을 통해 메시지 전송
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(message));
        console.log('전송된 메시지:', message);  // 메시지 확인을 위한 콘솔 출력
      } else {
        console.error('WebSocket이 열려 있지 않음');
      }
    }
  };

  // 경로 옵션 선택
  const handleOptionClick = (option) => {
    setSelectedOption(option); // 선택된 옵션 업데이트
  };

  return (
    <div className="w-full h-full p-8 bg-white flex flex-col justify-start">
      <h2 className="text-3xl font-bold mb-6">경로 찾기</h2>

      {/* 도착지 입력 필드 및 경로 탐색 버튼 */}
      <div className="flex items-center mb-6 space-x-4">
        <input
          type="text"
          className="flex-grow border border-gray-300 rounded-lg p-4"
          placeholder="도착지 입력"
          value={localEndPoint}
          onChange={(e) => setLocalEndPoint(e.target.value)}
        />
        <button
          className="w-1/4 bg-blue-500 text-white px-6 py-4 rounded-lg"
          onClick={handleFindRoute}
        >
          경로 탐색
        </button>
      </div>

      {/* 경로 옵션 선택 (버튼 형식) */}
      <div className="mb-6">
        <h3 className="text-xl font-semibold mb-4">경로 옵션 선택</h3>
        <div className="flex space-x-4">
          <button
            className={`flex-grow p-4 rounded-lg font-semibold text-white ${
              selectedOption === 'distance' ? 'bg-blue-500' : 'bg-gray-400'
            }`}
            onClick={() => handleOptionClick('distance')}
          >
            최단 거리
          </button>

          <button
            className={`flex-grow p-4 rounded-lg font-semibold text-white ${
              selectedOption === 'cost' ? 'bg-blue-500' : 'bg-gray-400'
            }`}
            onClick={() => handleOptionClick('cost')}
          >
            최소 비용
          </button>

          <button
            className={`flex-grow p-4 rounded-lg font-semibold text-white ${
              selectedOption === 'safeZone' ? 'bg-blue-500' : 'bg-gray-400'
            }`}
            onClick={() => handleOptionClick('safeZone')}
          >
            보호구역 회피
          </button>
        </div>
      </div>

      {/* 경로 결과 출력 */}
      {result && (
        <div className="mt-6">
          <p className="text-lg font-bold mb-2">탐색 결과</p>
          <p className="mb-2">거리: {result.distance}</p>
          <p className="mb-2">예상 도착 시간: {result.time}</p>
          <p className="mb-2">요금: {result.cost}</p>
          <p className="mb-2">도착 예정: {result.arrival}</p>
        </div>
      )}
    </div>
  );
};

export default RouteFinder;
