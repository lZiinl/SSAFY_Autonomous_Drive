import React, { useState } from 'react';

const RouteFinder = ({ resetLeftScreen }) => {
  const [startPoint, setStartPoint] = useState('');  // 출발지 상태
  const [endPoint, setEndPoint] = useState('');  // 도착지 상태
  const [routeOptionsVisible, setRouteOptionsVisible] = useState(false);
  const [selectedOption, setSelectedOption] = useState(null);
  const [result, setResult] = useState(null);
  const [navigationStarted, setNavigationStarted] = useState(false);

  // 출발지, 도착지가 변경되면 세션 리셋
  const handlePointChange = (e, setter) => {
    setter(e.target.value);  // 상태 업데이트
    resetSession(); // 출발지/도착지 변경 시 세션 리셋
  };

  // 세션 리셋 함수 (길찾기 페이지의 초기 상태로 리셋)
  const resetSession = () => {
    setRouteOptionsVisible(false);
    setSelectedOption(null);
    setResult(null);
    setNavigationStarted(false); // 길 안내 메시지도 리셋
  };

  // 길찾기 버튼 클릭 시 경로 옵션 표시
  const handleFindRoute = () => {
    if (startPoint && endPoint) {
      setRouteOptionsVisible(true);
    }
  };

  // 경로 옵션 버튼 클릭 시 거리 및 예상 시간 계산
  const handleOptionClick = (option) => {
    setSelectedOption(option);
    setNavigationStarted(false); // 클릭할 때마다 길 안내 메시지를 버튼으로 초기화
    if (option === 'distance') {
      setResult({ distance: '10 km', time: '15분' });
    } else if (option === 'cost') {
      setResult({ distance: '12 km', time: '18분' });
    } else if (option === 'safeZone') {
      setResult({ distance: '11 km', time: '20분' });
    }
  };

  // 길 안내 시작
  const handleStartNavigation = () => {
    setNavigationStarted(true);
  };

  // 새로고침 버튼 클릭 시 길찾기 초기화
  const handleRefresh = () => {
    resetSession(); // 길찾기 화면만 초기화
    setStartPoint(''); // 출발지 초기화
    setEndPoint(''); // 도착지 초기화
  };

  return (
    <div className="route-finder">
      <h2>경로 찾기</h2>

      {/* 출발지, 도착지 입력 필드 */}
      <input
        type="text"
        className="input-field"
        placeholder="출발지 입력"
        value={startPoint}
        onChange={(e) => handlePointChange(e, setStartPoint)}
      />
      <input
        type="text"
        className="input-field"
        placeholder="도착지 입력"
        value={endPoint}
        onChange={(e) => handlePointChange(e, setEndPoint)}
      />

      {/* 길찾기 및 새로고침 버튼 */}
      <div className="button-group">
        <button className="find-route-button" onClick={handleFindRoute}>
          길찾기
        </button>
        <button className="refresh-button" onClick={handleRefresh}>
          새로고침
        </button>
      </div>

      {/* 경로 옵션 세션 */}
      {routeOptionsVisible && (
        <div className="route-session">
          <h3>경로 옵션</h3>
          <div className="route-buttons">
            <button
              className={`route-button ${selectedOption === 'distance' ? 'active' : ''}`}
              onClick={() => handleOptionClick('distance')}
            >
              최단 거리
            </button>
            <button
              className={`route-button ${selectedOption === 'cost' ? 'active' : ''}`}
              onClick={() => handleOptionClick('cost')}
            >
              최소 비용
            </button>
            <button
              className={`route-button ${selectedOption === 'safeZone' ? 'active' : ''}`}
              onClick={() => handleOptionClick('safeZone')}
            >
              보호구역 회피
            </button>
          </div>

          {/* 경로 결과 출력 */}
          {result && (
            <div className="route-results">
              <p>거리: {result.distance}</p>
              <p>예상 도착 시간: {result.time}</p>
            </div>
          )}

          {/* 길 안내 시작 버튼 (결과가 있을 때만 표시) */}
          {result && !navigationStarted && (
            <button className="start-navigation-button" onClick={handleStartNavigation}>
              길 안내 시작
            </button>
          )}

          {/* 길 안내가 시작되면 메시지 출력 */}
          {navigationStarted && (
            <>
              <p>길 안내를 시작합니다...</p>
            </>
          )}
        </div>
      )}
    </div>
  );
};

export default RouteFinder;
