import React, { useState } from 'react';
import './App.css';
import CarIcon from './icons/CarIcon';
import MusicIcon from './icons/MusicIcon';
import RoadIcon from './icons/RoadIcon';
import CameraIcon from './icons/CameraIcon';
import VolumeIcon from './icons/VolumeIcon';
import VehicleInfo from './components/VehicleInfo.jsx';
import MusicPlayer from './components/MusicPlayer.jsx';
import RouteFinder from './components/RouteFinder.jsx';
import CameraView from './components/CameraView.jsx';
import MapView from './components/MapView.jsx'; 

const App = () => {
  const [activeScreen, setActiveScreen] = useState('car'); // 기본 화면 설정

  // 좌측 화면 초기화 함수 (초기 상태로 돌아감)
  const resetLeftScreen = () => {
    setActiveScreen('car'); // 좌측 화면을 기본 차량 정보 화면으로 초기화
  };

  const renderLeftScreen = () => {
    switch (activeScreen) {
      case 'car':
        return <VehicleInfo />;
      case 'music':
        return <MusicPlayer />;
      case 'road':
        return <RouteFinder resetLeftScreen={resetLeftScreen} />;
      case 'camera':
        return <CameraView />;
      default:
        return null;
    }
  };

  return (
    <div className="App">
      <div className="main-screen">
        {/* 좌측 화면 */}
        <div className="left-screen">{renderLeftScreen()}</div>

        {/* 우측 화면 (지도는 road 아이콘 클릭 시 표시) */}
        <div className="right-screen">{activeScreen === 'road' ? <MapView /> : null}</div>
      </div>

      {/* 하단 네비게이션 바 */}
      <div className="nav-bar">
        <CarIcon onClick={() => setActiveScreen('car')} />
        <MusicIcon onClick={() => setActiveScreen('music')} />
        <RoadIcon onClick={() => setActiveScreen('road')} />
        <CameraIcon onClick={() => setActiveScreen('camera')} />
        <VolumeIcon />
      </div>
    </div>
  );
};

export default App;
