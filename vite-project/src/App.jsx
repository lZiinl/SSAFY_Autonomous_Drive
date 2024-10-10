import React, { useState } from 'react';
import Draggable from 'react-draggable'; // Draggable import
import NavBar from './components/NavBar';  // NavBar 컴포넌트 import
import MusicPlayer from './components/MusicPlayer';  // MusicPlayer 컴포넌트 import
import RouteFinder from './components/RouteFinder';  // RouteFinder 컴포넌트 import
import MapView from './components/MapView';  // MapView 컴포넌트 import
import VelocityDisplay from './components/VelocityDisplay';  // VelocityDisplay import
import CameraFeed from './components/CameraFeed';
import AroundView from './components/AroundView';  // AroundView 컴포넌트 import
import ReverseView from './components/ReverseView';  // RearView 컴포넌트 import

const App = () => {
  const [activeScreen, setActiveScreen] = useState('car'); // 기본 화면은 차량 정보

  // 좌측 화면 렌더링 함수
  const renderLeftScreen = () => {
    if (activeScreen === 'car') {
      return <CameraFeed />;  
    }
    if (activeScreen === 'music') {
      return <CameraFeed />;  
    }
    if (activeScreen === 'road') {
      return <RouteFinder />;  
    }
    if (activeScreen === 'aroundView') {
      return <CameraFeed />;  
    }
    if (activeScreen === 'rearView') {
      return <CameraFeed />;  
    }
    
    return null;
  };

  // 우측 화면 렌더링 함수
  const renderRightScreen = () => {
    if (activeScreen === 'car') {
      return (
        <div className="relative w-full h-full" style={{ zIndex: 0 }}> {/* zIndex 값 낮게 설정 */} 
          <MapView />  
        </div>
      );
    }
    if (activeScreen === 'music') {
      return (
        <div className="relative w-full h-full">
          <MusicPlayer />
        </div>
      );
    }
    if (activeScreen === 'road') {
      return (
        <div className="relative w-full h-full" style={{ zIndex: 0 }}> {/* zIndex 값 낮게 설정 */} 
          <MapView />  
        </div>
      );
    }
    if (activeScreen === 'aroundView') {
      return <AroundView />;  
    }
    if (activeScreen === 'rearView') {
      return <ReverseView />;  
    }
    
    return null;
  };

  // 전체화면 AroundView 또는 RearView 렌더링 함수
  const renderFullScreen = () => {
    if (activeScreen === 'aroundView') {
      return (
        <div className="absolute top-0 left-0 w-full h-full z-40 bg-black"> {/* div1 내부에서 전체화면 렌더링 */}
          <AroundView /> {/* AroundView 컴포넌트 전체화면에 렌더링 */}
        </div>
      );
    }
    if (activeScreen === 'rearView') {
      return (
        <div className="absolute top-0 left-0 w-full h-full z-40 bg-black"> {/* div1 내부에서 전체화면 렌더링 */}
          <ReverseView /> {/* RearView 컴포넌트 전체화면에 렌더링 */}
        </div>
      );
    }
    return null;
  };

  return (
    <div className="h-screen w-screen flex flex-col bg-gray-100 font-sans justify-between relative">
      {/* 큰 컨테이너 (좌우 화면 및 네비게이션 바 포함) */}
      <div className="flex flex-col items-center w-[1920px] relative">
        {/* 좌우 화면 포함하는 div1 */}
        <div className="flex flex-row w-full h-[980px] bg-white relative">
          {/* 좌측 화면 */}
          <div className="w-[730px] bg-white shadow-inner border-r border-gray-300 overflow-hidden">
            {renderLeftScreen()}  {/* 좌측 화면 렌더링 */}
          </div>

          {/* 우측 화면 */}
          <div className="w-[1200px] bg-white flex justify-center items-center overflow-hidden relative">
            {renderRightScreen()}  {/* 우측 화면 렌더링 */}
          </div>

          {/* 드래그 가능한 박스 */}
          <Draggable>
            <div className="w-[150px] h-[150px] text-center flex items-center justify-center absolute bottom-20 right-20 cursor-move z-50"> {/* zIndex 값을 높게 설정 */}
              <VelocityDisplay />
            </div>
          </Draggable>
        </div>

        {/* 하단 네비게이션 바 (네비게이션 바가 div1 아래에 위치) */}
        <NavBar setActiveScreen={setActiveScreen} />
      </div>
    </div>
  );
};

export default App;
