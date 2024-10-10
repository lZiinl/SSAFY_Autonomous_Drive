import React from 'react';

const MusicPlayer = () => {
  return (
    <div className="p-4 bg-gray-800 rounded-lg text-white w-full h-full">
      {/* 유튜브 대신 외부 사이트 임베드 */}
      <div className="w-full h-full">
        <iframe
          className="w-full h-full"
          src="https://www.melon.com/tv/index.htm"  // 원하는 URL로 변경 가능
          title="External Site"
          frameBorder="0"
          allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
          allowFullScreen 
        ></iframe>
      </div>
    </div>
  );
};

export default MusicPlayer;
