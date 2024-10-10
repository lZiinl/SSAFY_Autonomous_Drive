import React from 'react';
import Navi from '../icons/Navi.png';
import Car from '../icons/Car.png';
import Back from '../icons/Camera.png';
import Around from '../icons/Around.png';
import Sound from '../icons/Sound.png';

const NavBar = ({ setActiveScreen }) => {
  return (
    <div className="w-[1920px] h-[100px] bg-gray-900 flex justify-around items-center shadow-lg space-x-0">
      <button
        className="flex flex-col items-center justify-center bg-gray-900 text-white hover:bg-gray-700 hover:text-yellow-400 p-4 border-none flex-grow transition-colors duration-300 ease-in-out rounded-none shadow-none relative"
        onClick={() => setActiveScreen('car')}
      >
        <img src={Car} alt="Main" className="w-13 h-12 mx-auto border-none" />
        <span className="text-2xl">Main</span>
      </button>
      <button
        className="flex flex-col items-center justify-center bg-gray-900 text-white hover:bg-gray-700 hover:text-yellow-400 p-4 border-none flex-grow transition-all duration-300 ease-in-out rounded-none shadow-none relative"
        onClick={() => setActiveScreen('music')}
      >
        <img src={Sound} alt="Sound" className="w-12 h-12 mx-auto border-none" />
        <span className="text-2xl">Music</span>
      </button>
      <button
        className="flex flex-col items-center justify-center bg-gray-900 text-white hover:bg-gray-700 hover:text-yellow-400 p-4 border-none flex-grow transition-all duration-300 ease-in-out rounded-none shadow-none relative"
        onClick={() => setActiveScreen('road')}
      >
        <img src={Navi} alt="Navigation" className="w-15 h-12 mx-auto border-none" />
        <span className="text-2xl">Navigation</span>
      </button>
      <button
        className="flex flex-col items-center justify-center bg-gray-900 text-white hover:bg-gray-700 hover:text-yellow-400 p-4 border-none flex-grow transition-all duration-300 ease-in-out rounded-none shadow-none relative"
        onClick={() => setActiveScreen('aroundView')}
      >
        <img src={Around} alt="Around" className="w-15 h-12 mx-auto border-none" />
        <span className="text-2xl">Around View</span>
      </button>
      <button
        className="flex flex-col items-center justify-center bg-gray-900 text-white hover:bg-gray-700 hover:text-yellow-400 p-4 border-none flex-grow transition-all duration-300 ease-in-out rounded-none shadow-none relative"
        onClick={() => setActiveScreen('rearView')}
      >
        <img src={Back} alt="Back" className="w-17 h-12 mx-auto border-none" />
        <span className="text-2xl">Back View</span>
      </button>
    </div>
  );
};

export default NavBar;
