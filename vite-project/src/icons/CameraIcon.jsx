import React from 'react';

const CameraIcon = ({ onClick }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    viewBox="0 0 24 24"
    width="50"
    height="50"
    className="nav-icon"
    onClick={onClick}
  >
    <path d="M12 7a5 5 0 1 1 0 10 5 5 0 0 1 0-10zm0 1a4 4 0 1 0 0 8 4 4 0 0 0 0-8zm-6.4-1h12.8a1 1 0 0 1 1 1v11.8a1 1 0 0 1-1 1H5.6a1 1 0 0 1-1-1V8a1 1 0 0 1 1-1z" />
  </svg>
);

export default CameraIcon;
