import React from 'react';

const MusicIcon = ({ onClick }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    viewBox="0 0 24 24"
    width="50"
    height="50"
    className="nav-icon"
    onClick={onClick}
  >
    <path d="M12 3v10.55a4 4 0 1 0 1.5 3.95A3.99 3.99 0 0 0 14 13.92V5.5h4V3h-6z" />
  </svg>
);

export default MusicIcon;
