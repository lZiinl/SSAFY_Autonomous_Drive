import React from 'react';

const VolumeIcon = ({ onClick }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    viewBox="0 0 24 24"
    width="50"
    height="50"
    className="nav-icon"
    onClick={onClick}
  >
    <path d="M9 9v6h-4l-4 4v-14l4 4h4zm11.6 3a5 5 0 0 0-1-3.59v7.18a5 5 0 0 0 1-3.59z" />
  </svg>
);

export default VolumeIcon;
