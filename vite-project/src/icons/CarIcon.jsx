import React from 'react';

const CarIcon = ({ onClick }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    viewBox="0 0 24 24"
    width="50"
    height="50"
    className="nav-icon"
    onClick={onClick}
  >
    <path d="M3 11l2-5h14l2 5v8a2 2 0 0 1-2 2h-2a2 2 0 0 1-2-2H9a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-8zm1.5 1.5A1.5 1.5 0 1 1 6 14a1.5 1.5 0 0 1-1.5-1.5zM18 13a1.5 1.5 0 1 0 1.5 1.5A1.5 1.5 0 0 0 18 13zm-9 5h6v1H9z" />
  </svg>
);

export default CarIcon;
