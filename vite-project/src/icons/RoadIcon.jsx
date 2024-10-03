import React from 'react';

const RoadIcon = ({ onClick }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    viewBox="0 0 24 24"
    width="50"
    height="50"
    className="nav-icon"
    onClick={onClick}
  >
    <path d="M10 2H4v18h16V2h-6v2h4v14H6V4h4V2z" />
  </svg>
);

export default RoadIcon;
