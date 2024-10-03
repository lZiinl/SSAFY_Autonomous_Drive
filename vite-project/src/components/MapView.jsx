// MapView.jsx
import React from 'react';

const MapView = () => {
  return (
    <div style={{ width: '1200px', height: '980px', display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
      <iframe src="/index4.html" title="Map View" style={{ width: '100%', height: '100%', border: 'none' }} />
    </div>
  );
};

export default MapView;

