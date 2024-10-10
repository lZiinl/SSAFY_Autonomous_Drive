import React, { useEffect, useRef } from 'react';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';
import ROSLIB from 'roslib';
import proj4 from 'proj4'; // proj4 라이브러리

const MapView = () => {
  const mapContainerRef = useRef(null); // DOM 참조
  const mapInstanceRef = useRef(null); // Leaflet 지도 인스턴스 참조
  const vehicleMarkerRef = useRef(null); // 자동차 마커 참조 (useRef로 관리)
  const routeLineRef = useRef(null); // 경로 폴리라인 참조 (useRef로 관리)

  const lastUpdateTimeRef = useRef(Date.now()); // 마지막 업데이트 시간을 기록

  // 하드코딩된 Offset 값 (미리 확인한 값)
  const eastOffset = 302459.942;
  const northOffset = 4122635.537;

  // UTM 좌표계를 WGS84로 변환하는 함수
  const utmToLatLng = (x, y, zone = 52) => {
    // Offset 적용하여 UTM 좌표를 조정
    x += eastOffset;
    y += northOffset;

    // Proj4를 사용하여 UTM 좌표를 WGS84 좌표로 변환
    const utmProj = `+proj=utm +zone=${zone} +ellps=WGS84 +datum=WGS84 +units=m +no_defs`;
    const wgs84Proj = '+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs';
    const [longitude, latitude] = proj4(utmProj, wgs84Proj, [x, y]);

    return [latitude, longitude]; // [위도, 경도] 형식으로 반환
  };

  // 아이콘 설정을 useEffect 외부에서 정의하여 컴포넌트 전체에서 접근 가능하도록 설정
  const originIcon = L.icon({
    iconUrl: '/images/origin.png',  // 시작점 아이콘 이미지 경로
    iconSize: [75, 45],     // 아이콘 크기
    iconAnchor: [35, 40],   // 아이콘의 앵커 포인트
    popupAnchor: [0, 0]
  });

  const destinationIcon = L.icon({
    iconUrl: '/images/flag1.png',   // 끝점 아이콘 이미지 경로
    iconSize: [35, 55],     // 아이콘 크기
    iconAnchor: [9, 75],    // 아이콘의 앵커 포인트
    popupAnchor: [0, -20]
  });

  // 차량 아이콘 설정 (절대 경로 사용)
  const carIcon = L.icon({
    iconUrl: '/images/car.png',  // 차량 아이콘 이미지 경로
    iconSize: [35, 55],  // 아이콘 크기
    iconAnchor: [15, 25],  // 아이콘의 앵커 포인트
    popupAnchor: [-15, -76]
  });

  useEffect(() => {
    // 지도 초기화 (지도 인스턴스가 없을 때만)
    if (mapInstanceRef.current === null) {
      // 지도 인스턴스 초기화
      mapInstanceRef.current = L.map(mapContainerRef.current).setView([37.241768, 126.774465], 16);

      // OpenStreetMap 타일 추가
      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 16,
      }).addTo(mapInstanceRef.current);

      // 하드코딩된 자동차 위치에 마커 추가
      vehicleMarkerRef.current = L.marker([37.241768, 126.774465], {
        icon: carIcon,
      }).addTo(mapInstanceRef.current);
    }

    // ROS 연결 및 /gps와 /global_path 토픽 구독
    const ros = new ROSLIB.Ros({
      url: 'ws://192.168.56.101:9090',
    });

    ros.on('connection', () => {
      console.log('Connected to ROS');
    });

    ros.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
    });

    ros.on('close', () => {
      console.log('Connection to ROS closed');
    });

    // /gps 토픽 구독 (실시간 차량 위치)
    const gpsTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/gps',
      messageType: 'morai_msgs/GPSMessage',
    });

    gpsTopic.subscribe((gpsMsg) => {
      const now = Date.now();
      const updateInterval = 1000; // 1초 간격으로 업데이트 (1000ms)

      if (now - lastUpdateTimeRef.current >= updateInterval) {
        const { latitude, longitude } = gpsMsg;

        if (vehicleMarkerRef.current) {
          vehicleMarkerRef.current.setLatLng([latitude, longitude]); // 마커 위치 업데이트
        }

        console.log(`Updated vehicle position to: ${latitude}, ${longitude}`);
        lastUpdateTimeRef.current = now; // 마지막 업데이트 시간 갱신
      }
    });

    // /global_path 토픽 구독 (경로 데이터)
    const pathTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/global_path',
      messageType: 'nav_msgs/Path', // 경로 데이터 타입
    });

    pathTopic.subscribe((pathMsg) => {
      // 경로 좌표를 UTM -> 위도/경도로 변환하여 latlngs 배열에 저장
      const latlngs = pathMsg.poses.map((pose) => {
        const { x, y } = pose.pose.position;
        // UTM 좌표에서 WGS84로 변환 및 Offset 적용
        return utmToLatLng(x, y, 52); // UTM -> WGS84 변환 (존 52)
      });

      // 이전 경로 폴리라인이 있다면 제거
      if (routeLineRef.current) {
        mapInstanceRef.current.removeLayer(routeLineRef.current);
      }

      // 새로운 경로 폴리라인 추가
      routeLineRef.current = L.polyline(latlngs, { color: 'red', weight: 5 }).addTo(mapInstanceRef.current);

      // 경로가 지도에 잘 보이도록 지도의 범위를 경로에 맞춰 조정
      mapInstanceRef.current.fitBounds(routeLineRef.current.getBounds());

      // 경로의 시작점과 끝점에 마커 추가
      if (latlngs.length > 0) {
        // // 시작점 마커 추가
        // const startLatLng = latlngs[0];
        // L.marker(startLatLng, { icon: originIcon }).addTo(mapInstanceRef.current)
        //   .on('add', () => console.log("Start marker added to the map"));

        // 끝점 마커 추가
        const endLatLng = latlngs[latlngs.length - 1];
        L.marker(endLatLng, { icon: destinationIcon }).addTo(mapInstanceRef.current)
          .on('add', () => console.log("End marker added to the map"));

        console.log("Added start and end markers to the map");
      }

      console.log('Updated route on the map');
    });

    // 컴포넌트 언마운트 시 지도 및 ROS 연결 정리
    return () => {
      if (mapInstanceRef.current !== null) {
        mapInstanceRef.current.remove(); // 지도 인스턴스 제거
        mapInstanceRef.current = null; // 초기화
      }
      ros.close(); // ROS 연결 종료
    };
  }, []); // 의존성 배열을 비워 한 번만 실행되도록 설정

  return <div id="map" ref={mapContainerRef} style={{ width: '100%', height: '980px' }}></div>;
};

export default MapView;
