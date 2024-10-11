<table align="center">
<tr>
<td align="center">
<img alt='logo' src='./README_IMG/안전모로고.png' width=100% align='center'>
</a>
</td>
</tr>
</table>
 

## Index
#### &emsp; [➤ 프로젝트 소개](#-프로젝트-소개)<br>
#### &emsp; [➤ 프로젝트 설계](#-프로젝트-설계)<br>
#### &emsp; [➤ 기능 소개](#-기능-소개)<br>
#### &emsp; [➤ 산출물](#-산출물)<br>
<br>


# 🚔 프로젝트 소개

## ADAS와 자율주행을 탑재한 스마트 주행 솔루션
1. 주행 경로 설정 (최소 거리, 최소 비용, 어린이 보호구역 회피)
2. 자율주행
3. 어라운드 뷰
4. 후방 카메라

<br>

## 프로젝트 기간

| 프로젝트 기간 | 2024.08.19 ~ 2024.10.11 (7주) |
|---|---|
<br>

## 팀 소개
<table>
  <thead>
    <tr>
      <th style="text-align: center;">윤의웅</th>
      <th style="text-align: center;">박건국</th>
      <th style="text-align: center;">정우영</th>
      <th style="text-align: center;">조정훈</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align: center;">어라운드 뷰<br>후방카메라</td>
      <td style="text-align: center;">경로 생성<br>경로-자율주행 통합
      </td>
      <td style="text-align: center;">ROS-Web 통신<br>시스템 통합
      </td>
      <td style="text-align: center;">자율주행 제어<br>경로 시각화</td>
    </tr>
  </tbody>
</table>
<br>

## 기획 배경

<img alt='func3.2' src='./README_IMG/자율시장.jpg'/>
자율주행 기술은 빠르게 발전하고 있지만, 아직 상용화를 위해서는 다양한 주행 환경에서의 철저한 테스트와 검증이 필요합니다. 
<br><br>그러나 실제 도로에서 테스트를 진행하는 것은 비용이 많이 들고 위험성이 크기 때문에, MORAI SIM과 같은 시뮬레이션을 통해 다양한 시나리오를 안전하고 효율적으로 검증할 수 있는 필요성이 대두되고 있습니다. 
<br><br>시뮬레이션 환경에서는 복잡한 도로 상황을 반복적으로 실험할 수 있으며, 위험한 상황도 안전하게 재현할 수 있다는 장점이 있습니다. 또한, ADAS를 탑재한 자율주행 기술은 운전자의 편의와 안전성을 극대화하는 핵심 요소로, 실제 차량에 적용하기 전 철저한 시뮬레이션을 통해 성능을 최적화하고 있습니다.

  
<br>

# 🚔 프로젝트 설계
## 개발 환경

<h3>OS</h3>
<img alt="ubuntu" src="https://img.shields.io/badge/ubuntu -E95420?style=for-the-badge&logo=ubuntu&logoColor=white"/>

<h3>MiddleWare</h3>
<img alt="ROS" src="https://img.shields.io/badge/ROS MELODIC -22314E?style=for-the-badge&logo=ROS&logoColor=white"/>

<h3>Simulator</h3>
<img alt="MORAI" src="https://img.shields.io/badge/MORAI SIM-0000C9?style=for-the-badge&logo=MORAI">
<h3>Embedded</h3>
<img alt="python" src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=Python&logoColor=white"/> 
<img alt="openvc" src="https://img.shields.io/badge/OpenCV-black?style=for-the-badge&logo=OpenCV&logoColor=white">
</p>

<h3>Server</h3>
<img alt="Flask" src="https://img.shields.io/badge/Flask-000000.svg?&style=for-the-badge&logo=Flask&logoColor=white"/>

<h3>Frontend</h3>
<p>
<img alt="HTML5" src="https://img.shields.io/badge/HTML5-E34F26.svg?&style=for-the-badge&logo=HTML5&logoColor=white"/>

<img alt="js" src="https://img.shields.io/badge/javascript-%23323330.svg?style=for-the-badge&logo=javascript&logoColor=%23F7DF1E">
<img alt="React" src="https://img.shields.io/badge/React-61DAFB?style=for-the-badge&logo=React&logoColor=black">
<img alt="vite" src="https://img.shields.io/badge/Vite-646CFF?style=for-the-badge&logo=Vite&logoColor=white">
<img alt="CSS" src="https://img.shields.io/badge/CSS-1572B6.svg?&style=for-the-badge&logo=CSS3&logoColor=white"/>
<img alt="Tailwind" src="https://img.shields.io/badge/TailwindCSS-38B2AC?style=for-the-badge&logoColor=white&logo=tailwindcss">
</p>
<h3>협업 툴</h3>
<p>
<a href="https://ssafy.atlassian.net/jira/software/c/projects/S11P12C104/boards/7000/timeline">
<img alt='jira' src="https://img.shields.io/badge/Jira-0052CC?style=for-the-badge&logo=Jira&logoColor=white">
</a>
<a href="https://www.notion.so/1-C104-c02fce6e587f4bb5acf3c834544dd04f">
<img alt='jira' src="https://img.shields.io/badge/Notion-black?style=for-the-badge&logo=Notion&logoColor=white">
</a>
</p>

<br>
  
<br>

## 시스템 아키텍처
<img alt='func4.2' src='./README_IMG/아키텍처.JPG'>

<br>

# 🚔 기능 소개

### 1. 주행 경로 설정 (최소 거리, 최소 비용, 어린이 보호구역 회피)


<p style='font-size:16px;'><b>1.1 A*알고리즘</b></p>
<img alt='func1.1' src='./README_IMG/a,dijk시간비교.JPG'>

<img alt='func1.1' src='./README_IMG/다익+A스타.JPG'>


- 기존 다익스트라 알고리즘에서 A* 알고리즘을 활용한 주행 경로 탐색 시간 80% 단축

### 2. 자율주행

### 자율주행 로직
<img alt='func2.1' src='./README_IMG/자율주행 로직.JPG'>

- 경로 생성 + 장애물 정보 -> 차량 제어

<p style='font-size:16px;'><b>2.1 장애물 회피</p>
<img alt='func2.2' src='./README_IMG/장애물 회피 주행.gif'>
<p style='font-size:16px;'>2.2 PID 제어</p>
<img alt='func2.2' src='./README_IMG/직선 주행.gif'>
<p style='font-size:16px;'>2.3 곡률 기반 제어</p>
<img alt='func2.2' src='./README_IMG/커브 주행.gif'>




### 3. 어라운드 뷰
<img alt='func2.1' src='./README_IMG/이미지 변환 과정 사진.png'>
</b>

- 카메라 배치
- 캘리브레이션 및 보정
- 이미지 변환
- 이미지 스티칭
- 경계 영역 및 블랜딩

<b>
<p style='font-size:16px;'>3.1 광각 카메라 배치</p>
<img alt='func3.1' src='./README_IMG/카메라 4대.png'>

<p style='font-size:16px;'>3.2 캘리브레이션 및 보정</p>
<img alt='func3.1' src='./README_IMG/체크보드 사진_!.jpg'>
<img alt='func3.1' src='./README_IMG/체크보드 사진_2.jpg'>

<p style='font-size:16px;'>3.3 이미지 변환</p>
<img alt='func3.1' src='./README_IMG/이미지처리.JPG'>

<p style='font-size:16px;'>3.4 이미지 스티칭</p>
<img alt='func3.1' src='./README_IMG/이미지스티칭.JPG'>

<p style='font-size:16px;'>3.5 경계 영역 및 블랜딩</p>
<img alt='func3.2' src='./README_IMG/이미지블랜딩.JPG'>


    
### 4. 후방 카메라

<img alt='func2.1' src='./README_IMG/후방카메라로직.JPG'>

<p style='font-size:16px;'>4.1 전체 로직 </p>
<img alt='func4.1' src='./README_IMG/경고및후진가이드.JPG'>

<p style='font-size:16px;'>4.2 속도 및 조향 정보 기반 경로 추출</p>
<img alt='func4.2' src='./README_IMG/후방_카메라.png'>



<br><br>

---

### 커밋 컨벤션
- feat : 새로운 기능 추가
- fix : 버그 수정
- hotfix : 급하게 치명적인 버그 수정
- docs : 문서 수정
- style : 코드 포맷팅, 세미콜론 등의 스타일 수정(코드 자체 수정 X)
- refactor : 프로덕션 코드 리팩토링
- test : 테스트 코드, 테스트 코드 리팩토링
- chore : 빌드 과정 또는 보조 기능(문서 생성 기능 등) 수정
- rename : 파일 혹은 폴더명을 수정하거나 옮기는 작업만인 경우
- remove : 파일을 삭제하는 작업만 수행한 경우
- comment : 필요한 주석 추가 및 변경

