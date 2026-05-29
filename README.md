# PX4 SIH + ROS2 Humble 설치 가이드

본 문서는 PX4 SIH 기반 디지털 트윈 및 AgentX 개발 환경 구축을 위한 설치 문서 모음입니다.

## 설치 순서

### 0. PX4 SIH v1.15.4 설치

- PX4 v1.15.4 선택 이유
- PX4 다운로드
- 의존성 설치
- SIH 빌드
- SIH 실행
- MAV_SYS_ID 설정
- GPS 위치 설정

📄 [0 PX4_SIH_v1.15.4 설치하기.md](./0%20PX4_SIH_v1.15.4%20설치하기.md)

---

### 1. ROS2 Humble 설치

- ROS2 Humble 설치
- rosdep 초기화
- colcon 설치
- px4_msgs 설치
- Micro XRCE DDS Agent 설치
- MAVROS 설치

📄 [1 ROS2_Humble 설치하기.md](./1%20ROS2_Humble%20설치하기.md)

---

## 최종 목표 구조

```text
PX4 SIH
    │
    ▼
uXRCE-DDS
    │
    ▼
ROS2 Humble
    │
    ▼
AgentX
    │
    ▼
DTSpaceTime
```

## 권장 설치 순서

1. PX4 SIH 설치
2. ROS2 Humble 설치
3. px4_msgs 빌드
4. Micro XRCE DDS Agent 실행
5. PX4 SIH 실행
6. ROS2 Offboard 예제 실행
7. AgentX 연동

## 검증 완료 환경

| 항목 | 버전 |
|--------|--------|
| Ubuntu | 22.04 |
| PX4 | v1.15.4 |
| ROS2 | Humble |
| Simulator | SIH QuadX |
| DDS | Micro XRCE DDS |