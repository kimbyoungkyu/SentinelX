# PX4 SIH (Simulation-In-Hardware) v1.15.4 설치 및 실행 가이드

## 개요

본 문서는 PX4 SIH(Simulation-In-Hardware) 환경을 Ubuntu에서 구축하고 실행하는 과정을 정리한 문서이다.

실제 검증된 환경은 다음과 같다.

- PX4-Autopilot v1.15.4
- Ubuntu 22.04
- PX4 SITL + SIH QuadX
- ROS2 Humble 연동 예정

---

# 왜 PX4 v1.15.4를 선택했는가?

## 1. 안정성 우선

본 프로젝트의 목적은 다음과 같다.

- AgentX
- ROS2
- DDS
- DTSim
- DTPilotX

연동을 위한 디지털 트윈 플랫폼 구축

따라서 최신 기능보다 안정성이 중요하다.

PX4 v1.15.4는 다음 특성을 가진다.

- SIH 기능 포함
- ROS2 연동 검증 사례 다수
- EKF2 안정성 우수
- Multicopter 안정성 우수
- 커뮤니티 사용 사례 풍부

## 2. SIH 지원 확인

실제 소스 확인 결과:

- src/modules/simulation/simulator_sih
- ROMFS/px4fmu_common/init.d-posix/airframes/10040_sihsim_quadx

존재함을 확인하였다.

## 3. 최신 버전 대비 리스크 감소

- DDS 변경
- Gazebo 변경
- MAVLink 변경
- Simulation 구조 변경

초기 구축 단계에서는 검증된 버전 사용이 유리하다.

---

# 설치

## PX4 다운로드

```bash
cd ~

git clone https://github.com/PX4/PX4-Autopilot.git

cd PX4-Autopilot

git checkout v1.15.4

git submodule update --init --recursive
```

## 버전 확인

```bash
git describe --tags
```

예상 출력:

```text
v1.15.4
```

---

# Ubuntu 의존성 설치

```bash
bash Tools/setup/ubuntu.sh
```

설치 후:

```bash
source ~/.bashrc
```

또는 터미널 재시작

---

# 빌드

```bash
cd ~/PX4-Autopilot

make px4_sitl
```

성공 시:

```text
PX4 config: px4_sitl_default
```

빌드 결과:

```text
build/px4_sitl_default
```

---

# SIH 존재 확인

```bash
find . -iname "*sih*"
```

확인 항목:

```text
ROMFS/px4fmu_common/init.d-posix/airframes/10040_sihsim_quadx
src/modules/simulation/simulator_sih
```

---

# SIH 실행

```bash
cd ~/PX4-Autopilot/build/px4_sitl_default

PX4_SIM_MODEL=sihsim_quadx \
./bin/px4 \
-s etc/init.d-posix/rcS \
etc
```

---

# 정상 실행 확인

```text
INFO [simulator_sih]
INFO [ekf2]
INFO [commander]

pxh>
```

---

# Arm

```bash
commander arm
```

정상 로그:

```text
INFO [commander] Armed by external command
```

---

# Takeoff

```bash
commander takeoff
```

정상 로그:

```text
INFO [commander] Takeoff detected
```

---

# 상태 확인

```bash
listener vehicle_attitude
listener vehicle_local_position
listener sensor_gps
listener vehicle_global_position
```

---

# 착륙

```bash
commander land
```

정상 로그:

```text
INFO [navigator] RTL: land at destination
INFO [commander] Landing detected
INFO [commander] Disarmed by landing
```

---

# SIH GPS 시작 위치

```bash
param show SIH_LOC_LAT0
param show SIH_LOC_LON0
param show SIH_LOC_H0
```

---

# Null Island 설정

```bash
param set SIH_LOC_LAT0 0
param set SIH_LOC_LON0 0
param set SIH_LOC_H0 0

param save

reboot
```

---

# MAVLink System ID 설정

```bash
param set MAV_SYS_ID 10

param save

reboot
```

---

# 최종 실행 명령

```bash
cd ~/PX4-Autopilot/build/px4_sitl_default

PX4_SIM_MODEL=sihsim_quadx \
./bin/px4 \
-s etc/init.d-posix/rcS \
etc
```
