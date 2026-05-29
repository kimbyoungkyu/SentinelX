# ROS2 Humble 설치 가이드 (Ubuntu 22.04)

## 개요

본 문서는 Ubuntu 22.04 환경에서 ROS2 Humble Hawksbill을 설치하는 과정을 정리한다.

현재 환경:

- Ubuntu 22.04
- PX4 v1.15.4
- SIH (Simulation In Hardware)
- DDS 기반 시스템

---

# 시스템 업데이트

```bash
sudo apt update
sudo apt upgrade -y
```

---

# Locale 설정

ROS2는 UTF-8 Locale을 요구한다.

확인:

```bash
locale
```

설정:

```bash
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale     LC_ALL=en_US.UTF-8     LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

확인:

```bash
locale
```

---

# Universe Repository 활성화

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

---

# ROS2 Repository 추가

```bash
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Repository 등록:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

---

# ROS2 Humble 설치

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

---

# 환경 변수 설정

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# 설치 확인

```bash
ros2 --version
```

---

# 테스트

터미널 1

```bash
ros2 run demo_nodes_cpp talker
```

터미널 2

```bash
ros2 run demo_nodes_cpp listener
```

---

# 개발 도구 설치

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool git build-essential cmake ninja-build -y
```

---

# rosdep 초기화

```bash
sudo rosdep init
rosdep update
```

---

# Workspace 생성

```bash
mkdir -p ~/px4_ros2_ws/src
cd ~/px4_ros2_ws
colcon build
```

환경 적용:

```bash
echo "source ~/px4_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# PX4 개발용 패키지

## Micro XRCE DDS Agent

```bash
sudo apt install ros-humble-micro-ros-agent -y
```

실행:

```bash
MicroXRCEAgent udp4 -p 8888
```

---

## MAVROS

```bash
sudo apt install ros-humble-mavros* -y
```

---

# PX4 ROS2 메시지

```bash
cd ~/px4_ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
```

빌드:

```bash
cd ~/px4_ros2_ws
colcon build
```

환경 적용:

```bash
source install/setup.bash
```

확인:

```bash
ros2 interface list | grep px4_msgs
```

---

# PX4 + ROS2 구조

```text
PX4 SIH
    |
uXRCE-DDS
    |
ROS2
```

---

# 설치 확인 체크리스트

- Ubuntu 22.04
- ROS2 Humble 설치 완료
- rosdep 초기화 완료
- colcon 정상 동작
- px4_ros2_ws 생성 완료
- px4_msgs 빌드 완료
- MicroXRCEAgent 설치 완료

위 항목이 모두 완료되면 PX4 Offboard 개발 준비가 완료된다.
