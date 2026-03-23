# SO-ARM101 Ubuntu 작업 가이드

이 문서는 SO-ARM101 로봇 팔 프로젝트의 우분투 환경 설정 및 실행 가이드입니다.

---

## 완료된 작업

### 1. data.yaml 경로 수정
- 위치: `src/garbage_detection/data/data.yaml`
- 상대 경로 → 절대 경로로 변경
```yaml
train: /home/chan/SO-ARM101/src/garbage_detection/data/train/images
val:   /home/chan/SO-ARM101/src/garbage_detection/data/valid/images
test:  /home/chan/SO-ARM101/src/garbage_detection/data/test/images
```

---

### 2. ultralytics 설치
```bash
conda activate lerobot
pip install ultralytics  # 이미 설치됨 (v8.4.23)
```

---

### 3. YOLOv8 모델 학습

**학습 스크립트**: `src/garbage_detection/train.py`

```bash
conda activate lerobot
python src/garbage_detection/train.py
```

**학습 결과 (yolov8n_garbage2 - Early Stopping, 39 epoch):**
| 클래스 | mAP50 | mAP50-95 |
|--------|-------|----------|
| 전체 | 0.972 | 0.718 |
| bottle | 0.995 | 0.766 |
| bottle-cap | 0.926 | 0.555 |
| box | 0.995 | 0.833 |

**모델 저장 위치:** `src/garbage_detection/runs/yolov8n_garbage2/weights/best.pt`

**학습 설정:**
- epochs: 50000 (Early Stopping patience=20 → 39 epoch에서 종료)
- imgsz: 640
- device: GPU 0 (RTX 2080 Ti)

> **주의:** 현재 모델은 Roboflow 인터넷 데이터셋으로 학습됨.
> 실제 로봇 카메라 시점과 달라 인식률이 낮음 (병을 glass로 분류하는 문제 발생).
> 추후 실제 카메라로 촬영한 데이터로 재학습 필요.

---

### 4. 로봇 캘리브레이션

캘리브레이션 파일 저장 위치:
- Follower: `~/.cache/huggingface/lerobot/calibration/robots/so_follower/`
- Leader: `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/`

> Leader 팔 불필요 (프로젝트에서 Follower 팔만 사용)

---

### 5. ROS2 Jazzy 설치

Ubuntu 24.04 (Noble) 기준:

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install -y ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

---

### 6. Intel RealSense D405 연결

**설치:**
```bash
# realsense-ros ROS2 패키지 설치
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description

# udev 룰 설치 (USB 권한)
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**동작 확인:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

**주의사항:**
- D405는 640x480 미지원 → **848x480** 사용
- USB 허브 없이 PC에 직접 연결 권장 (허브 2단계 시 프레임 수신 불안정)
- pyrealsense2는 lerobot conda 환경에 이미 설치됨 (v2.56.5)

---

### 7. URDF 파일 생성

**위치:** `src/so101.urdf`

공식 SO-101 URDF joint 값 기반으로 생성
- 출처: https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf

**관절 정보:**
| 관절 | 범위 |
|------|------|
| shoulder_pan | -1.92 ~ +1.92 rad |
| shoulder_lift | -1.75 ~ +1.75 rad |
| elbow_flex | -1.69 ~ +1.69 rad |
| wrist_flex | -1.66 ~ +1.66 rad |
| wrist_roll | -2.74 ~ +2.84 rad |

---

### 8. Pick & Place 통합 코드

**메인 스크립트:** `src/pick_and_place.py`

**파이프라인:**
```
D405 카메라 (848x480)
    ↓
YOLOv8 감지 (화면 상단 60%만 사용, 하단 그리퍼 영역 제외)
    ↓
Visual Servoing (shoulder_pan/lift로 물체 화면 중앙 정렬)
    ↓
Depth → 3D 좌표 변환 (Eye-in-Hand, FK 기반 동적 변환)
    ↓
IKPy (URDF 기반 역기구학)
    ↓
hover → pick → 그리퍼 닫기 → 들어올리기 → drop → 복귀
```

**실행:**
```bash
conda activate lerobot
python src/pick_and_place.py
```

**주요 설정값 (`pick_and_place.py`):**
```python
ROBOT_PORT   = "/dev/ttyACM0"   # 실제 포트 확인 필요 (ACM0 or ACM1)
CONF_THRESH  = 0.25             # YOLO 신뢰도 임계값
ROI_BOTTOM   = 0.60             # 화면 상단 60%만 감지
CAM_Z        = 0.035            # 카메라 wrist 위 3.5cm
CAM_TILT     = 45도             # 카메라 아래로 45도 기울어짐
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE= 80.0
```

**DROP 위치:** 현재 임시값 → 실측 후 수정 필요
```python
DROP = {
    "shoulder_pan.pos":  45.0,
    "shoulder_lift.pos": -20.0,
    "elbow_flex.pos":    30.0,
    ...
}
```

---

### 9. 디버그 카메라 스크립트

**위치:** `src/debug_camera.py`

```bash
conda activate lerobot
python src/debug_camera.py
```

- 실시간 카메라 영상 + YOLO 감지 결과 오버레이 표시
- `s` 키: 현재 프레임 저장 (`src/debug_frame_XXX.jpg`)
- `q` 키: 종료
- 노란 선: ROI 경계선 (하단 그리퍼 영역 제외)

---

## 남은 작업

### 1학기 (완성 목표)
- [ ] 실제 로봇 카메라로 데이터 수집 (물체별 100~200장)
  - `debug_camera.py`에서 `s` 키로 촬영
  - Roboflow 업로드 → 라벨링 → export
- [ ] YOLOv8 재학습 (실제 카메라 데이터)
- [ ] DROP 위치 실측 및 코드 반영
- [ ] Pick & Place 전체 동작 테스트

### 2학기 (자율주행 통합)
- [ ] 탑 카메라 장착 (차량 상단 RGB 카메라)
- [ ] TurtleBot3 Waffle 자율주행 연동
- [ ] 야외 쓰레기 데이터셋 추가 수집 및 재학습
- [ ] ROS2 노드로 코드 리팩토링 (클래스화)
- [ ] 전체 시스템 통합 테스트 (탑카메라 → 이동 → D405 정밀 집기)

---

## 데이터셋 정보

- 위치: `src/garbage_detection/data/`
- Roboflow 프로젝트: garbage-detection-6oh6h-wfqa6
- workspace: soarm101-hqdge
- 클래스: bottle, bottle-cap, box, glass

## 환경 정보

- OS: Ubuntu 24.04 LTS
- ROS2: Jazzy
- Python: 3.12 (lerobot conda 환경)
- GPU: NVIDIA GeForce RTX 2080 Ti
- 로봇 포트: `/dev/ttyACM0` or `/dev/ttyACM1`
- 카메라: Intel RealSense D405 (`/dev/video2`, `/dev/video4`)
