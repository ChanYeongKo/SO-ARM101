# SO-ARM101 Ubuntu 작업 가이드

이 문서는 SO-ARM101 로봇 팔 프로젝트의 우분투 환경 설정 및 실행 가이드입니다.

---

## 프로젝트 개요

**목표:** 쓰레기 수거차에 부착된 SO-ARM101 로봇 팔이 6cm 너비 쓰레기통을 자율적으로 집어 비우는 시스템

**파이프라인:**
```
D405 eye-in-hand 카메라 → YOLOv8 쓰레기통 감지 → Visual Servoing 정렬
→ Depth 3D 좌표 → IKPy 역기구학 → hover → pick → wrist_roll 뒤집기 → drop
```

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

### 2. YOLOv8 모델 학습

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

> **현재 상태:** Roboflow 인터넷 데이터셋으로 학습된 임시 모델.
> 쓰레기통(trash_bin) 단일 클래스로 재학습 예정 (Isaac Sim 합성 데이터 + 실제 카메라 데이터).

---

### 3. 로봇 캘리브레이션

캘리브레이션 파일 저장 위치:
- Follower: `~/.cache/huggingface/lerobot/calibration/robots/so_follower/`
- Leader: `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/`

> Leader 팔 불필요 (프로젝트에서 Follower 팔만 사용)

---

### 4. ROS2 Jazzy 설치

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

### 5. Intel RealSense D405 연결

**설치:**
```bash
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**주의사항:**
- D405는 640x480 미지원 → **848x480** 사용
- USB 허브 없이 PC에 직접 연결 권장
- pyrealsense2는 lerobot conda 환경에 이미 설치됨 (v2.56.5)

---

### 6. URDF 파일 (`src/so101.urdf`)

공식 SO-101 URDF 기반 + D405 카메라 조인트 추가.

**관절 정보:**
| 관절 | 범위 |
|------|------|
| shoulder_pan | -1.92 ~ +1.92 rad |
| shoulder_lift | -1.75 ~ +1.75 rad |
| elbow_flex | -1.69 ~ +1.69 rad |
| wrist_flex | -1.66 ~ +1.66 rad |
| wrist_roll | -2.74 ~ +2.84 rad |

**D405 카메라 조인트 (gripper_to_d405):**
```xml
<joint name="gripper_to_d405" type="fixed">
  <parent link="gripper_link"/>
  <child link="d405_link"/>
  <origin xyz="0.08 0.0 0.05" rpy="0 0.698 0"/>
</joint>
```
- 그리퍼 기준: 앞 8cm, 위 5cm, 아래로 40도 기울어짐 (0.698 rad)

> **주의:** IKPy 체인 충돌 방지를 위해 gripper revolute 조인트는 URDF에서 제외.
> IKPy 체인 끝은 gripper_frame_link (fixed)로 종료.
> active_links_mask = [False, True, True, True, True, True, False]

---

### 7. Pick & Place 통합 코드 (`src/pick_and_place.py`)

**파이프라인:**
```
D405 카메라 (848x480)
    ↓
YOLOv8 감지 (화면 상단 60%만 사용, 하단 그리퍼 영역 제외)
    ↓
Visual Servoing (shoulder_pan/lift로 물체 화면 중앙 정렬)
    ↓
Depth → 3D 좌표 (rs2_deproject_pixel_to_point)
    ↓
Eye-in-Hand 변환: cam_to_base() — FK + URDF 기반 동적 좌표 변환
    ↓
IKPy 역기구학 → hover → pick → gripper close → lift → drop
```

**실행:**
```bash
conda activate lerobot
python src/pick_and_place.py
```

**주요 설정값:**
```python
ROBOT_PORT   = "/dev/ttyACM0"
CONF_THRESH  = 0.25
ROI_BOTTOM   = 0.60       # 화면 상단 60%만 감지
CAM_X        = 0.08       # gripper 앞 8cm (URDF 기반)
CAM_Z        = 0.05       # gripper 위 5cm (URDF 기반)
CAM_TILT     = 0.698      # 40도 아래 기울어짐 (URDF rpy Y축)
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE= 80.0
```

**cam_to_base() — Eye-in-Hand 좌표 변환:**
```python
# URDF gripper_to_d405: xyz=[0.08, 0.0, 0.05], rpy=[0, 0.698, 0]
# Y축 회전(pitch) 행렬 적용
R_tilt = np.array([
    [ cos(CAM_TILT), 0, sin(CAM_TILT)],
    [ 0,             1, 0            ],
    [-sin(CAM_TILT), 0, cos(CAM_TILT)],
])
T_wrist_to_cam[:3,:3] = R_tilt
T_wrist_to_cam[:3, 3] = [CAM_X, 0, CAM_Z]
T_base_to_cam = T_wrist @ T_wrist_to_cam
```

**DROP 위치:** 현재 임시값 → 실측 후 수정 필요
```python
DROP = {
    "shoulder_pan.pos":  45.0,
    "shoulder_lift.pos": -20.0,
    "elbow_flex.pos":    30.0,
    "wrist_flex.pos":    0.0,
    "wrist_roll.pos":    0.0,
}
```

---

### 8. 디버그 카메라 스크립트 (`src/debug_camera.py`)

```bash
conda activate lerobot
python src/debug_camera.py
```

- 실시간 카메라 영상 + YOLO 감지 결과 오버레이 표시
- `s` 키: 현재 프레임 저장 (`src/debug_frame_XXX.jpg`)
- `q` 키: 종료
- 노란 선: ROI 경계선

---

### 9. 시뮬레이션 뷰어 (`src/simulation.py`)

```bash
conda activate lerobot
python src/simulation.py
```

- matplotlib 기반 실시간 3D 로봇 시각화
- 6개 모터 bar chart 표시 (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper)
- IKPy URDF 기반 FK로 링크 위치 계산
- 슬라이더로 목표 위치(x, y, z) 조정 및 IK 솔루션 확인

---

### 10. Isaac Sim 합성 데이터 생성 파이프라인

**목적:** 실제 촬영 데이터 부족을 Isaac Sim 합성 데이터로 보완 → YOLOv8 trash_bin 재학습

**환경 설정:**
```bash
# Python 3.10 conda 환경 (Isaac Sim 4.5 요구사항)
conda create -n isaacsim python=3.10 -y
conda activate isaacsim

# Isaac Sim 설치 (pip, ~20GB)
pip install isaacsim==4.5.0 --extra-index-url https://pypi.nvidia.com

# 최초 실행 시 EULA 동의 필요
isaacsim  # "Yes" 입력
```

**파일 위치:** `/home/chan/issac_sim/`
- `generate_data.py` — 합성 이미지 1000장 생성
- `preview.py` — 실시간 카메라 미리보기 (키보드 조작)
- `convert_to_yolo.py` — Isaac Sim output → YOLO format 변환

**generate_data.py 실행:**
```bash
conda activate isaacsim
cd /home/chan/issac_sim
python generate_data.py
# 출력: /home/chan/issac_sim/output/ (rgb + bounding_box npy)
```

**카메라 설정 (D405 eye-in-hand 실측값):**
```python
camera = rep.create.camera(
    position=(0.0, 0.0, 0.22),   # 바닥에서 22cm
    rotation=(-40, 0, 0),         # 40도 아래 기울어짐
    focal_length=1.93,            # D405 근사값
)
```

**랜덤화 항목 (Domain Randomization):**
- 쓰레기통 위치: 전방 8~28cm, 좌우 ±12cm
- 쓰레기통 회전: 0~360도
- 조명 강도: 1000~5000
- 바닥 색상: 0.3~0.9 (회색 계열)

**convert_to_yolo.py 실행:**
```bash
conda activate isaacsim
python /home/chan/issac_sim/convert_to_yolo.py
# 출력: /home/chan/issac_sim/yolo_dataset/ (train 80% / val 20%)
```

**preview.py — 실시간 미리보기:**
```bash
conda activate isaacsim
python /home/chan/issac_sim/preview.py
```
| 키 | 동작 |
|----|------|
| W/S | 카메라 높이 ↑/↓ |
| A/D | 카메라 좌우 |
| Q/E | 카메라 앞뒤 |
| ↑/↓ | 카메라 pitch |
| [/] | 조명 밝기 ↓/↑ |
| Space | 현재 설정값 출력 |
| ESC | 종료 |

**알려진 문제 및 해결책:**
- 검은 화면 → 렌더링 워밍업 프레임 추가 (현재 20 frames), RTSubframes 설정
- Illegal cycle 오류 → `rep.randomizer.register()` 방식 사용 (직접 color() 호출 금지)

---

## 남은 작업

### 1단계: 데이터 수집 및 모델 재학습
- [ ] Isaac Sim `generate_data.py` 실행 → 합성 이미지 1000장 생성 확인
- [ ] `convert_to_yolo.py`로 YOLO 포맷 변환
- [ ] 실제 D405 카메라로 쓰레기통 촬영 (100장 이상)
  - `debug_camera.py`에서 `s` 키로 촬영
  - Roboflow 업로드 → trash_bin 클래스 라벨링 → export
- [ ] YOLOv8 trash_bin 단일 클래스로 재학습 (합성 + 실제 데이터 혼합)

### 2단계: 동작 완성
- [ ] 6cm 쓰레기통 실물 제작 (원통형)
- [ ] DROP 위치 실측 및 코드 반영
- [ ] wrist_roll 뒤집기 모션 구현 (pick_and_place.py에 추가)
  - 쓰레기통 집은 후 wrist_roll ±163도 회전으로 내용물 비우기
- [ ] Pick & Place 전체 동작 테스트

### 3단계: 자율주행 통합 (2학기)
- [ ] 탑 카메라 장착 (차량 상단 RGB 카메라)
- [ ] TurtleBot3 Waffle 자율주행 연동
- [ ] ROS2 노드로 코드 리팩토링
- [ ] 전체 시스템 통합 테스트

---

## 데이터셋 정보

**기존 (임시):**
- 위치: `src/garbage_detection/data/`
- Roboflow 프로젝트: garbage-detection-6oh6h-wfqa6
- 클래스: bottle, bottle-cap, box, glass

**목표:**
- 클래스: trash_bin (단일)
- 합성 데이터: Isaac Sim 1000장
- 실제 데이터: D405 카메라 100장+

---

## 환경 정보

| 항목 | 값 |
|------|-----|
| OS | Ubuntu 24.04 LTS |
| ROS2 | Jazzy |
| Python (lerobot) | 3.12 |
| Python (isaacsim) | 3.10 |
| GPU | NVIDIA GeForce RTX 2080 Ti |
| 로봇 포트 | `/dev/ttyACM0` or `/dev/ttyACM1` |
| 카메라 | Intel RealSense D405 |
| Isaac Sim | 4.5.0 (설치 위치: conda isaacsim 환경) |
