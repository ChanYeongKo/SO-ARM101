# SO-ARM101 Ubuntu 작업 가이드

## 현재 상태
- ✅ LeRobot 설치 및 conda 환경 (lerobot) 구성 완료
- ✅ SO-ARM101 모터 등록 완료
- ✅ Roboflow 데이터셋 다운로드 완료 (`src/garbage_detection/data/`)
- ⬜ data.yaml 경로 수정
- ⬜ YOLOv8 학습
- ⬜ SO-ARM101 캘리브레이션
- ⬜ 텔레오퍼레이션 확인
- ⬜ D405 연결 확인

---

## STEP 1 — data.yaml 경로 수정

```bash
cd ~/SO-ARM101/src/garbage_detection/data
sed -i 's|../train|train|g; s|../valid|valid|g; s|../test|test|g' data.yaml
cat data.yaml   # 확인
```

정상이면 이렇게 보여야 함:
```
train: train/images
val: valid/images
test: test/images
```

---

## STEP 2 — ultralytics 설치

```bash
conda activate lerobot
pip install ultralytics
```

GPU 확인:
```bash
python -c "import torch; print(torch.cuda.is_available())"
# True 가 나와야 GPU 학습 가능
```

---

## STEP 3 — YOLOv8 학습

```bash
conda activate lerobot
cd ~/SO-ARM101
python src/garbage_detection/scripts/train.py
```

옵션 (필요 시):
```bash
# 모델 크기 변경 (기본 yolov8n = 가장 빠름)
python src/garbage_detection/scripts/train.py --model yolov8s.pt

# GPU 메모리 부족 시 배치 줄이기
python src/garbage_detection/scripts/train.py --batch 8

# CPU로 학습 (느림, GPU 없을 때만)
python src/garbage_detection/scripts/train.py --device cpu
```

학습 완료 후 모델 위치:
```
src/garbage_detection/models/garbage_detect/weights/best.pt
```

---

## STEP 4 — 여러 모델 비교 학습 (선택)

같은 데이터로 다른 모델들 비교:
```bash
python src/garbage_detection/scripts/train.py --model yolov8n.pt  --epochs 50
python src/garbage_detection/scripts/train.py --model yolov9c.pt  --epochs 50
python src/garbage_detection/scripts/train.py --model yolo11n.pt  --epochs 50
```

결과 비교:
```
src/garbage_detection/models/garbage_detect/results.png  ← 학습 그래프
```

---

## STEP 5 — SO-ARM101 캘리브레이션

```bash
conda activate lerobot
cd ~/lerobot

# Follower 캘리브레이션
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=my_follower

# Leader 캘리브레이션
lerobot-calibrate \
  --robot.type=so101_leader \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_leader
```

포트 확인 방법:
```bash
ls /dev/ttyACM*   # 연결 전후 비교해서 포트 확인
```

---

## STEP 6 — 텔레오퍼레이션 확인

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=my_follower \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.id=my_leader \
  --display_data=true \
  --robot.cameras="{wrist:{type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}"
```

리더 팔 움직이면 팔로워 팔이 따라오면 정상

---

## STEP 7 — D405 연결 확인

```bash
# realsense SDK 설치 확인
realsense-viewer

# ROS2 토픽 확인
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=1280x720x30 \
    pointcloud.enable:=true \
    align_depth.enable:=true

ros2 topic list | grep camera
```

---

## 데이터셋 정보
- 클래스 4개: bottle, bottle-cap, box, glass
- 위치: `src/garbage_detection/data/`
- Roboflow 프로젝트: garbage-detection-6oh6h-wfqa6 (workspace: soarm101-hqdge)
