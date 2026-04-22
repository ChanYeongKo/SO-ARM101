# Isaac Sim SO-ARM101 가상환경 + 실시간 미러링

Ubuntu 24.04의 실제 SO-ARM101 로봇이 움직이면, Windows의 Isaac Sim 안에 있는 가상 로봇이 **실시간으로 동일하게** 움직이도록 하는 시스템입니다.

---

## 파일 구성

| 파일 | 역할 |
|------|------|
| `so_arm_test.py` | **독립 테스트** - Ubuntu/로봇 없이 Isaac Sim만으로 동작 확인 |
| `add_visuals_to_urdf.py` | 원본 URDF에 시각화 geometry 추가 → `so101_visual.urdf` 생성 |
| `joint_streamer.py` | Ubuntu에서 실행 - 관절각 UDP 전송 |
| `so_arm_visualizer.py` | Windows Isaac Sim에서 실행 - UDP 수신 + 가상 로봇 구동 |
| `run_test.bat` | `so_arm_test.py` 실행 편의 배치 파일 |

---

## Step 1: 가상환경 단독 테스트 (Ubuntu/로봇 불필요)

```bat
REM 방법 1: 배치 파일 더블클릭
project\isaac_sim\run_test.bat

REM 방법 2: CMD에서 직접
cd D:\isaac-sim
python.bat D:\isaac-sim\so-arm101\SO-ARM101\project\isaac_sim\so_arm_test.py
```

**테스트 동작 순서:**
1. `so101_visual.urdf` 없으면 자동 생성 (약 1초)
2. `SO101 Assembly.stl` → USD 변환 후 반투명 ghost로 배치 (STL이 있을 때)
3. Isaac Sim 창 열림 → 가상 로봇 로드
4. 관절 애니메이션: `HOME(2초) → SAFE_POSE(3초) → NEUTRAL(3초) → SIN파 반복`
5. 콘솔에 60프레임마다 현재 관절각 출력

**정상 동작 확인:**
```
[검증 통과] 6개 관절 모두 확인됨
[F00060] pan=-8.3° lift=-65.2° elbow=22.1° wrist_f=67.0° wrist_r=-0.3° gripper=95.0°
```

---

## SO101 Assembly.stl 활용

현재 repo에 있는 `SO101 Assembly.stl`은 **전체 조립체 단일 메시**입니다.

| 용도 | 가능 여부 |
|------|---------|
| 개별 관절 애니메이션 | ❌ (단일 메시라 분리 불가) |
| 정적 레퍼런스 / Ghost | ✅ 반투명 25%로 배치 |
| 실제 로봇 형태 확인 | ✅ |

> 개별 링크 STL이 필요한 경우: CAD 소프트웨어(OnShape/Fusion360)에서  
> 각 링크를 별도로 내보내야 합니다. 현재는 geometric primitive로 대체합니다.

---

## 전체 구조 (실시간 미러링)

```
[Ubuntu 24.04]                       [Windows - Isaac Sim]
┌──────────────────────────┐         ┌──────────────────────────────┐
│  SO-ARM101 (실제 로봇)    │         │  Isaac Sim 2023.1.1           │
│  Feetech STS3215 서보     │         │                              │
│         ↓ USB             │         │  so101_visual.urdf 임포트    │
│  LeRobot get_observation()│         │  (cylinder + sphere 시각화)  │
│         ↓ 관절각 [deg]    │  UDP    │         ↓                    │
│  joint_streamer.py ───────┼────────►│  so_arm_visualizer.py        │
│  (30Hz, JSON 패킷)        │  5005   │  set_joint_positions()       │
└──────────────────────────┘  포트    └──────────────────────────────┘
```

**통신 방식**: UDP (30Hz, JSON 패킷 ~150 bytes)  
이유: 설치 불필요 (Python 내장), 낮은 레이턴시, ROS2 버전 호환 문제 없음

---

## Step 2: 실시간 미러링 실행 순서

### Step 2-0: 한 번만 - 시각용 URDF 생성

원본 URDF(`so101_new_calib.urdf`)는 FK/IK 전용으로 시각적 메시가 없습니다.
Isaac Sim에서 로봇이 보이려면 시각화 geometry를 추가해야 합니다.

**Ubuntu에서:**
```bash
conda activate lerobot
cd ~/lerobot2/project
python isaac_sim/add_visuals_to_urdf.py
# → project/so101_visual.urdf 생성됨
```

`so101_visual.urdf`는 각 링크에 아래 geometry를 추가합니다:
- `base_link` → 회색 박스
- `shoulder_link` → 회색 구체 + 실린더
- `upper_arm_link` / `lower_arm_link` → 파란 실린더 (팔 방향 정렬)
- `wrist_link` → 초록 실린더
- `gripper_link` → 주황 박스
- `moving_jaw` → 주황 박스 (그리퍼 조)

git push 후 Windows에서 git pull 하거나, 파일을 직접 복사하면 됩니다.

---

### Step 2-1: Windows - 방화벽 열기 (관리자 권한 CMD)

```bat
netsh advfirewall firewall add rule name="IsaacSimUDP" protocol=UDP dir=in localport=5005 action=allow
```

---

### Step 2-2: Windows - Isaac Sim 시각화기 실행

```bat
cd D:\isaac-sim
python.bat D:\isaac-sim\so-arm101\SO-ARM101\project\isaac_sim\so_arm_visualizer.py
```

Isaac Sim 창이 열리고 가상 로봇이 로드됩니다.  
"Ubuntu에서 joint_streamer.py를 실행하세요" 메시지가 로그에 뜨면 준비 완료.

---

### Step 2-3: Ubuntu - 스트리머 실행

**IP 확인 (Windows):**
```bat
ipconfig
# IPv4 주소 예: 192.168.1.100
```

**Ubuntu에서:**
```bash
conda activate lerobot
cd ~/lerobot2/project

# 실제 로봇 연결 후 스트리밍
python isaac_sim/joint_streamer.py --target-ip 192.168.1.100

# 로봇 없이 sin파 테스트 (Isaac Sim 연결 확인용)
python isaac_sim/joint_streamer.py --target-ip 192.168.1.100 --no-robot
```

Ubuntu와 Windows가 같은 Wi-Fi 또는 유선 네트워크에 연결되어 있어야 합니다.

---

### Step 2-4: 동작 확인

Isaac Sim 로그에서:
```
[Info] 감지된 DOF (6개): ['shoulder_pan', 'shoulder_lift', ...]
```
가 뜨면 관절 매핑 완료. 로봇을 움직이면 Isaac Sim에서 동시에 움직입니다.

---

## 관절 매핑

| LeRobot 키 | URDF joint | 단위 변환 |
|-----------|-----------|---------|
| `shoulder_pan.pos` | `shoulder_pan` | deg → rad |
| `shoulder_lift.pos` | `shoulder_lift` | deg → rad |
| `elbow_flex.pos` | `elbow_flex` | deg → rad |
| `wrist_flex.pos` | `wrist_flex` | deg → rad |
| `wrist_roll.pos` | `wrist_roll` | deg → rad |
| `gripper.pos` | `gripper` | deg → rad |

그리퍼: LeRobot `100°(열림) ~ -10°(닫힘)` ↔ URDF `1.745 rad ~ -0.175 rad` (동일 스케일)

---

## 기술 상세

### UDP 패킷 형식

```json
{"shoulder_pan": -13.5, "shoulder_lift": -30.2, "elbow_flex": 45.1,
 "wrist_flex": -20.0, "wrist_roll": 0.5, "gripper": 95.0}
```

- 30Hz × 6관절 × float → 초당 ~4.5 KB
- UDP: 패킷 손실 시 이전 값 유지 (시각화에 충분)

### Isaac Sim에서의 Kinematic 미러링

```python
robot.set_joint_positions(target_pos)   # 물리 없이 직접 위치 설정
robot.set_joint_velocities(np.zeros(n)) # 속도를 0으로 고정
```

- `fix_base=True`: 베이스는 바닥에 고정
- `default_drive_strength=1e7`: 높은 드라이브 강성 → 빠른 추종
- 물리 시뮬레이션이 아닌 **kinematic replay** 방식

### URDF 관절 한계 (rad)

| 관절 | 하한 | 상한 |
|------|------|------|
| shoulder_pan | -1.920 (-110°) | 1.920 (110°) |
| shoulder_lift | -1.745 (-100°) | 1.745 (100°) |
| elbow_flex | -1.690 (-97°) | 1.690 (97°) |
| wrist_flex | -1.658 (-95°) | 1.658 (95°) |
| wrist_roll | -2.744 (-157°) | 2.841 (163°) |
| gripper | -0.175 (-10°) | 1.745 (100°) |

---

## 수정 이력

### v2 (2026-04-22) - URDF 임포트 패턴 수정
**오류**: `Prim path /so101_new_calib is invalid`  
**원인**: `_urdf.ImportConfig()` 직접 생성 + `World` 클래스 사용 → prim이 생성되지 않음  
**해결**: 공식 Isaac Sim 예제(`standalone_examples/api/omni.importer.urdf/urdf_import.py`) 패턴 적용

| 항목 | 이전 (오류) | 이후 (정상) |
|------|------------|------------|
| Config 생성 | `_urdf.ImportConfig()` | `URDFCreateImportConfig` 커맨드 |
| Import 옵션 | 없음 | `get_articulation_root=True` |
| Physics 설정 | `World` 클래스 | `UsdPhysics.Scene` 직접 정의 |
| 시뮬레이션 시작 | `world.reset()` | `timeline.play()` + `kit.update()` |
| Articulation | `world.scene.add(Articulation(...))` | `Articulation(...)` + `.initialize()` |
| 루프 | `world.step(render=True)` | `kit.update()` |

---

## 트러블슈팅

### Isaac Sim에서 로봇이 안 보임
- `add_visuals_to_urdf.py`를 실행해서 `so101_visual.urdf`를 생성했는지 확인
- 또는 Isaac Sim 뷰포트에서 카메라를 조정 (숫자패드 `7`: top view, `1`: front view)

### DOF 이름이 예상과 다름
Isaac Sim 로그에서 감지된 DOF 이름을 확인 후, `so_arm_visualizer.py`의 `JOINT_NAMES` 리스트를 맞춰주세요.

### UDP 데이터가 안 들어옴
```bash
# Ubuntu에서 확인
python isaac_sim/joint_streamer.py --target-ip <IP> --no-robot
```
Windows 방화벽에서 UDP 5005 포트가 허용되어 있는지 재확인.

### "URDF 임포트 실패" 오류
`omni.importer.urdf` 익스텐션이 Isaac Sim에 활성화되어 있어야 합니다.  
Isaac Sim 메뉴: Window → Extensions → "URDF Importer" 검색 → Enable

### 로봇이 Isaac Sim에서 특이한 자세를 취함
LeRobot과 URDF의 관절 0° 기준이 다를 수 있습니다.  
각 관절에 오프셋을 적용하려면 `so_arm_visualizer.py`에서:
```python
# name_to_idx 매핑 후 아래 오프셋 추가
JOINT_OFFSETS_RAD = {
    "shoulder_lift": np.deg2rad(90),  # 예시
}
```

---

## 향후 확장

- **pick_and_dump 연동**: `pick_and_dump.py`의 로봇 제어 루프에 `joint_streamer`를 쓰레드로 삽입하면 자율 동작 중 실시간 시각화 가능
- **양방향**: Isaac Sim에서 목표 자세를 설정하고 Ubuntu로 보내면 실제 로봇 원격 제어 가능
- **RealSense 영상 오버레이**: Isaac Sim의 카메라 피드와 실제 D405 영상을 나란히 표시
