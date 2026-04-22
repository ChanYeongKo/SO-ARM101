# SO-ARM101 Isaac Sim 실시간 미러링

## 목표

Ubuntu 24.04에서 실제 SO-ARM101 로봇이 움직이면,  
Windows의 Isaac Sim 안에 있는 가상 로봇이 **실시간으로 동일하게** 움직이도록 한다.

---

## 현재 상태

| 항목 | 상태 |
|---|---|
| Isaac Sim 가상환경 구축 | ✅ 완료 |
| 링크별 STL 메시 로드 | ✅ 완료 |
| 관절 DriveAPI 연결 | ✅ 완료 |
| 단독 테스트 (로봇 없이) | ✅ 완료 (`so_arm_test.py`) |
| Ubuntu → Windows UDP 스트리밍 | ✅ 완료 |
| pick_and_dump 통합 | ✅ 완료 |
| **실제 로봇 연결 후 미러링 테스트** | 🔲 미완료 |

---

## 전체 구조

```
[Ubuntu 24.04 - 실제 로봇]              [Windows - Isaac Sim]

pick_and_dump.py
  ├── Phase 1: 검출 + 파지
  ├── Phase 2: 웨이포인트 이동
  ├── Phase 3: 비우기
  ├── Phase 4: 복귀
  └── start_isaac_streamer()             so_arm_visualizer.py
        robot.get_observation()               UDPReceiver (port 5005)
        → 6개 관절각 [deg]                        ↓
        → JSON 패킷          ─── UDP ──►    DriveAPI.SetTarget(deg)
          30Hz, ~150 bytes    192.168.0.47       ↓
                                           가상 로봇 관절 이동
```

---

## 파일 구성

| 파일 | 실행 위치 | 역할 |
|---|---|---|
| `so_arm_test.py` | Windows | 로봇/Ubuntu 없이 Isaac Sim 단독 동작 확인. 자동 애니메이션으로 관절 정상 작동 검증 |
| `so_arm_visualizer.py` | Windows | UDP 수신 대기 → 받은 관절각으로 가상 로봇 구동 |
| `joint_streamer.py` | Ubuntu | (단독 테스트용) 실제 로봇 또는 sin파 신호를 Isaac Sim으로 전송 |
| `run_test.bat` | Windows | `so_arm_test.py` 실행 배치파일 |

> `pick_and_dump.py`에 `start_isaac_streamer()` 함수가 내장되어 있어  
> `joint_streamer.py`와 동시 실행 없이 자동으로 스트리밍됩니다.

---

## 3D 모델 구성

- **URDF**: `SO-ARM100/Simulation/SO101/so101_new_calib.urdf`
  - onshape-to-robot으로 생성된 완전한 URDF
  - 링크별 개별 STL 메시 포함 (노란색 3D 프린트 부품 + 검정 서보모터)
  - 정확한 origin/rpy가 각 링크에 이미 정의되어 있음

- **STL 부품들**: `SO-ARM100/Simulation/SO101/assets/`

| STL 파일 | URDF 링크 | 설명 |
|---|---|---|
| `base_so101_v2.stl` | `base_link` | 베이스 플레이트 |
| `base_motor_holder_so101_v1.stl` | `base_link` | 베이스 모터 홀더 |
| `rotation_pitch_so101_v1.stl` | `shoulder_link` | 숄더 회전 부품 |
| `motor_holder_so101_base_v1.stl` | `shoulder_link` | 숄더 모터 홀더 |
| `upper_arm_so101_v1.stl` | `upper_arm_link` | 상완 |
| `under_arm_so101_v1.stl` | `lower_arm_link` | 하완 |
| `motor_holder_so101_wrist_v1.stl` | `lower_arm_link` | 손목 모터 홀더 |
| `wrist_roll_pitch_so101_v2.stl` | `wrist_link` | 손목 롤/피치 부품 |
| `wrist_roll_follower_so101_v1.stl` | `gripper_link` | 손목 롤 팔로워 |
| `moving_jaw_so101_v1.stl` | `moving_jaw_so101_v1_link` | 그리퍼 가동 조 |
| `sts3215_03a_v1.stl` | 각 링크 | Feetech STS3215 서보모터 |

---

## 관절 매핑

| 관절 이름 | 실제 서보 | 전송값 단위 | Isaac Sim 적용 |
|---|---|---|---|
| `shoulder_pan` | motor1 | degrees | DriveAPI angular target |
| `shoulder_lift` | motor2 | degrees | DriveAPI angular target |
| `elbow_flex` | motor3 | degrees | DriveAPI angular target |
| `wrist_flex` | motor4 | degrees | DriveAPI angular target |
| `wrist_roll` | motor5 | degrees | DriveAPI angular target |
| `gripper` | motor6 | degrees | DriveAPI angular target |

---

## 실행 방법

### Step 1: Windows 방화벽 (최초 1회)

관리자 권한 CMD:
```bat
netsh advfirewall firewall add rule name="IsaacSimUDP" protocol=UDP dir=in localport=5005 action=allow
```

### Step 2: Windows — Isaac Sim 시작

```bat
cd D:\isaac-sim
python.bat D:\isaac-sim\so-arm101\SO-ARM101\project\isaac_sim\so_arm_visualizer.py
```

로그에 아래가 뜨면 대기 상태:
```
6개 관절 DriveAPI 모두 확인됨 ✓
준비 완료. Ubuntu에서 pick_and_dump.py를 실행하면 미러링이 시작됩니다.
```

### Step 3: Ubuntu — 로봇 실행

```bash
conda activate lerobot
cd ~/lerobot2/project
python pick_and_dump/pick_and_dump.py
```

로봇이 연결되는 순간부터 Isaac Sim이 실시간으로 따라 움직입니다.

---

## 단독 테스트 (Ubuntu/로봇 없이)

Isaac Sim 가상환경만 확인하고 싶을 때:

```bat
cd D:\isaac-sim
python.bat D:\isaac-sim\so-arm101\SO-ARM101\project\isaac_sim\so_arm_test.py
```

자동으로 `HOME → SAFE_POSE → NEUTRAL → SIN파` 순서로 관절을 움직여  
링크별 STL 메시가 올바르게 애니메이션되는지 확인합니다.

연결 테스트 (Ubuntu에서 sin파 신호만 전송):
```bash
python isaac_sim/joint_streamer.py --target-ip 192.168.0.47 --no-robot
```

---

## 네트워크 설정

| 항목 | 값 |
|---|---|
| Windows IP | `192.168.0.47` (이더넷) |
| UDP 포트 | `5005` |
| 전송 주파수 | `30Hz` |
| 패킷 크기 | `~150 bytes` |

Ubuntu와 Windows가 **같은 공유기(192.168.0.x 대역)** 에 연결되어 있어야 합니다.

---

## 기술 상세

### Isaac Sim 임포트 패턴

```python
# ImportConfig에 드라이브 타입 설정 금지 (TypeError 발생)
_, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.distance_scale = 100   # URDF[m] → stage[cm]
cfg.fix_base = True

ok, stage_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=...,
    import_config=cfg,
    get_articulation_root=True,
)
kit.update()  # DriveAPI 프림 등록 대기 필수
```

### 관절 제어 방식

```python
# URDF 임포트 후 DriveAPI를 직접 탐색해서 stiffness 설정
drive = UsdPhysics.DriveAPI.Get(prim, "angular")
drive.GetStiffnessAttr().Set(1e8)
drive.GetDampingAttr().Set(1e6)

# 매 프레임마다 목표각 설정 (단위: degrees)
drive.GetTargetPositionAttr().Set(target_deg)
```

### pick_and_dump 통합 방식

```python
# pick_and_dump.py의 follower.connect() 직후 호출
start_isaac_streamer(follower)

# 내부적으로 daemon 스레드로 동작 (메인 로직에 영향 없음)
def _loop():
    obs = robot.get_observation()
    joints = {n: float(obs[f"{n}.pos"]) for n in JOINT_NAMES}
    sock.sendto(json.dumps(joints).encode(), (ISAAC_SIM_IP, 5005))
    time.sleep(1/30)
```

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `TypeError: UrdfJointTargetType` | `cfg.default_drive_type = 1` 정수 사용 | ImportConfig에서 drive 설정 제거, DriveAPI로 직접 설정 |
| 드라이브 1개만 발견 | articulation_root(`/base_link`) 하위만 탐색 | 로봇 최상위 네임스페이스(`/so101_new_calib/`) 전체 탐색으로 변경 |
| 카메라 xformOp 오류 | 이미 존재하는 op에 `Add` 재호출 | 기존 op `Get` 후 `Set` 사용 |
| UDP 데이터 없음 경고 | 방화벽 또는 네트워크 문제 | 방화벽 규칙 확인, 동일 대역 연결 확인 |
| 로봇이 흰색으로 보임 | `so101_visual.urdf` (실린더/구체) 사용 | `SO-ARM100/Simulation/SO101/so101_new_calib.urdf` 사용 |
