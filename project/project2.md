# Project 2: SO-ARM101 쓰레기통 파지 시스템 — 진행 보고서

## 1. 프로젝트 개요

SO-ARM101 5DOF 로봇암 2대(리더/팔로워)와 Intel RealSense D405 카메라를 사용하여,
커스텀 YOLO 모델로 종이컵 모양 쓰레기통을 검출하고 자동으로 파지하는 **Classical 방식** 시스템.

| 항목 | 내용 |
|------|------|
| 로봇 | SO-ARM101 x 2 (리더 ACM0 / 팔로워 ACM1) |
| 카메라 | Intel RealSense D405 (eye-in-hand, 그리퍼 장착) |
| 검출 | YOLOv8 커스텀 모델 (`best.pt`) |
| 운동학 | placo 기반 FK/IK (lerobot v0.5.2 내장) |
| 캘리브레이션 | ChArUco 보드 + OpenCV Hand-Eye (TSAI) |
| OS | Ubuntu 24.04, Python 3.12+ |

---

## 2. 현재 작업 단계

### 현재: 모션 시퀀스 최적화 (파지 성공률 향상)

지금 하고 있는 작업은 **로봇이 물체를 안정적으로 파지하기 위한 모션 시퀀스 최적화**입니다.

검출 → 좌표 변환 → IK까지의 파이프라인은 완성되었고, 실제 로봇이 물체를 잡으러 이동할 때
**카메라 충돌 방지**, **그리퍼 각도 유지**, **파지 힘 조절** 등의 실전 문제를 해결하는 중입니다.

**2026-04-16: 첫 파지 성공!** — 쓰레기통 검출 → 파지 → 들어올리기 → 내려놓기 성공

### 다음 단계

| 순서 | 작업 | 설명 |
|------|------|------|
| ~~완료~~ | ~~모션 시퀀스 최적화~~ | ~~팔 세우기→각도 세팅→하강→파지 흐름 튜닝~~ |
| **현재** | 다양한 위치 테스트 | 쓰레기통 위치를 바꿔가며 파지 성공률 측정 |
| 다음 1 | 물체 놓기(Place) | 파지 후 지정 위치에 놓는 동작 추가 |
| 다음 2 | 에러 복구 | IK 실패, 파지 실패 시 재시도 로직 |
| 다음 3 | 자동화 | 사람 개입 없이 검출→파지→놓기 전체 자동화 |
| 다음 4 | (선택) 학습 기반 | Classical 결과를 시드로 강화학습/모방학습 적용 |

---

## 3. 완료된 작업

### 3-1. 환경 구축
- [x] lerobot v0.5.2 설치 (`pip install -e ".[feetech,intelrealsense,kinematics]"`)
- [x] 관절 캘리브레이션 (기존 `~/.cache/huggingface/lerobot/calibration/` 활용)
- [x] URDF 정리: STL 메시 제거 (FK/IK 전용), D405 카메라 마운트 추가
- [x] D405 카메라 연결 확인 (640x480, 30fps, color+depth)

### 3-2. Hand-Eye Calibration
- [x] ChArUco 보드 생성 (`calibration/generate_charuco.py`) — 5x7, DICT_6X6_250
- [x] 보드 크기 실측 반영: square=30mm, marker=22mm
- [x] 데이터 수집 (`calibration/step2_collect_handeye.py`) — 15개+ 자세
- [x] T_cam→gripper 계산 (`calibration/step2_run_handeye.py`) — cv2.calibrateHandEye (TSAI)
- [x] 결과 저장: `calibration/hand_eye_result.npz`

### 3-3. 객체 검출
- [x] YOLO 검출 모듈 (`grasp/detect_target.py`)
- [x] Robust depth 계산 (bbox 중앙 80% 영역 median)
- [x] 실시간 검출 테스트 (`grasp/test_detection.py`) — 검출 정상 확인

### 3-4. 좌표 변환
- [x] Camera → Gripper → Base 변환 파이프라인 (`grasp/coord_transform.py`)
- [x] Hand-Eye 결과 lazy loading

### 3-5. IK 솔버
- [x] Multi-candidate IK 솔버 (`grasp/ik_solver.py`)
- [x] 실측 파지 자세 기반 초기값 후보 11개
- [x] Top-down 자세 선호 정렬 (downward_score)
- [x] Fallback: 자세 무시, 위치만 달성

### 3-6. 파지 컨트롤러
- [x] 텔레오퍼레이션 + 실시간 검출 루프 (`grasp/grasp_controller.py`)
- [x] Pitch sum 보존 알고리즘 (하강 시 그리퍼 각도 유지)
- [x] 선형 보간 이동 (`move_joints_smooth`)
- [x] 들어올리기 동작
- [x] **Upright Preparation 방식** (v3): 팔 세우기→각도 세팅→하강→파지
- [x] **그리퍼 닫기 시 관절 고정**: 현재 관절값을 읽어서 그리퍼만 변경

### 3-7. 해결한 주요 문제들

| 문제 | 원인 | 해결 |
|------|------|------|
| STL 메시 로드 실패 | placo가 URDF의 visual/collision 로드 시도 | URDF에서 visual/collision 블록 제거 |
| IK 100mm+ 오차 | 5DOF로 6DOF 자세 달성 불가 | orientation_weight=0.0 (위치 우선) |
| IK fallback 항상 None | threshold 체크 누락 | fallback 섹션에 threshold 체크 추가 |
| 그리퍼 열림/닫힘 반대 | GRASP_OPEN=0.0이 실제로 닫힘 | GRASP_OPEN=100.0, GRASP_CLOSE=0.0 |
| 카메라가 물체 밀기 (v1) | 수평 이동 시 카메라가 물체 충돌 | 수직 하강 접근 방식으로 변경 |
| 하강 시 wrist_flex 풀림 | IK가 매번 wrist_flex 재계산 | pitch sum 보존 알고리즘 적용 |
| wrist_flex 꺾기 시 물체 충돌 (v2) | 바닥에서 꺾으면 카메라가 물체 침범 | 2cm 후퇴 → 꺾기 → 재전진 |
| 카메라가 물체 밀기 (v2) | 바닥에서 각도 조정 시 카메라 충돌 | **Upright Preparation 방식 (v3)**: 팔을 세워서 각도 미리 세팅 |
| 파지 시 wrist_flex 변경 | 그리퍼 닫기 명령에 관절값 포함 | 현재 관절값을 읽어서 그리퍼만 닫기 |
| 파지 힘 부족 | GRASP_CLOSE=0.0이 최소 닫힘 | GRASP_CLOSE=-10.0으로 강화 |
| 파지 위치 Y축 편차 | GRASP_Y_OFFSET=0.03 (왼쪽 3cm) | GRASP_Y_OFFSET=0.02로 1cm 우측 보정 |
| 하강 IK 실패 (14cm 위에서 파지) | GRASP_Z_LOWER 상대값이 음수 좌표 생성 | **GRASP_Z_TARGET=0.03 절대 높이**로 변경 |

---

## 4. 현재 파라미터

```python
GRASP_OPEN = 100.0       # 그리퍼 열림 (0~100)
GRASP_CLOSE = -10.0      # 그리퍼 닫힘 (음수=더 강하게)
GRASP_Y_OFFSET = 0.02    # Y축 왼쪽 오프셋 [m]
GRASP_X_OFFSET = 0.04    # X축 전진 오프셋 [m]
WRIST_FLEX_EXTRA = -15    # wrist_flex 추가 보정 [deg] (음수=위로)
GRASP_Z_TARGET = 0.03    # 파지 시 목표 Z 높이 [m] (베이스 기준 절대값)
MAX_REL_TARGET = 5.0     # 한 스텝 최대 이동량 [deg]
```

실측 파지 자세:
```
v3 (성공): shoulder_pan=-11, shoulder_lift=3, elbow_flex=18, wrist_flex=-8, wrist_roll=0
v1 (이전): shoulder_pan=-13, shoulder_lift=44, elbow_flex=58, wrist_flex=-83, wrist_roll=0
```

---

## 5. 모션 시퀀스 변경 이력

### v1: 수평 접근
```
목표 위(+10cm) → 수직 하강 → 전진 → 파지
```
**문제**: 카메라가 물체를 밀어냄

### v2: 바닥에서 각도 조정
```
목표 위(+10cm) → 수직 하강 → 전진 → 후퇴(2cm) → wrist_flex 꺾기 → 재전진 → 파지
```
**문제**: 바닥에서 wrist_flex 꺾을 때 카메라가 물체 충돌

### v3 (현재): Upright Preparation 방식
```
팔 세우기(SL=90°) → 각도 미리 세팅 → 목표 위 이동 → 수직 하강 → 전진 → 파지
```
**장점**: 높은 곳에서 모든 각도를 세팅하므로 카메라 충돌 없음

---

## 6. 파일 구조

```
project/
├── so101_new_calib.urdf          # 로봇 URDF (FK/IK 전용, D405 마운트 포함)
├── best.pt                       # YOLO 커스텀 모델 (쓰레기통 검출)
├── project1.md                   # 초기 설계 문서
├── project2.md                   # 진행 보고서 (이 문서)
├── calibration/
│   ├── generate_charuco.py       # ChArUco 보드 이미지 생성
│   ├── step2_collect_handeye.py  # Hand-Eye 캘리브레이션 데이터 수집
│   ├── step2_run_handeye.py      # T_cam→gripper 계산
│   ├── handeye_data.npz          # 수집된 캘리브레이션 데이터
│   └── hand_eye_result.npz       # 캘리브레이션 결과 (T_cam→gripper)
└── grasp/
    ├── detect_target.py          # YOLO + D405 → 3D 좌표 검출
    ├── coord_transform.py        # Camera → Gripper → Base 좌표 변환
    ├── ik_solver.py              # Multi-candidate IK 솔버
    ├── grasp_controller.py       # 메인 파지 컨트롤러
    └── test_detection.py         # YOLO 검출 테스트 (독립 실행)
```

---

## 7. grasp_controller.py 블록도

### 7-1. 전체 흐름 (v3: Upright Preparation)

```mermaid
flowchart TD
    START([시작]) --> INIT[로봇 / 카메라 / IK 초기화]
    INIT --> WARMUP[카메라 안정화<br/>30프레임 대기]
    WARMUP --> TELEOP_LOOP

    subgraph TELEOP_LOOP [1. 텔레오퍼레이션 + 검출 루프]
        direction TB
        TL1[리더암 get_action] --> TL2[팔로워암 send_action]
        TL2 --> TL3[YOLO 검출 시도]
        TL3 --> TL4{검출 성공?}
        TL4 -- Yes --> TL5[화면에 DETECTED 표시]
        TL4 -- No --> TL6[화면에 NOT DETECTED 표시]
        TL5 --> TL7{키 입력?}
        TL6 --> TL7
        TL7 -- ENTER + 검출됨 --> TL_EXIT([루프 탈출])
        TL7 -- q --> QUIT([종료])
        TL7 -- 그 외 --> TL1
    end

    TELEOP_LOOP --> COORD

    subgraph COORD [2. 좌표 변환]
        direction TB
        C1[카메라 3D 좌표<br/>point_cam] --> C2[관절각 읽기<br/>q_current]
        C2 --> C3[cam_to_base 변환<br/>Camera → Gripper → Base]
        C3 --> C4[Y축 오프셋 적용<br/>+0.02m]
        C4 --> C5[point_base]
    end

    COORD --> PREVIEW

    subgraph PREVIEW [3. 최종 파지 자세 미리 계산]
        direction TB
        PV1[최종 파지 위치 IK<br/>X+4cm, Z-17cm] --> PV2[target_pitch_sum 결정<br/>SL+EF+WF + EXTRA - 10]
    end

    PREVIEW --> MOTION

    subgraph MOTION [4. 파지 모션 시퀀스]
        direction TB
        M1["① 팔 세우기<br/>SL=90°, EF=0°<br/>WF = pitch_sum - 90 - 0"]
        M1 -->|"40 steps"| M2["② 목표 위 이동<br/>Z+10cm<br/>WF = pitch_sum - SL - EF"]
        M2 -->|"30 steps"| M3["③ 수직 하강<br/>Z-17cm<br/>WF = pitch_sum - SL - EF"]
        M3 -->|"25 steps"| M4["④ 전진 4cm<br/>X+4cm<br/>WF = pitch_sum - SL - EF"]
        M4 -->|"15 steps"| M5["⑤ 그리퍼 닫기<br/>관절 고정, 그리퍼만 닫기"]
        M5 -->|"30 steps"| M6["⑥ 들어올리기<br/>Z=0.20m<br/>WF = pitch_sum - SL - EF"]
    end

    MOTION --> DONE([완료])
    QUIT --> CLEANUP[pipeline.stop<br/>disconnect]
    DONE --> CLEANUP
```

### 7-2. Upright Preparation 상세 (v3 핵심)

```mermaid
flowchart TD
    subgraph UPRIGHT ["Upright Preparation: 왜 팔을 세우는가?"]
        direction TB
        PROB["문제: 바닥 근처에서 각도 조정 시<br/>카메라가 물체를 침범"]
        PROB --> SOL["해결: shoulder_lift=90°로 팔을 세워서<br/>카메라를 물체에서 멀리 떨어뜨린 상태에서<br/>모든 각도를 미리 세팅"]
        SOL --> STEP1["1. 팔 세우기<br/>SL=90°, EF=0°<br/>→ 팔이 수직으로 서서 카메라가 높이 위치"]
        STEP1 --> STEP2["2. wrist_flex 세팅<br/>WF = target_pitch_sum - 90 - 0<br/>→ 최종 파지 각도를 미리 적용"]
        STEP2 --> STEP3["3. 하강 시작<br/>IK가 SL, EF를 변경<br/>→ WF를 보정해서 pitch 유지"]
        STEP3 --> STEP4["4. 최종 파지 위치 도달<br/>그리퍼 각도가 처음 세팅한 그대로 유지"]
    end
```

### 7-3. 그리퍼 닫기 시 관절 고정

```mermaid
flowchart LR
    subgraph BEFORE ["이전 방식 (문제)"]
        B1["close_cmd에 IK 결과<br/>관절값 포함"] --> B2["보간 중 관절값 미세 변경"] --> B3["wrist_flex 변화<br/>→ 그리퍼 하늘 향함"]
    end

    subgraph AFTER ["현재 방식 (해결)"]
        A1["get_observation()으로<br/>현재 관절값 실시간 읽기"] --> A2["모든 관절값 그대로 유지<br/>gripper.pos만 변경"] --> A3["관절 움직임 없음<br/>→ 각도 완벽 유지"]
    end
```

### 7-4. IK 솔버 상세 흐름

```mermaid
flowchart TD
    IK_IN([목표 위치 + 현재 관절각]) --> PHASE1

    subgraph PHASE1 [Phase 1: Top-Down 후보 탐색]
        direction TB
        P1_1[11개 실측 기반 초기값 후보]
        P1_1 --> P1_2["각 후보로 IK 시도<br/>orientation_hint = -90도 Y회전<br/>ori_weight = 0.01"]
        P1_2 --> P1_3{오차 ≤ 25mm?}
        P1_3 -- Yes --> P1_4[results에 추가<br/>q, err, downward_score]
        P1_3 -- No --> P1_5[버림]
    end

    PHASE1 --> PHASE2

    subgraph PHASE2 [Phase 2: 현재/홈 자세 기반]
        direction TB
        P2_1["현재 관절각으로 IK<br/>ori_weight = 0.0"]
        P2_1 --> P2_2["홈 자세(0,0,0,0,0)로 IK<br/>ori_weight = 0.0"]
        P2_2 --> P2_3{오차 ≤ 25mm?}
        P2_3 -- Yes --> P2_4[results에 추가]
        P2_3 -- No --> P2_5[버림]
    end

    PHASE2 --> CHECK{results 있음?}

    CHECK -- Yes --> SORT["정렬:<br/>downward_score 높은 순<br/>→ 오차 적은 순"]
    SORT --> BEST[최적 해 반환]

    CHECK -- No --> FALLBACK

    subgraph FALLBACK [Phase 3: Fallback]
        direction TB
        F1["모든 후보 + 현재 + 홈으로<br/>ori_weight = 0.0 재시도"]
        F1 --> F2{최소 오차 ≤ 25mm?}
        F2 -- Yes --> F3[해 반환]
        F2 -- No --> F4[None 반환<br/>IK 실패]
    end
```

### 7-5. 좌표 변환 파이프라인

```mermaid
flowchart LR
    subgraph DETECT [YOLO 검출]
        D1[D405 Color Frame] --> D2[YOLOv8 추론]
        D2 --> D3[bbox 중앙 좌표<br/>cx, cy]
        D1b[D405 Depth Frame] --> D4[Robust Depth<br/>bbox 80% median]
        D3 --> D5[rs2_deproject_pixel_to_point]
        D4 --> D5
        D5 --> D6["P_cam = (Xc, Yc, Zc)"]
    end

    subgraph TRANSFORM [좌표 변환]
        T1["T_cam→gripper<br/>(Hand-Eye 결과, 고정)"]
        T2["T_base→gripper = FK(q)<br/>(현재 관절각, 실시간)"]
        D6 --> T3["P_gripper = T_cam→gripper × P_cam"]
        T1 --> T3
        T3 --> T4["P_base = FK(q) × P_gripper"]
        T2 --> T4
    end

    T4 --> RESULT["P_base = (Xb, Yb, Zb)<br/>로봇 베이스 좌표"]
```

### 7-6. Pitch Sum Conservation 상세

```mermaid
flowchart TD
    subgraph PRINCIPLE ["원리: 직렬 3관절의 pitch 합 = 그리퍼 절대 pitch"]
        direction TB
        PR1["shoulder_lift + elbow_flex + wrist_flex = 그리퍼 pitch"]
        PR1 --> PR2["이 합이 일정하면 → 그리퍼가 바라보는 방향 불변"]
    end

    PRINCIPLE --> CALC

    subgraph CALC ["계산 과정"]
        direction TB
        CA1["1. 최종 파지 IK로 target_pitch_sum 결정<br/>= SL + EF + WF + WRIST_FLEX_EXTRA(-15°) - 10°"]
        CA1 --> CA2["2. 팔 세운 상태에서 wrist_flex 세팅<br/>WF = target_pitch_sum - 90 - 0"]
        CA2 --> CA3["3. 하강 후 IK 결과에서 보정<br/>WF_new = target_pitch_sum - SL_new - EF_new"]
        CA3 --> CA4["4. 전진/들기 시에도 동일하게 보정"]
    end

    CALC --> EXAMPLE

    subgraph EXAMPLE ["예시 (target_pitch_sum = -6)"]
        direction TB
        EX1["세운 상태: SL=90, EF=0 → WF=-96"]
        EX1 --> EX2["하강 후: SL=44, EF=58 → WF=-108"]
        EX2 --> EX3["두 경우 모두 합 = -6<br/>→ 그리퍼 같은 방향"]
    end
```

---

## 8. 사용된 알고리즘

### 8-1. Forward Kinematics (FK)

관절각 → EEF 위치/자세 변환. URDF의 DH 파라미터를 기반으로 각 관절의 변환행렬을 연쇄 곱합니다.

```
T_base→eef = T_01(q1) × T_12(q2) × T_23(q3) × T_34(q4) × T_45(q5) × T_5→gripper(fixed)
```

- **라이브러리**: placo (lerobot 내장 `RobotKinematics`)
- **입력**: 관절각 5개 [deg]
- **출력**: 4x4 동차변환행렬 (위치 + 회전)
- **사용처**: 좌표 변환 (그리퍼→베이스), IK 결과 검증

### 8-2. Inverse Kinematics (IK)

목표 EEF 위치 → 관절각 역산. placo의 수치적 IK를 사용하며, 5DOF 제약으로 인해 위치만 우선합니다.

```
minimize  ||FK(q)_pos - target_pos||² × w_pos + ||FK(q)_rot - target_rot||² × w_ori
subject to  joint_limits
```

- **라이브러리**: placo (수치적 IK, damped least squares 계열)
- **위치 가중치**: 1.0 (항상)
- **자세 가중치**: 0.01 (top-down 힌트 시) / 0.0 (위치 우선)
- **특징**: 5DOF로 6DOF 자세 달성 불가 → `orientation_weight` 를 최소화

### 8-3. Multi-Candidate IK Strategy

5DOF IK는 국소 최적해에 빠지기 쉬우므로, 11개 실측 기반 초기값 후보에서 병렬 탐색합니다.

```
후보 선정 기준:
  1) 실측 파지 자세 [-13, 44, 58, -83, 0] 및 변형
  2) 현재 관절각
  3) 홈 자세 [0, 0, 0, 0, 0]

해 선택 기준 (prefer_topdown=True):
  1차: downward_score 높은 것 (그리퍼가 아래를 향할수록 좋음)
  2차: 위치 오차 적은 것

downward_score = -gripper_z[2]  (그리퍼 Z축의 월드 Z성분, 양수일수록 아래)
```

### 8-4. Hand-Eye Calibration (AX=XB)

Eye-in-hand 구성에서 카메라-그리퍼 간 고정 변환(T_cam→gripper)을 구합니다.

```
A_i × X = X × B_i

A_i = T_gripper2base(pose_i)^(-1) × T_gripper2base(pose_j)   (로봇 움직임)
B_i = T_target2cam(pose_i) × T_target2cam(pose_j)^(-1)       (보드 관측 변화)
X   = T_cam→gripper                                            (구하는 값)
```

- **방법**: Tsai-Lenz (cv2.CALIB_HAND_EYE_TSAI)
- **보드**: ChArUco 5x7 (DICT_6X6_250, square=30mm, marker=22mm)
- **데이터**: 15개+ 다양한 자세에서 수집
- **보드 검출**: `cv2.aruco.CharucoDetector` + `cv2.solvePnP`

### 8-5. Pitch Sum Conservation (그리퍼 각도 유지)

5DOF 직렬 로봇에서 shoulder_lift, elbow_flex, wrist_flex 3개 관절의 합이 일정하면 그리퍼의 절대 pitch 각도가 보존됩니다.

```
pitch_sum = shoulder_lift + elbow_flex + wrist_flex = constant

target_pitch_sum 결정:
  IK로 최종 파지 자세 미리 계산 → SL + EF + WF + WRIST_FLEX_EXTRA(-15°) - 10°

모든 이동 단계에서:
  wrist_flex_new = target_pitch_sum - shoulder_lift_new - elbow_flex_new
```

- **목적**: 하강/전진/들어올리기 시 그리퍼가 바닥을 향한 각도를 일정하게 유지
- **보정 타이밍**: IK 결과를 받은 직후, wrist_flex만 덮어쓰기
- **적용 범위**: 팔 세우기, 목표 위 이동, 하강, 전진, 들어올리기 — **전 구간**

### 8-6. Upright Preparation (팔 세우기 접근)

카메라 충돌을 원천 차단하기 위해, 팔을 수직으로 세운 상태에서 모든 관절 각도를 세팅합니다.

```
1. shoulder_lift = 90° (팔 수직)
2. elbow_flex = 0° (팔꿈치 펴기)
3. wrist_flex = target_pitch_sum - 90 - 0 (최종 파지 각도 미리 세팅)
4. shoulder_pan = IK 결과의 방향 (목표 XY 방향)
5. wrist_roll = 0° (중립)
```

- **목적**: 카메라가 물체와 먼 상태에서 안전하게 각도 세팅
- **이전 방식 대비**: 바닥에서 후퇴→꺾기→재전진 불필요

### 8-7. Gripper-Only Close (관절 고정 파지)

파지 시 그리퍼만 닫고 다른 관절은 변경하지 않습니다.

```python
obs = follower.get_observation()               # 현재 관절값 실시간 읽기
close_cmd = {k: obs[k] for k in obs if ".pos" in k}  # 모든 관절 현재값 유지
close_cmd["gripper.pos"] = GRASP_CLOSE          # 그리퍼만 닫기
```

- **목적**: 파지 시 wrist_flex 등이 변경되어 그리퍼 방향이 바뀌는 문제 방지
- **이전 방식**: IK 결과의 관절값을 close_cmd에 포함 → 미세한 차이로 관절 이동 발생

### 8-8. Linear Joint Interpolation (부드러운 이동)

급격한 관절 이동을 방지하기 위해 현재→목표를 선형 보간합니다.

```
for step i = 1 to N:
    alpha = i / N
    q_cmd = q_current + alpha × (q_target - q_current)
    robot.send_action(q_cmd)
    sleep(50ms)
```

- **이동 시간**: steps × 50ms (예: 30 steps = 1.5초)
- **구간별 steps**: 팔 세우기 40, 접근 30, 하강 25, 전진 15, 파지 30, 들기 20

### 8-9. Robust Depth Estimation

D405 depth 센서의 노이즈와 경계 불연속을 제거합니다.

```
1. bbox의 중앙 80% 영역만 사용 (가장자리 10% 제거)
2. depth > 0.01m인 유효 픽셀만 선택
3. 유효 픽셀의 median 값 반환
```

- **목적**: 물체 경계의 depth 불연속, 0값(미검출), 배경 혼입 방지

### 8-10. 3D Deprojection

2D 픽셀 좌표 + depth → 카메라 프레임 3D 좌표 변환.

```
X_cam = (cx - ppx) × depth / fx
Y_cam = (cy - ppy) × depth / fy
Z_cam = depth
```

- **함수**: `rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)`
- **intrinsics**: D405 공장 캘리브레이션 (fx, fy, ppx, ppy)

---

## 9. 시스템 연결도

```mermaid
flowchart LR
    subgraph HARDWARE [하드웨어]
        LEADER["리더암<br/>SO-ARM101<br/>/dev/ttyACM0"]
        FOLLOWER["팔로워암<br/>SO-ARM101<br/>/dev/ttyACM1"]
        D405["Intel D405<br/>그리퍼 장착"]
        TRASH["쓰레기통<br/>(종이컵 모양)"]
    end

    subgraph SOFTWARE [소프트웨어]
        YOLO["YOLOv8<br/>best.pt"]
        HE["Hand-Eye<br/>T_cam→gripper"]
        FK_IK["FK / IK<br/>placo"]
        CTRL["grasp_controller<br/>모션 시퀀스"]
    end

    LEADER -->|"get_action()"| CTRL
    CTRL -->|"send_action()"| FOLLOWER
    D405 -->|"color + depth"| YOLO
    YOLO -->|"bbox + 3D"| HE
    HE -->|"P_base"| FK_IK
    FK_IK -->|"joint angles"| CTRL
    FOLLOWER -->|"get_observation()"| FK_IK
    D405 -.->|"장착"| FOLLOWER
    FOLLOWER -.->|"파지"| TRASH
```
