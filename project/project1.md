# Project 1: SO-ARM101 쓰레기통 파지 (Classical 방식)

## 시스템 구성

```
리더암 (SO-ARM101)          팔로워암 (SO-ARM101)
  - 토크 OFF                  - 토크 ON
  - 사람이 잡고 움직임          - 리더암 자세 추종
  - get_action() 으로          - get_observation() 으로
    관절각 읽어서 전송            실제 관절각 읽기
                                - 그리퍼 위에 D405 장착 (eye-in-hand)
```

**입력**: 팔로워암 D405 (그리퍼 장착) + YOLO → 쓰레기통 3D 위치  
**출력**: SO-ARM101 팔로워암 관절 제어 → 파지  
**방식**: Eye-in-hand Hand-Eye Calibration + lerobot 내장 IK (placo)

---

## 전체 파이프라인

```
[리더암으로 팔로워암 조작]
      ↓ 팔로워암 관절각 = get_observation()
[FK(q) → T_base→gripper]   ← 매 순간 계산 (팔이 움직이면 계속 바뀜)
      ↑
      └─ 관절각(q)은 항상 팔로워암 엔코더에서 읽음

[D405 (그리퍼에 장착) + YOLO]
      ↓ 쓰레기통 픽셀 + depth → (Xc, Yc, Zc) 카메라 프레임
[T_cam→gripper 적용]        ← Hand-Eye Calibration 결과 (고정값)
      ↓ 그리퍼 프레임 좌표
[FK(q) = T_base→gripper 적용]  ← 현재 관절각으로 실시간 계산
      ↓ 로봇 베이스 프레임 좌표
[IK → 목표 관절각]
      ↓
[팔로워암 send_action()]
      ↓
[파지 실행]
```

---

## Calibration 상세 설명

이 프로젝트에서 필요한 캘리브레이션은 3종류입니다.

### 1) 관절 캘리브레이션 (Joint Calibration)

모터 엔코더의 raw 값(0~4095)을 실제 각도(degrees)로 변환하는 기준점 설정입니다.

```
모터 raw값: 0 ~ 4095  (12bit 엔코더)
                ↓ [캘리브레이션]
실제 각도: -150° ~ +150° (관절마다 다름)

과정:
① 팔을 가동 범위 중간 위치에 놓고 → homing offset 기록
② 각 관절을 끝에서 끝까지 움직여서 → range_min, range_max 기록
③ 결과를 모터 메모리와 파일에 저장
```

**리더암, 팔로워암 둘 다 따로 해야 합니다.**

### 2) 카메라 내부 캘리브레이션 (Intrinsic Calibration)

픽셀 좌표를 실제 각도로 변환하는 파라미터(fx, fy, cx, cy)입니다.  
D405는 **RealSense SDK가 공장 캘리브레이션 값을 내장**하고 있어서 별도 작업 불필요합니다.

```python
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
# intr.fx, intr.fy: 초점거리 (픽셀)
# intr.ppx, intr.ppy: 주점 (이미지 중심 근처)
# intr.coeffs: 왜곡 계수
```

### 3) Hand-Eye Calibration (핵심)

**"그리퍼를 기준으로 카메라가 어디에 어떻게 달려있는지"를 수식으로 구하는 작업**입니다.  
카메라가 그리퍼에 달려 있으므로 **eye-in-hand** 방식입니다.

```
구해야 할 것: T_cam→gripper  (4x4 변환행렬, 고정값)
              카메라 프레임 기준으로 그리퍼 원점이 어디에 있는지

왜 필요한가:
  D405가 본 물체 좌표 (카메라 프레임)
          ×  T_cam→gripper         ← 캘리브레이션 결과
  그리퍼 프레임 기준 물체 좌표
          ×  FK(q) = T_base→gripper  ← 현재 관절각으로 계산
  로봇 베이스 프레임 기준 물체 좌표
          →  IK  →  관절각
```

**수집 방법 (리더암 활용):**
```
① 바닥에 ChArUco 보드를 고정
② 리더암을 잡고 팔로워암을 자세 1로 이동
③ 동시에 기록:
   - 팔로워암 get_observation() → FK → T_base→gripper_1
   - D405가 본 보드 pose → T_board→cam_1
④ 리더암으로 자세 2, 3, ... 20으로 이동 반복
⑤ cv2.calibrateHandEye()로 T_cam→gripper 계산

수학적 원리 (AX = XB):
A_i = 자세i→자세j 그리퍼 상대 이동 (FK로 계산)
B_i = 자세i→자세j 카메라 상대 이동 (보드 관측으로 계산)
X = T_cam→gripper  (우리가 구하는 값)
```

---

## 사전 요구사항

### 설치 (Ubuntu 24.04)

```bash
pip install -e ".[dev,test,feetech,intelrealsense,kinematics]"
pip install opencv-python pyrealsense2 ultralytics scipy
```

### 파일 구조

```
project1/
├── calibration/
│   ├── step1_joint_calibration.sh      # Step 1: 관절 캘리브레이션 명령
│   ├── step2_collect_handeye.py        # Step 2-1: Hand-Eye 데이터 수집
│   ├── step2_run_handeye.py            # Step 2-2: 변환행렬 계산
│   └── hand_eye_result.npz             # 결과: T_cam_to_gripper
├── grasp/
│   ├── detect_target.py                # Step 3: YOLO + D405 → 3D 좌표
│   ├── coord_transform.py              # Step 4: 좌표 변환 (eye-in-hand)
│   ├── ik_solver.py                    # Step 5: IK 래퍼
│   └── grasp_controller.py            # Step 6: 메인 파지 루프
└── so_arm101.urdf
```

---

## Step 1: 관절 캘리브레이션

리더암과 팔로워암 **각각** 실행합니다.

```bash
# 팔로워암 캘리브레이션 (카메라 장착된 팔)
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyUSB0

# 리더암 캘리브레이션
lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyUSB1
```

**절차:**
1. 팔을 가동 범위 중간으로 옮기고 ENTER
2. 각 관절(`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`)을 끝에서 끝까지 움직인 후 ENTER
3. `wrist_roll`은 360도 full-turn이므로 자동 처리
4. 결과 → `~/.cache/huggingface/lerobot/calibration/`

**확인:**
```python
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

robot = SOFollower(SOFollowerRobotConfig(port="/dev/ttyUSB0"))
robot.connect(calibrate=False)
print(robot.get_observation())
# {'shoulder_pan.pos': 0.0, 'shoulder_lift.pos': 0.0, ...}  ← degrees
robot.disconnect()
```

---

## Step 2: Hand-Eye Calibration (eye-in-hand)

D405가 그리퍼에 달려 있으므로 **고정 보드를 팔 여러 자세에서 관찰**합니다.  
**리더암으로 팔로워암을 움직여서** 데이터를 수집합니다.

### Step 2-1: 데이터 수집 (`step2_collect_handeye.py`)

```python
# step2_collect_handeye.py
import numpy as np
import cv2
import pyrealsense2 as rs
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
from lerobot.model.kinematics import RobotKinematics

FOLLOWER_PORT = "/dev/ttyUSB0"
LEADER_PORT   = "/dev/ttyUSB1"
URDF_PATH     = "project1/so_arm101.urdf"
EEF_FRAME     = "gripper_frame_link"   # URDF 내 end-effector 프레임 이름 확인 필요
JOINT_NAMES   = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# ChArUco 보드 설정 (실제 인쇄 크기에 맞게 조정)
SQUARES_X      = 5
SQUARES_Y      = 7
SQUARE_LENGTH  = 0.04   # 40mm
MARKER_LENGTH  = 0.03   # 30mm


def make_charuco_board():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), SQUARE_LENGTH, MARKER_LENGTH, dictionary
    )
    return board, dictionary


def detect_board_pose(frame_gray, camera_matrix, dist_coeffs):
    """바닥에 고정된 ChArUco 보드의 카메라 기준 pose 반환."""
    board, _ = make_charuco_board()
    detector = cv2.aruco.CharucoDetector(board)
    corners, ids, _, _ = detector.detectBoard(frame_gray)
    if ids is None or len(ids) < 6:
        return None, None
    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        corners, ids, board, camera_matrix, dist_coeffs, None, None
    )
    if not ok:
        return None, None
    return rvec, tvec   # 카메라 프레임 기준 보드 pose


def main():
    # ── 장치 연결 ──────────────────────────────────────────
    follower = SOFollower(SOFollowerRobotConfig(port=FOLLOWER_PORT))
    follower.connect(calibrate=False)

    leader = SOLeader(SOLeaderTeleopConfig(port=LEADER_PORT))
    leader.connect(calibrate=False)

    kin = RobotKinematics(URDF_PATH, target_frame_name=EEF_FRAME, joint_names=JOINT_NAMES)

    # ── D405 연결 ───────────────────────────────────────────
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    profile = pipeline.start(cfg)

    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    camera_matrix = np.array([[intr.fx, 0, intr.ppx],
                               [0, intr.fy, intr.ppy],
                               [0,       0,         1]])
    dist_coeffs = np.array(intr.coeffs)

    # ── 수집 루프 ───────────────────────────────────────────
    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list   = []
    t_target2cam_list   = []

    print("=" * 50)
    print("Hand-Eye Calibration 데이터 수집")
    print("리더암을 잡고 팔로워암을 원하는 자세로 이동하세요.")
    print("보드가 검출되면 ENTER로 저장, 'q'로 종료.")
    print("최소 15개 이상, 다양한 방향/거리에서 수집하세요.")
    print("=" * 50)

    while True:
        # 리더암 관절각 읽어서 팔로워암에 전달 (텔레오퍼레이션)
        action = leader.get_action()
        follower.send_action(action)

        # D405 프레임 읽기
        frames = pipeline.wait_for_frames()
        color_image = np.asanyarray(frames.get_color_frame().get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)

        rvec, tvec = detect_board_pose(gray, camera_matrix, dist_coeffs)

        # 시각화
        vis = color_image.copy()
        status = f"저장됨: {len(R_gripper2base_list)}개"
        if rvec is not None:
            cv2.drawFrameAxes(vis, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
            cv2.putText(vis, f"[보드 검출됨] {status}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(vis, f"[보드 미검출] {status}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow("Hand-Eye Calibration", cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == 13:  # ENTER
            if rvec is None:
                print("보드가 검출되지 않았습니다. 보드가 카메라에 보이는지 확인하세요.")
                continue

            # 팔로워암 현재 관절각 읽기 → FK → T_base→gripper
            obs = follower.get_observation()
            q = np.array([obs[f"{name}.pos"] for name in JOINT_NAMES])
            T_base_gripper = kin.forward_kinematics(q)   # 4x4

            R_g2b = T_base_gripper[:3, :3]
            t_g2b = T_base_gripper[:3, 3].reshape(3, 1)
            R_t2c, _ = cv2.Rodrigues(rvec)

            R_gripper2base_list.append(R_g2b)
            t_gripper2base_list.append(t_g2b)
            R_target2cam_list.append(R_t2c)
            t_target2cam_list.append(tvec)
            print(f"[{len(R_gripper2base_list)}] 저장 완료. "
                  f"그리퍼 위치: {t_g2b.T}, 보드 거리: {tvec[2][0]:.3f}m")

    pipeline.stop()
    follower.disconnect()
    leader.disconnect()
    cv2.destroyAllWindows()

    np.savez(
        "calibration/handeye_data.npz",
        R_gripper2base=np.array(R_gripper2base_list),
        t_gripper2base=np.array(t_gripper2base_list),
        R_target2cam=np.array(R_target2cam_list),
        t_target2cam=np.array(t_target2cam_list),
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
    )
    print(f"\n수집 완료: {len(R_gripper2base_list)}개 자세 저장")


if __name__ == "__main__":
    main()
```

### Step 2-2: 변환행렬 계산 (`step2_run_handeye.py`)

```python
# step2_run_handeye.py
import numpy as np
import cv2

data = np.load("calibration/handeye_data.npz")

# eye-in-hand 방식으로 T_cam→gripper 계산
# 반환값: 카메라 프레임 → 그리퍼 프레임 변환
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    list(data["R_gripper2base"]),
    list(data["t_gripper2base"]),
    list(data["R_target2cam"]),
    list(data["t_target2cam"]),
    method=cv2.CALIB_HAND_EYE_TSAI,
)

T_cam_to_gripper = np.eye(4)
T_cam_to_gripper[:3, :3] = R_cam2gripper
T_cam_to_gripper[:3, 3]  = t_cam2gripper.flatten()

print("T_cam_to_gripper (카메라→그리퍼 변환행렬):")
print(np.round(T_cam_to_gripper, 4))
print(f"\n카메라 위치 (그리퍼 기준): {t_cam2gripper.flatten()} [m]")
print("  → 이 값이 카메라가 그리퍼에서 얼마나 떨어져 장착됐는지를 나타냄")

np.savez(
    "calibration/hand_eye_result.npz",
    T_cam_to_gripper=T_cam_to_gripper,
    camera_matrix=data["camera_matrix"],
    dist_coeffs=data["dist_coeffs"],
)
print("\n결과 저장: calibration/hand_eye_result.npz")
```

---

## Step 3: YOLO + D405 → 3D 좌표 (`detect_target.py`)

### 2D → 3D 변환 원리 (핀홀 카메라 모델)

```
픽셀 (u, v)에서 깊이 d [meters] 가 주어지면:

  Xc = (u - cx) * d / fx      ← 카메라 기준 좌우
  Yc = (v - cy) * d / fy      ← 카메라 기준 상하
  Zc = d                       ← 카메라에서 전방 거리

카메라 좌표계:
       Z (전방, depth 방향)
       ↑
       |
       +──→ X (오른쪽)
      /
    Y (아래쪽)
```

bbox 중심 픽셀 하나만 사용하면 반사·투명 재질·엣지 노이즈에 취약합니다.  
**bbox 내부 중앙 80% 영역의 median depth**를 사용합니다.

```python
# detect_target.py
import numpy as np
import pyrealsense2 as rs
import cv2
from ultralytics import YOLO

model = YOLO("yolov8n.pt")  # 쓰레기통 인식 학습된 가중치


def get_robust_depth(x1, y1, x2, y2, depth_image):
    """
    bbox 중앙 80% 영역의 유효 depth 중앙값을 반환합니다.
    엣지 픽셀(깊이 불연속)과 0값(미검출)을 제외합니다.
    """
    mx = max(1, (x2 - x1) // 10)
    my = max(1, (y2 - y1) // 10)
    roi = depth_image[y1 + my : y2 - my, x1 + mx : x2 - mx]
    valid = roi[roi > 0.1]   # 10cm 이상만
    if len(valid) == 0:
        return None
    return float(np.median(valid))


def get_trash_can_3d_cam(pipeline, align, depth_scale, color_intr):
    """
    YOLO로 쓰레기통 검출 후 카메라 프레임 3D 좌표를 반환합니다.
    반환: np.array([Xc, Yc, Zc]) meters, 카메라 프레임 기준
    """
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()
    if not color_frame or not depth_frame:
        return None

    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data()).astype(float) * depth_scale

    results = model(color_image)

    best = None
    best_conf = 0.0
    for box in results[0].boxes:
        conf = float(box.conf[0])
        if conf < 0.5 or conf <= best_conf:
            continue
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
        d = get_robust_depth(x1, y1, x2, y2, depth_image)
        if d is None:
            continue
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        point = rs.rs2_deproject_pixel_to_point(color_intr, [cx, cy], d)
        best = np.array(point)
        best_conf = conf

    return best
```

---

## Step 4: 좌표 변환 (`coord_transform.py`)

카메라 좌표 → 베이스 좌표 변환 과정입니다.  
카메라가 그리퍼에 달려 있으므로 **현재 관절각(FK)**이 반드시 필요합니다.

```python
# coord_transform.py
import numpy as np
from lerobot.model.kinematics import RobotKinematics

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# 캘리브레이션 결과 로드
_cal = np.load("calibration/hand_eye_result.npz")
T_CAM_TO_GRIPPER = _cal["T_cam_to_gripper"]   # 고정값 (카메라→그리퍼)


def cam_to_base(point_cam: np.ndarray,
                current_joints_deg: np.ndarray,
                kinematics: RobotKinematics) -> np.ndarray:
    """
    카메라 프레임 3D 좌표를 로봇 베이스 프레임으로 변환합니다.

    eye-in-hand이므로 변환이 2단계입니다:
      1. p_cam × T_cam→gripper  →  p_gripper  (고정, 캘리브 결과)
      2. p_gripper × FK(q)      →  p_base     (매 순간 변함)

    Args:
        point_cam: 카메라 프레임 3D 좌표 [Xc, Yc, Zc] meters
        current_joints_deg: 팔로워암 현재 관절각 [deg] (get_observation()으로 읽은 값)
        kinematics: RobotKinematics 인스턴스
    Returns:
        p_base: 로봇 베이스 프레임 3D 좌표 [Xb, Yb, Zb] meters
    """
    # 단계 1: 카메라 프레임 → 그리퍼 프레임
    p_cam_h = np.append(point_cam, 1.0)            # 동차 좌표
    p_gripper_h = T_CAM_TO_GRIPPER @ p_cam_h
    p_gripper = p_gripper_h[:3]

    # 단계 2: 그리퍼 프레임 → 베이스 프레임 (현재 관절각으로 FK 계산)
    T_base_gripper = kinematics.forward_kinematics(current_joints_deg)   # 4x4
    p_gripper_h2 = np.append(p_gripper, 1.0)
    p_base_h = T_base_gripper @ p_gripper_h2

    return p_base_h[:3]
```

---

## Step 5: IK 래퍼 (`ik_solver.py`)

### IK가 관절값을 찾는 원리

```
① 초기값: 현재 팔로워암 관절각 q (get_observation()으로 읽은 값)
          ← 반드시 현재 실제 관절각을 초기값으로 사용
          ← 다른 값을 넣으면 엉뚱한 해 또는 특이점에 빠질 수 있음

② FK(q): URDF의 링크/조인트 구조로 현재 EEF 위치/자세 계산

③ 오차: e_pos = target - current (위치), e_rot (자세)

④ 야코비안(J): 관절각 변화 → EEF 변화 비율 (6×5 행렬)
   Δq = J⁺ · [e_pos, e_rot]

⑤ q ← q + Δq, 수렴할 때까지 ②~⑤ 반복

결과: q = [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll] [deg]
```

```python
# ik_solver.py
import numpy as np
from scipy.spatial.transform import Rotation as R
from lerobot.model.kinematics import RobotKinematics

URDF_PATH  = "project1/so_arm101.urdf"
EEF_FRAME  = "gripper_frame_link"   # URDF 내 end-effector 프레임 이름 확인 필요
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


class IKSolver:
    def __init__(self):
        self.kin = RobotKinematics(
            urdf_path=URDF_PATH,
            target_frame_name=EEF_FRAME,
            joint_names=JOINT_NAMES,
        )

    def check_eef_axes(self):
        """홈 자세에서 FK 실행 → 그리퍼 축 방향 확인. 처음 설정 시 한 번 실행."""
        T = self.kin.forward_kinematics(np.zeros(len(JOINT_NAMES)))
        print("=== 홈 자세 EEF 정보 ===")
        print(f"위치 (베이스 기준): {T[:3, 3]}")
        print(f"X축 방향: {T[:3, 0]}")
        print(f"Y축 방향: {T[:3, 1]}")
        print(f"Z축 방향: {T[:3, 2]}  ← 보통 그리퍼 접근 방향")
        return T

    def solve(self,
              target_pos: np.ndarray,
              target_rot: np.ndarray,
              current_joints_deg: np.ndarray) -> np.ndarray | None:
        """
        Args:
            target_pos: 목표 EEF 위치 [x, y, z] meters, 베이스 프레임
            target_rot: 목표 EEF 자세 3x3 회전행렬, 베이스 프레임
            current_joints_deg: 현재 팔로워암 관절각 [deg]
        Returns:
            q [deg] 또는 None
        """
        T_target = np.eye(4)
        T_target[:3, :3] = target_rot
        T_target[:3, 3]  = target_pos

        try:
            q = self.kin.inverse_kinematics(current_joints_deg, T_target)
            # FK로 검증
            T_result = self.kin.forward_kinematics(q)
            err_mm = np.linalg.norm(T_result[:3, 3] - target_pos) * 1000
            if err_mm > 10.0:
                print(f"IK 수렴 불량: 위치 오차 {err_mm:.1f}mm")
                return None
            print(f"IK 성공: 위치 오차 {err_mm:.1f}mm, 관절각: {np.round(q, 1)}")
            return q
        except Exception as e:
            print(f"IK 예외: {e}")
            return None

    def compute_grasp_pose(self,
                           target_pos: np.ndarray,
                           approach_offset: float = 0.05
                           ) -> tuple[np.ndarray, np.ndarray]:
        """
        목표 위치 위쪽에서 수직으로 내려오는 접근 pose를 계산합니다.
        어느 방향이 '아래'인지는 URDF에 따라 달라질 수 있으므로
        check_eef_axes()로 먼저 확인하세요.
        """
        # 위에서 수직으로 내려오는 자세: X축 기준 180도 회전 (그리퍼가 아래를 향함)
        grasp_rot = R.from_euler("x", 180, degrees=True).as_matrix()
        approach_pos = target_pos.copy()
        approach_pos[2] += approach_offset   # 목표 위 5cm 에서 접근
        return approach_pos, grasp_rot
```

---

## Step 6: 메인 파지 루프 (`grasp_controller.py`)

```python
# grasp_controller.py
import time
import numpy as np
import pyrealsense2 as rs
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
from lerobot.model.kinematics import RobotKinematics

from detect_target import get_trash_can_3d_cam
from coord_transform import cam_to_base, JOINT_NAMES
from ik_solver import IKSolver

# ─── 설정 ──────────────────────────────────────────────
FOLLOWER_PORT      = "/dev/ttyUSB0"
LEADER_PORT        = "/dev/ttyUSB1"
URDF_PATH          = "project1/so_arm101.urdf"
EEF_FRAME          = "gripper_frame_link"
MAX_REL_TARGET     = 5.0    # 한 스텝 최대 관절 이동량 [deg]
GRASP_OPEN         = 0.0    # 그리퍼 열림 (0~100%)
GRASP_CLOSE        = 60.0   # 그리퍼 닫힘
# ────────────────────────────────────────────────────────


def move_joints_smooth(robot, target_joints: dict, steps: int = 30):
    """현재 자세에서 목표 관절각으로 선형 보간해서 이동합니다."""
    obs = robot.get_observation()
    current = {k: obs[k] for k in target_joints}

    for i in range(1, steps + 1):
        alpha = i / steps
        cmd = {k: current[k] + alpha * (target_joints[k] - current[k])
               for k in target_joints}
        robot.send_action(cmd)
        time.sleep(0.05)


def main():
    # ── 장치 연결 ──────────────────────────────────────────
    follower = SOFollower(SOFollowerRobotConfig(
        port=FOLLOWER_PORT,
        max_relative_target=MAX_REL_TARGET,
    ))
    follower.connect(calibrate=False)

    leader = SOLeader(SOLeaderTeleopConfig(port=LEADER_PORT))
    leader.connect(calibrate=False)

    kin = RobotKinematics(URDF_PATH, target_frame_name=EEF_FRAME, joint_names=JOINT_NAMES)
    ik  = IKSolver()

    # ── D405 연결 ───────────────────────────────────────────
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    color_intr  = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    print("=== 파지 시작 ===")

    try:
        # 1. 홈 포지션으로 이동 (그리퍼 열린 상태)
        home = {f"{j}.pos": 0.0 for j in JOINT_NAMES}
        home["gripper.pos"] = GRASP_OPEN
        move_joints_smooth(follower, home, steps=30)
        print("홈 포지션 완료")
        time.sleep(0.5)

        # 2. 쓰레기통 검출 (카메라 프레임 3D 좌표)
        print("쓰레기통 검출 중...")
        point_cam = None
        while point_cam is None:
            # 리더암 → 팔로워암 텔레오퍼레이션 유지하면서 검출
            action = leader.get_action()
            follower.send_action(action)
            point_cam = get_trash_can_3d_cam(pipeline, align, depth_scale, color_intr)
        print(f"카메라 프레임 좌표: {point_cam}")

        # 3. 현재 팔로워암 관절각 읽기 (eye-in-hand이므로 반드시 필요)
        obs = follower.get_observation()
        q_current = np.array([obs[f"{j}.pos"] for j in JOINT_NAMES])

        # 4. 카메라 → 베이스 좌표 변환
        #    T_cam→gripper (고정) + FK(q_current) (현재 자세) 사용
        point_base = cam_to_base(point_cam, q_current, kin)
        print(f"베이스 프레임 좌표: {point_base}")

        # 5. 파지 접근 pose 계산 (목표 위 5cm)
        approach_pos, grasp_rot = ik.compute_grasp_pose(point_base, approach_offset=0.05)

        # 6. IK: 현재 관절각을 초기값으로 → 접근 위치 관절각 계산
        q_approach = ik.solve(approach_pos, grasp_rot, q_current)
        if q_approach is None:
            print("IK 실패. 쓰레기통이 작업 공간 내에 있는지 확인하세요.")
            return

        # 7. 접근 위치로 이동
        approach_cmd = {f"{j}.pos": float(q_approach[i]) for i, j in enumerate(JOINT_NAMES)}
        approach_cmd["gripper.pos"] = GRASP_OPEN
        move_joints_smooth(follower, approach_cmd, steps=40)
        print("접근 위치 이동 완료")
        time.sleep(0.5)

        # 8. 수직 하강 (실제 파지 위치로)
        grasp_pos = point_base.copy()
        q_down = ik.solve(grasp_pos, grasp_rot, q_approach)
        if q_down is not None:
            down_cmd = {f"{j}.pos": float(q_down[i]) for i, j in enumerate(JOINT_NAMES)}
            down_cmd["gripper.pos"] = GRASP_OPEN
            move_joints_smooth(follower, down_cmd, steps=15)
        time.sleep(0.3)

        # 9. 그리퍼 닫기 (파지)
        q_final = q_down if q_down is not None else q_approach
        close_cmd = {f"{j}.pos": float(q_final[i]) for i, j in enumerate(JOINT_NAMES)}
        close_cmd["gripper.pos"] = GRASP_CLOSE
        follower.send_action(close_cmd)
        time.sleep(1.0)
        print("파지 완료!")

        # 10. 들어올리기
        lift_pos = point_base.copy()
        lift_pos[2] += 0.15
        q_lift = ik.solve(lift_pos, grasp_rot, q_final)
        if q_lift is not None:
            lift_cmd = {f"{j}.pos": float(q_lift[i]) for i, j in enumerate(JOINT_NAMES)}
            lift_cmd["gripper.pos"] = GRASP_CLOSE
            move_joints_smooth(follower, lift_cmd, steps=20)
            print("들어올리기 완료!")

    finally:
        pipeline.stop()
        follower.disconnect()
        leader.disconnect()


if __name__ == "__main__":
    main()
```

---

## 체크리스트

### URDF 확인 (필수)
```bash
# end-effector 링크 이름 확인
grep -i "gripper\|end_effector\|tool" so_arm101.urdf | grep "<link"
# 결과를 EEF_FRAME 변수에 반영
```

### 캘리브레이션 품질 검증
Hand-Eye 캘리브레이션 완료 후 아래로 정확도를 확인합니다:
```
① 보드를 바닥에 고정
② 팔로워암을 새로운 자세로 이동
③ 카메라로 보드 3D 좌표 계산 → cam_to_base()로 변환
④ 실제 보드 위치(줄자로 측정)와 비교
→ 오차 < 10mm 이면 양호
```

### 안전 파라미터
- `MAX_REL_TARGET = 5.0`: 한 스텝 최대 5도 이동 (첫 실행 시 낮게 유지)
- `move_joints_smooth(steps=40)`: 숫자 크게 할수록 느리고 안전
- 팔 주변 장애물 제거 후 실행

---

## 실행 순서

```bash
# 1. 관절 캘리브레이션 (처음 한 번)
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyUSB0
lerobot-calibrate --teleop.type=so101_leader  --teleop.port=/dev/ttyUSB1

# 2. Hand-Eye 데이터 수집 (카메라 위치 변경 시 재수집)
python calibration/step2_collect_handeye.py

# 3. 변환행렬 계산
python calibration/step2_run_handeye.py

# 4. EEF 축 방향 확인 (처음 한 번)
python -c "from grasp.ik_solver import IKSolver; IKSolver().check_eef_axes()"

# 5. 파지 실행
python grasp/grasp_controller.py
```
