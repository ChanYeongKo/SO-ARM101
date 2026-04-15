# Project 1: SO-ARM101 쓰레기통 파지 (Classical 방식)

## 개요

**입력**: D405 RealSense + YOLO → 쓰레기통 3D 위치 (카메라 프레임)  
**출력**: SO-ARM101 관절 제어 → 파지  
**방식**: Hand-Eye Calibration + lerobot 내장 IK (placo)

---

## 전체 파이프라인

```
[D405 depth + YOLO]
      ↓ 쓰레기통 픽셀 좌표 + 깊이값
[3D 좌표 변환 (카메라 프레임)]
      ↓ (u, v, d) → (Xc, Yc, Zc)
[Hand-Eye Calibration 변환행렬 T_base_cam]
      ↓ 카메라 프레임 → 로봇 베이스 프레임
[목표 EEF pose 계산]
      ↓ 파지 방향 결정 포함
[lerobot RobotKinematics.inverse_kinematics()]
      ↓ 관절각 (degrees)
[robot.send_action()]
      ↓
[SO-ARM101 파지 실행]
```

---

## 사전 요구사항

### 설치 (Ubuntu 24.04)

```bash
# lerobot kinematics, intelrealsense extras 포함 설치
pip install -e ".[dev,test,feetech,intelrealsense,kinematics]"

# placo 설치 (kinematics extras에 포함되지 않을 경우 수동)
pip install placo

# 추가 라이브러리
pip install opencv-python pyrealsense2 ultralytics scipy
```

### 파일 구조

```
project1/
├── calibration/
│   ├── run_joint_calibration.py      # Step 1: 관절 캘리브레이션
│   ├── collect_handeye_data.py       # Step 2-1: hand-eye 데이터 수집
│   ├── run_handeye_calibration.py    # Step 2-2: 변환행렬 계산
│   └── hand_eye_result.npz           # 결과 저장 (T_base_cam)
├── grasp/
│   ├── detect_target.py              # Step 3: YOLO + D405 → 3D 좌표
│   ├── ik_solver.py                  # Step 4: IK 래퍼
│   └── grasp_controller.py           # Step 5: 메인 파지 루프
└── so_arm101.urdf                    # 보유 중인 URDF
```

---

## Step 1: 관절 캘리브레이션

lerobot 내장 캘리브레이션 도구를 사용합니다.  
각 관절의 zero point와 range를 모터에 기록합니다.

```bash
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyUSB0
```

**절차 안내:**
1. 팔을 대략 중간 위치(가동 범위 중간)로 옮기고 ENTER
2. 각 관절(`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`)을 순서대로 끝에서 끝까지 움직인 후 ENTER
3. `wrist_roll`은 360도 full-turn 관절이므로 자동 처리
4. 결과가 `~/.cache/huggingface/lerobot/calibration/` 에 저장됨

**결과 확인:**
```python
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

config = SOFollowerRobotConfig(port="/dev/ttyUSB0")
robot = SOFollower(config)
robot.connect()
obs = robot.get_observation()
# {'shoulder_pan.pos': 0.0, 'shoulder_lift.pos': 90.0, ...} (degree 값)
print(obs)
robot.disconnect()
```

---

## Step 2: Hand-Eye Calibration

D405가 로봇에 **고정 장착**되어 있으므로 **eye-to-hand** 방식을 사용합니다.  
(`T_base←camera`: 카메라 프레임 → 로봇 베이스 프레임 변환행렬)

### Step 2-1: 데이터 수집 (`collect_handeye_data.py`)

ChArUco 보드를 여러 위치에 놓고, 각 위치에서:
- 로봇 EEF(엔드이펙터)의 베이스 기준 위치/자세 (FK로 계산)
- 카메라가 본 ChArUco 보드의 위치/자세

를 수집합니다. 최소 15개 이상의 자세에서 수집하며, 다양한 방향·거리에서 수집할수록 정확합니다.

```python
# collect_handeye_data.py
import numpy as np
import cv2
import pyrealsense2 as rs
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
from lerobot.model.kinematics import RobotKinematics

URDF_PATH = "project1/so_arm101.urdf"
EEF_FRAME = "gripper_frame_link"  # URDF의 엔드이펙터 프레임 이름 확인 필요
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# ChArUco 보드 설정 (실제 인쇄 크기에 맞게 조정)
CHARUCO_SQUARES_X = 5
CHARUCO_SQUARES_Y = 7
CHARUCO_SQUARE_LENGTH = 0.04  # 40mm
CHARUCO_MARKER_LENGTH = 0.03  # 30mm

def get_charuco_board():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard(
        (CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y),
        CHARUCO_SQUARE_LENGTH,
        CHARUCO_MARKER_LENGTH,
        dictionary,
    )
    return board, dictionary

def detect_charuco_pose(frame, camera_matrix, dist_coeffs):
    """ChArUco 보드의 카메라 기준 pose를 반환."""
    board, dictionary = get_charuco_board()
    detector = cv2.aruco.CharucoDetector(board)
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)
    if charuco_ids is None or len(charuco_ids) < 6:
        return None, None

    valid, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None
    )
    if not valid:
        return None, None
    return rvec, tvec  # 카메라 프레임 기준 보드 pose

def get_eef_pose(robot, kinematics):
    """현재 관절각으로 FK → EEF 4x4 변환행렬 (로봇 베이스 기준)."""
    obs = robot.get_observation()
    q = np.array([obs[f"{name}.pos"] for name in JOINT_NAMES])
    T = kinematics.forward_kinematics(q)  # 4x4 matrix
    return T

def main():
    # 로봇 연결
    config = SOFollowerRobotConfig(port="/dev/ttyUSB0")
    robot = SOFollower(config)
    robot.connect(calibrate=False)

    # Kinematics
    kin = RobotKinematics(URDF_PATH, target_frame_name=EEF_FRAME, joint_names=JOINT_NAMES)

    # D405 연결
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(cfg)

    # 카메라 내부 파라미터 (intrinsics)
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_stream.get_intrinsics()
    camera_matrix = np.array([
        [intr.fx, 0,       intr.ppx],
        [0,       intr.fy, intr.ppy],
        [0,       0,       1       ]
    ])
    dist_coeffs = np.array(intr.coeffs)

    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list = []
    t_target2cam_list = []

    print("ChArUco 보드를 여러 위치에 놓고, 각 위치에서 ENTER를 누르세요.")
    print("최소 15개 이상의 자세를 수집하세요. 'q' 입력 시 종료.")

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = np.asanyarray(frames.get_color_frame().get_data())

        rvec, tvec = detect_charuco_pose(color_frame, camera_matrix, dist_coeffs)

        # 시각화
        vis = color_frame.copy()
        if rvec is not None:
            cv2.drawFrameAxes(vis, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
            cv2.putText(vis, f"Poses: {len(R_gripper2base_list)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(vis, "보드 미검출", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("Hand-Eye Calibration", cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == 13:  # ENTER
            if rvec is None:
                print("보드가 검출되지 않았습니다. 다시 시도하세요.")
                continue

            # EEF pose (로봇 베이스 기준, 4x4)
            T_eef = get_eef_pose(robot, kin)
            R_g2b = T_eef[:3, :3]
            t_g2b = T_eef[:3, 3].reshape(3, 1)

            # ChArUco pose (카메라 기준)
            R_t2c, _ = cv2.Rodrigues(rvec)

            R_gripper2base_list.append(R_g2b)
            t_gripper2base_list.append(t_g2b)
            R_target2cam_list.append(R_t2c)
            t_target2cam_list.append(tvec)
            print(f"[{len(R_gripper2base_list)}] 자세 저장됨. EEF: {t_g2b.T}")

    pipeline.stop()
    robot.disconnect()
    cv2.destroyAllWindows()

    # 저장
    np.savez(
        "calibration/handeye_data.npz",
        R_gripper2base=np.array(R_gripper2base_list),
        t_gripper2base=np.array(t_gripper2base_list),
        R_target2cam=np.array(R_target2cam_list),
        t_target2cam=np.array(t_target2cam_list),
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
    )
    print(f"데이터 저장 완료: {len(R_gripper2base_list)}개 자세")

if __name__ == "__main__":
    main()
```

### Step 2-2: 변환행렬 계산 (`run_handeye_calibration.py`)

```python
# run_handeye_calibration.py
import numpy as np
import cv2

data = np.load("calibration/handeye_data.npz")

R_gripper2base = list(data["R_gripper2base"])
t_gripper2base = list(data["t_gripper2base"])
R_target2cam   = list(data["R_target2cam"])
t_target2cam   = list(data["t_target2cam"])
camera_matrix  = data["camera_matrix"]
dist_coeffs    = data["dist_coeffs"]

# eye-to-hand: TSAI 방법 사용
# 반환값: R_cam2base, t_cam2base (카메라 → 로봇 베이스 변환)
R_cam2base, t_cam2base = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,
    R_target2cam,   t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI,
)

# 4x4 변환행렬로 조합
T_base_cam = np.eye(4)
T_base_cam[:3, :3] = R_cam2base
T_base_cam[:3, 3]  = t_cam2base.flatten()

print("T_base_cam (카메라→베이스 변환행렬):")
print(T_base_cam)
print(f"\n추정 카메라 위치 (베이스 기준): {t_cam2base.flatten()} [m]")

# 저장
np.savez(
    "calibration/hand_eye_result.npz",
    T_base_cam=T_base_cam,
    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,
)
print("결과 저장: calibration/hand_eye_result.npz")
```

---

## Step 3: YOLO + D405 → 3D 좌표 (`detect_target.py`)

D405의 depth 스트림과 RGB 스트림을 align하여 YOLO가 검출한 쓰레기통의 3D 좌표를 얻습니다.

```python
# detect_target.py
import numpy as np
import pyrealsense2 as rs
import cv2
from ultralytics import YOLO

model = YOLO("yolov8n.pt")  # 또는 커스텀 학습된 가중치

def pixel_to_3d(u, v, depth_frame, intrinsics):
    """
    픽셀 좌표 (u, v)와 depth 값으로 카메라 프레임 3D 좌표를 반환합니다.
    D405는 depth-to-color align 후 depth 해상도 = color 해상도.
    """
    depth = depth_frame.get_distance(u, v)
    if depth == 0:
        return None  # depth 없음
    # RealSense SDK 역투영 함수 사용
    point = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth)
    return np.array(point)  # [X, Y, Z] in meters, camera frame

def get_trash_can_3d():
    """YOLO로 쓰레기통 검출 후 카메라 프레임 3D 좌표를 반환."""
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(cfg)

    # Color-Depth 정렬 (D405는 동일 광학축이지만 align 권장)
    align = rs.align(rs.stream.color)

    color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            results = model(color_image, classes=[60])  # COCO class 60: dining table, 필요시 커스텀

            # 쓰레기통 클래스 ID는 학습한 모델에 맞게 조정
            # results[0].boxes 에 검출 결과가 있음

            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])
                if conf < 0.5:
                    continue

                # bbox 중심점
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                point_3d = pixel_to_3d(cx, cy, depth_frame, color_intr)
                if point_3d is None:
                    continue

                # 시각화
                vis = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(vis, f"{point_3d[2]:.3f}m", (cx, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.imshow("Detection", vis)

                print(f"쓰레기통 3D 좌표 (카메라 프레임): {point_3d} [m]")
                return point_3d  # 첫 번째 검출 결과 반환

            cv2.imshow("Detection", cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return None
```

---

## Step 4: IK 래퍼 (`ik_solver.py`)

lerobot 내장 `RobotKinematics` (placo)를 활용합니다.

```python
# ik_solver.py
import numpy as np
from scipy.spatial.transform import Rotation as R
from lerobot.model.kinematics import RobotKinematics

URDF_PATH = "project1/so_arm101.urdf"
EEF_FRAME = "gripper_frame_link"  # URDF 내 end-effector 프레임 이름 확인 필요
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

class IKSolver:
    def __init__(self):
        self.kin = RobotKinematics(
            urdf_path=URDF_PATH,
            target_frame_name=EEF_FRAME,
            joint_names=JOINT_NAMES,
        )

    def solve(self, target_pos_base: np.ndarray,
              target_rot_base: np.ndarray,
              current_joints_deg: np.ndarray) -> np.ndarray | None:
        """
        Args:
            target_pos_base: 목표 EEF 위치 [x, y, z] (meters, 로봇 베이스 프레임)
            target_rot_base: 목표 EEF 자세 3x3 회전행렬 (로봇 베이스 프레임)
            current_joints_deg: 현재 관절각 [deg] (초기값으로 사용)
        Returns:
            joint_angles_deg: 목표 관절각 [deg] 또는 None (수렴 실패)
        """
        T_target = np.eye(4)
        T_target[:3, :3] = target_rot_base
        T_target[:3, 3] = target_pos_base

        try:
            q_target = self.kin.inverse_kinematics(current_joints_deg, T_target)
            return q_target  # degrees
        except Exception as e:
            print(f"IK 실패: {e}")
            return None

    def compute_grasp_pose(self, target_pos_base: np.ndarray,
                           approach_axis: str = "z") -> tuple[np.ndarray, np.ndarray]:
        """
        쓰레기통 위치로부터 파지 pose를 계산합니다.
        위에서 아래로 접근하는 기본 grasp pose를 생성합니다.

        Args:
            target_pos_base: 쓰레기통 중심 위치 [x, y, z] (로봇 베이스 프레임)
            approach_axis: 접근 방향 ("z": 위에서 아래, "x": 앞에서)
        Returns:
            (grasp_pos, grasp_rot): EEF 목표 위치(meters), 회전행렬(3x3)
        """
        # 기본 파지 자세: 그리퍼가 수직으로 내려와서 잡는 방향
        # 로봇 베이스 z축 = 위 방향으로 가정
        if approach_axis == "z":
            # 위에서 아래로 내려오는 grasp
            # gripper z축(접근 방향)이 -z_base 방향 (아래를 향함)
            rot = R.from_euler("xyz", [180, 0, 0], degrees=True).as_matrix()
            # 실제 접근 위치 = 쓰레기통 위에서 일정 높이 위로
            grasp_pos = target_pos_base.copy()
            grasp_pos[2] += 0.05  # 5cm 위에서 접근
        else:
            # 앞에서 수평으로 접근하는 grasp
            rot = np.eye(3)
            grasp_pos = target_pos_base.copy()

        return grasp_pos, rot
```

> **주의**: `target_frame_name` ("gripper_frame_link")은 보유하신 URDF의 실제 프레임 이름으로 변경해야 합니다.  
> URDF에서 `<link name="...">` 으로 end-effector 프레임 이름을 확인하세요.

---

## Step 5: 메인 파지 루프 (`grasp_controller.py`)

```python
# grasp_controller.py
import time
import numpy as np
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

from detect_target import get_trash_can_3d
from ik_solver import IKSolver

# ─── 설정 ──────────────────────────────────────────────
ROBOT_PORT = "/dev/ttyUSB0"
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# Hand-Eye 캘리브레이션 결과 로드
cal = np.load("calibration/hand_eye_result.npz")
T_BASE_CAM = cal["T_BASE_CAM"]  # 4x4: 카메라 → 로봇 베이스

# 안전 파라미터
MAX_RELATIVE_TARGET = 5.0   # 한 스텝 최대 관절 이동량 [deg]
GRASP_OPEN  = 0.0            # 그리퍼 열림 (0~100%)
GRASP_CLOSE = 60.0           # 그리퍼 닫힘
MOVE_SLEEP  = 0.05           # 명령 사이 대기 [s]
# ────────────────────────────────────────────────────────

def cam_to_base(point_cam: np.ndarray) -> np.ndarray:
    """카메라 프레임 3D 좌표 → 로봇 베이스 프레임 변환."""
    p_hom = np.append(point_cam, 1.0)
    p_base = T_BASE_CAM @ p_hom
    return p_base[:3]

def move_to_joints(robot, target_joints: dict, steps: int = 20):
    """현재 위치에서 목표 관절각으로 부드럽게 이동 (interpolation)."""
    obs = robot.get_observation()
    current = {k: obs[k] for k in obs if k.endswith(".pos") and not k.startswith("gripper")}

    for i in range(1, steps + 1):
        alpha = i / steps
        interp = {}
        for key in target_joints:
            if key != "gripper.pos":
                interp[key] = current[key] + alpha * (target_joints[key] - current[key])
        # 그리퍼는 마지막 스텝에서만
        if i == steps and "gripper.pos" in target_joints:
            interp["gripper.pos"] = target_joints["gripper.pos"]
        robot.send_action(interp)
        time.sleep(MOVE_SLEEP)

def main():
    # 1. 로봇 연결
    config = SOFollowerRobotConfig(
        port=ROBOT_PORT,
        max_relative_target=MAX_RELATIVE_TARGET,
    )
    robot = SOFollower(config)
    robot.connect(calibrate=False)
    ik = IKSolver()

    print("=== 파지 시작 ===")

    try:
        # 2. 그리퍼 열기 (홈 포지션으로 이동)
        home = {f"{j}.pos": 0.0 for j in JOINT_NAMES}
        home["gripper.pos"] = GRASP_OPEN
        move_to_joints(robot, home, steps=30)
        print("홈 포지션 이동 완료")

        # 3. 쓰레기통 3D 좌표 검출 (카메라 프레임)
        print("쓰레기통 검출 중...")
        point_cam = get_trash_can_3d()
        if point_cam is None:
            print("쓰레기통을 찾지 못했습니다.")
            return

        print(f"검출된 좌표 (카메라): {point_cam}")

        # 4. 카메라 프레임 → 로봇 베이스 프레임
        point_base = cam_to_base(point_cam)
        print(f"변환된 좌표 (베이스): {point_base}")

        # 5. 현재 관절 상태 읽기
        obs = robot.get_observation()
        current_joints = np.array([obs[f"{j}.pos"] for j in JOINT_NAMES])

        # 6. 파지 pose 계산 (위에서 접근)
        grasp_pos, grasp_rot = ik.compute_grasp_pose(point_base, approach_axis="z")

        # 7. IK 풀기
        q_target = ik.solve(grasp_pos, grasp_rot, current_joints)
        if q_target is None:
            print("IK 수렴 실패. 작업 공간을 확인하세요.")
            return

        print(f"목표 관절각 [deg]: {q_target}")

        # 8. 접근 위치로 이동 (그리퍼 열린 상태)
        approach_action = {f"{j}.pos": float(q_target[i]) for i, j in enumerate(JOINT_NAMES)}
        approach_action["gripper.pos"] = GRASP_OPEN
        move_to_joints(robot, approach_action, steps=40)
        print("접근 위치 이동 완료")
        time.sleep(0.5)

        # 9. 수직 하강 (Z축으로 내려가며 실제 파지 위치로)
        grasp_pos_down = grasp_pos.copy()
        grasp_pos_down[2] -= 0.05  # 5cm 내려오기
        q_down = ik.solve(grasp_pos_down, grasp_rot, q_target)
        if q_down is not None:
            down_action = {f"{j}.pos": float(q_down[i]) for i, j in enumerate(JOINT_NAMES)}
            down_action["gripper.pos"] = GRASP_OPEN
            move_to_joints(robot, down_action, steps=15)
        time.sleep(0.3)

        # 10. 그리퍼 닫기 (파지)
        close_action = {f"{j}.pos": float(q_down[i]) if q_down is not None else float(q_target[i])
                        for i, j in enumerate(JOINT_NAMES)}
        close_action["gripper.pos"] = GRASP_CLOSE
        robot.send_action(close_action)
        time.sleep(1.0)
        print("파지 완료!")

        # 11. 들어올리기
        lift_pos = grasp_pos.copy()
        lift_pos[2] += 0.10  # 10cm 들어올리기
        q_lift = ik.solve(lift_pos, grasp_rot, q_down if q_down is not None else q_target)
        if q_lift is not None:
            lift_action = {f"{j}.pos": float(q_lift[i]) for i, j in enumerate(JOINT_NAMES)}
            lift_action["gripper.pos"] = GRASP_CLOSE
            move_to_joints(robot, lift_action, steps=20)
            print("들어올리기 완료!")

    finally:
        robot.disconnect()

if __name__ == "__main__":
    main()
```

---

## 체크리스트 및 주의사항

### URDF 확인 필수
- `so_arm101.urdf`에서 end-effector 링크 이름 확인 → `EEF_FRAME` 변수에 반영
  ```bash
  grep -i "gripper\|end_effector\|tool" so_arm101.urdf | grep "<link"
  ```
- 로봇 베이스 프레임이 고정되어 있는지 확인 (placo는 `mask_fbase(True)` 사용)

### Hand-Eye Calibration 품질 검증
```python
# 검증: 알려진 위치의 보드 재검출 후 오차 확인
# 재투영 오차 < 5mm 이면 양호
error_mm = np.linalg.norm(predicted_pos - actual_pos) * 1000
print(f"재투영 오차: {error_mm:.1f} mm")
```

### 작업 공간 확인
- SO-ARM101의 작업 반경 (~40cm) 내에 쓰레기통 위치
- IK가 실패하면 쓰레기통이 작업 공간 밖일 가능성 있음

### 안전
- 첫 실행 시 `MAX_RELATIVE_TARGET = 5.0` (5도/스텝) 으로 낮게 유지
- `move_to_joints`의 `steps` 파라미터를 크게 설정할수록 느리고 안전
- 로봇 주변 장애물 제거 후 실행

---

## 실행 순서 요약

```bash
# 1. 관절 캘리브레이션 (처음 한 번만)
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyUSB0

# 2. Hand-Eye 데이터 수집 (처음 한 번만, 카메라 위치 변경 시 재수집)
python calibration/collect_handeye_data.py

# 3. 변환행렬 계산
python calibration/run_handeye_calibration.py

# 4. 파지 실행
python grasp/grasp_controller.py
```
