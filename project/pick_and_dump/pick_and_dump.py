"""
쓰레기통 파지 → 들어올리기 → 비우기(wrist_roll) → 제자리에 내려놓기

에러 복구 포함:
  - IK 실패 시 재시도 (오차 허용 확대)
  - 파지 실패 시 그리퍼 열고 재시도
  - 어떤 Phase에서든 실패 시 안전 자세로 복귀
  - 웨이포인트 도달 확인

사용법:
  cd ~/lerobot2/project
  python pick_and_dump/pick_and_dump.py
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "grasp"))

import time
import numpy as np
import cv2
import pyrealsense2 as rs

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.model.kinematics import RobotKinematics

from detect_target import get_trash_can_3d_cam, get_trash_can_3d_cam_with_height
from coord_transform import cam_to_base, JOINT_NAMES
from ik_solver import IKSolver

# ─── 설정 ──────────────────────────────────────────────
FOLLOWER_PORT = "/dev/ttyACM1"
URDF_PATH = "so101_new_calib.urdf"
EEF_FRAME = "gripper_frame_link"
MAX_REL_TARGET = 5.0

GRASP_OPEN = 100.0
GRASP_CLOSE = -10.0
GRASP_Y_OFFSET = -0.01
GRASP_X_OFFSET = 0.04
WRIST_FLEX_EXTRA = 10
GRASP_Z_TARGET = 0.03

DUMP_ROLL_ANGLE = -160.0   # wrist_roll 회전량 [deg] (반대 방향으로 비우기)

# 에러 복구 설정
GRASP_CHECK_THRESHOLD = 3.0   # 그리퍼 닫힘 판정 기준 [deg] (GRASP_CLOSE 대비)
MAX_GRASP_RETRIES = 2         # 파지 재시도 횟수
IK_RETRY_THRESHOLD = 40.0     # IK 재시도 시 확대된 오차 허용 [mm]
JOINT_ARRIVAL_THRESHOLD = 8.0 # 웨이포인트 도달 판정 기준 [deg]

# 안전 자세 (팔을 세운 중립 자세)
SAFE_POSE = {
    "shoulder_pan.pos": -13.67,
    "shoulder_lift.pos": -102.07,
    "elbow_flex.pos": 35.21,
    "wrist_flex.pos": 104.53,
    "wrist_roll.pos": -0.75,
    "gripper.pos": GRASP_OPEN,
}

# 시연 기반 웨이포인트 (2026-04-16 실측)
# WP1: 들어올리기 (팔 접어서 위로)
# WP2: 베이스 우측 회전 (비울 위치)
DUMP_WAYPOINTS = [
    {"shoulder_pan": -9.89, "shoulder_lift": -28.48, "elbow_flex": -64.48,
     "wrist_flex": 88.0, "wrist_roll": 0.66},
    {"shoulder_pan": 85.05, "shoulder_lift": -17.27, "elbow_flex": -52.70,
     "wrist_flex": 70.07, "wrist_roll": 3.0},
]
# ────────────────────────────────────────────────────────


def move_joints_smooth(robot, target_joints: dict, steps: int = 30):
    """현재 자세에서 목표 관절각으로 선형 보간해서 이동합니다."""
    obs = robot.get_observation()
    current = {k: obs[k] for k in target_joints}

    for i in range(1, steps + 1):
        alpha = i / steps
        cmd = {
            k: current[k] + alpha * (target_joints[k] - current[k])
            for k in target_joints
        }
        robot.send_action(cmd)
        update_camera_display()
        time.sleep(0.04)


def check_joints_arrived(robot, target_joints: dict, threshold: float = JOINT_ARRIVAL_THRESHOLD) -> bool:
    """목표 관절값에 도달했는지 확인합니다."""
    obs = robot.get_observation()
    for k, v in target_joints.items():
        if ".pos" not in k or "gripper" in k:
            continue
        actual = obs.get(k, 0.0)
        if abs(actual - v) > threshold:
            print(f"  [도달 확인] {k}: 목표={v:.1f}, 실제={actual:.1f}, 차이={abs(actual-v):.1f}° > {threshold}°")
            return False
    return True


def check_grasp_success(robot) -> bool:
    """그리퍼가 물체를 잡았는지 확인합니다.
    GRASP_CLOSE로 명령했지만 물체가 있으면 완전히 닫히지 않아
    그리퍼 값이 GRASP_CLOSE보다 큰 값을 보입니다."""
    obs = robot.get_observation()
    gripper_pos = obs["gripper.pos"]
    # 물체를 잡았으면 그리퍼가 완전히 닫히지 않음 (GRASP_CLOSE보다 큼)
    grasped = gripper_pos > (GRASP_CLOSE + GRASP_CHECK_THRESHOLD)
    print(f"  [파지 확인] gripper={gripper_pos:.1f}, 기준={GRASP_CLOSE + GRASP_CHECK_THRESHOLD:.1f}, 결과={'성공' if grasped else '실패'}")
    return grasped


def safe_return(robot, q_grasp_pose=None, waypoint_index=-1):
    """에러 발생 시 안전 자세로 복귀합니다.

    Args:
        robot: 팔로워 로봇
        q_grasp_pose: 파지 자세 (있으면 물체를 잡고 있으므로 내려놓기 시도)
        waypoint_index: 현재 진행 중인 웨이포인트 인덱스 (역순 복귀용)
                        -1이면 웨이포인트 복귀 없이 안전 자세로만 이동
    """
    print("\n[에러 복구] 안전 자세로 복귀 중...")

    # 웨이포인트를 따라왔다면 역순으로 복귀
    if waypoint_index >= 0:
        for i in range(waypoint_index, -1, -1):
            wp = DUMP_WAYPOINTS[i]
            print(f"  웨이포인트 {i + 1} 복귀 중...")
            wp_cmd = {f"{j}.pos": float(wp[j]) for j in JOINT_NAMES}
            wp_cmd["gripper.pos"] = GRASP_CLOSE if q_grasp_pose else GRASP_OPEN
            move_joints_smooth(robot, wp_cmd, steps=80)
            time.sleep(1.0)

    # 물체를 잡고 있으면 내려놓기 시도
    if q_grasp_pose is not None:
        print("  물체 내려놓기 시도...")
        place_cmd = dict(q_grasp_pose)
        place_cmd["gripper.pos"] = GRASP_CLOSE
        move_joints_smooth(robot, place_cmd, steps=80)
        time.sleep(1.0)

        # 그리퍼 열기
        obs = robot.get_observation()
        open_cmd = {k: obs[k] for k in obs if ".pos" in k}
        open_cmd["gripper.pos"] = GRASP_OPEN
        move_joints_smooth(robot, open_cmd, steps=20)
        time.sleep(0.5)

    # 안전 자세로 이동
    print("  안전 자세로 이동...")
    move_joints_smooth(robot, SAFE_POSE, steps=80)
    time.sleep(1.0)
    print("[에러 복구] 안전 자세 복귀 완료")


# 카메라 표시용 전역 변수
_camera_pipeline = None
_camera_align = None
_camera_phase = ""


def update_camera_display():
    """카메라 프레임을 한 번 읽어서 화면에 표시합니다."""
    global _camera_pipeline, _camera_align, _camera_phase
    if _camera_pipeline is None:
        return
    try:
        frames = _camera_pipeline.wait_for_frames(timeout_ms=30)
        aligned = _camera_align.process(frames)
        color_frame = aligned.get_color_frame()
        if color_frame:
            vis = np.asanyarray(color_frame.get_data())
            vis = cv2.cvtColor(vis, cv2.COLOR_RGB2BGR)
            if _camera_phase:
                cv2.putText(vis, _camera_phase, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("Pick and Dump", vis)
            cv2.waitKey(1)
    except Exception:
        pass


def main():
    # ── 로봇 연결 ─────────────────────────────────────────
    follower = SO101Follower(
        SO101FollowerConfig(
            port=FOLLOWER_PORT,
            max_relative_target=MAX_REL_TARGET,
        )
    )
    follower.connect()

    kin = RobotKinematics(
        urdf_path=URDF_PATH,
        target_frame_name=EEF_FRAME,
        joint_names=JOINT_NAMES,
    )
    ik = IKSolver()

    # ── D405 연결 ─────────────────────────────────────────
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    color_intr = (
        profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    )

    for _ in range(30):
        pipeline.wait_for_frames()

    # 카메라 표시용 전역 변수 설정
    global _camera_pipeline, _camera_align, _camera_phase
    _camera_pipeline = pipeline
    _camera_align = align

    print("=" * 50)
    print("쓰레기통 파지 → 비우기 → 제자리 놓기")
    print("=" * 50)

    # 에러 복구용 상태 추적
    q_grasp_pose = None       # 파지 자세 (내려놓기용)
    current_wp_index = -1     # 현재 웨이포인트 진행 상태

    try:
        # SAFE_POSE로 초기 위치 이동
        print("초기 자세로 이동 중...")
        move_joints_smooth(follower, SAFE_POSE, steps=80)
        time.sleep(1.0)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # PHASE 1: 검출 + 파지 (grasp_controller.py와 동일)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        print("\n[Phase 1] 쓰레기통 검출 중... (ENTER=파지, q=종료)")
        point_cam = None
        object_height = None
        while True:
            height_result = get_trash_can_3d_cam_with_height(pipeline, align, depth_scale, color_intr)

            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            if color_frame:
                vis = np.asanyarray(color_frame.get_data())
                vis = cv2.cvtColor(vis, cv2.COLOR_RGB2BGR)
                if height_result is not None:
                    point_cam = height_result[0]  # center_3d
                    object_height = height_result[3]  # object_height_m
                    cv2.putText(vis, f"DETECTED h={object_height*100:.1f}cm", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(vis, "ENTER to grasp, q to quit", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    cv2.putText(vis, "NOT DETECTED - move arm", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("Pick and Dump", vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("종료합니다.")
                return
            elif key == 13 and point_cam is not None:
                break

        # Phase 1 이후부터 카메라 계속 표시
        _camera_phase = "GRASPING..."

        if point_cam is None:
            print("쓰레기통을 검출하지 못했습니다.")
            return

        print(f"카메라 프레임 좌표: {point_cam}")
        print(f"물체 높이: {object_height*100:.1f}cm")

        # 현재 관절각
        obs = follower.get_observation()
        q_current = np.array([obs[f"{j}.pos"] for j in JOINT_NAMES])

        # 좌표 변환
        point_base = cam_to_base(point_cam, q_current, kin)
        print(f"베이스 프레임 좌표 (원본): {point_base}")
        point_base[1] += GRASP_Y_OFFSET
        print(f"베이스 프레임 좌표 (오프셋): {point_base}")

        # 파지 높이를 물체 중간으로 동적 설정
        if object_height is not None and object_height > 0.02:
            grasp_z = object_height * 0.7
            grasp_z = max(0.02, min(0.10, grasp_z))
            print(f"파지 높이 (물체 중간): {grasp_z*100:.1f}cm")
        else:
            grasp_z = GRASP_Z_TARGET
            print(f"파지 높이 (기본값): {grasp_z*100:.1f}cm")

        # 관절 인덱스
        SP = JOINT_NAMES.index("shoulder_pan")
        SL = JOINT_NAMES.index("shoulder_lift")
        EF = JOINT_NAMES.index("elbow_flex")
        WF = JOINT_NAMES.index("wrist_flex")
        WR = JOINT_NAMES.index("wrist_roll")

        # pitch_sum 계산
        grasp_final_pos = point_base.copy()
        grasp_final_pos[0] += GRASP_X_OFFSET
        grasp_final_pos[2] = grasp_z
        q_grasp_preview = ik.solve(grasp_final_pos, q_current)

        if q_grasp_preview is not None:
            target_pitch_sum = q_grasp_preview[SL] + q_grasp_preview[EF] + q_grasp_preview[WF] + WRIST_FLEX_EXTRA - 10
            print(f"최종 파지 IK 기반 pitch_sum: {target_pitch_sum:.1f}")
        else:
            target_pitch_sum = 19.0 + WRIST_FLEX_EXTRA - 10
            print(f"실측값 기반 pitch_sum: {target_pitch_sum:.1f}")

        # 팔 세우기 (shoulder_lift ~90도)
        print("팔 세우기 중...")
        q_upright = q_current.copy()
        if q_grasp_preview is not None:
            q_upright[SP] = q_grasp_preview[SP]
        q_upright[SL] = 90.0
        q_upright[EF] = 0.0
        q_upright[WR] = 0.0
        q_upright[WF] = target_pitch_sum - q_upright[SL] - q_upright[EF]
        print(f"세운 자세: SL={q_upright[SL]:.0f}, EF={q_upright[EF]:.0f}, WF={q_upright[WF]:.1f}")

        upright_cmd = {
            f"{j}.pos": float(q_upright[i]) for i, j in enumerate(JOINT_NAMES)
        }
        upright_cmd["gripper.pos"] = GRASP_OPEN
        move_joints_smooth(follower, upright_cmd, steps=40)
        print("팔 세우기 + 각도 세팅 완료")
        time.sleep(0.5)

        # 목표 위로 이동 (세운 상태에서 XY 위치 맞춤 + 약간 내려감)
        above_pos = point_base.copy()
        above_pos[2] += 0.15
        print("목표 위 접근 IK 계산 중...")
        q_above = ik.solve(above_pos, q_upright)
        if q_above is None:
            print("접근 위치 IK 실패. 오차 허용 확대하여 재시도...")
            q_above = ik.solve(above_pos, q_upright, err_threshold_mm=IK_RETRY_THRESHOLD)
        if q_above is None:
            print("[실패] 접근 위치 IK 최종 실패.")
            safe_return(follower)
            return
        q_above[WF] = target_pitch_sum - q_above[SL] - q_above[EF]
        print(f"목표 위 wrist_flex 보정: {q_above[WF]:.1f}도")

        above_cmd = {
            f"{j}.pos": float(q_above[i]) for i, j in enumerate(JOINT_NAMES)
        }
        above_cmd["gripper.pos"] = GRASP_OPEN
        move_joints_smooth(follower, above_cmd, steps=30)
        print("목표 위 접근 완료")
        time.sleep(0.3)

        # 수직 하강 (pitch_sum 보정으로 그리퍼 각도 유지)
        print("수직 하강 IK 계산 중...")
        lower_pos = point_base.copy()
        lower_pos[2] = grasp_z
        q_down = ik.solve(lower_pos, q_above)
        if q_down is None:
            q_down = ik.solve(lower_pos, q_above, err_threshold_mm=IK_RETRY_THRESHOLD)
        if q_down is not None:
            q_down[WF] = target_pitch_sum - q_down[SL] - q_down[EF]
            print(f"하강 wrist_flex 보정: {q_down[WF]:.1f}도 (pitch_sum={target_pitch_sum:.1f})")
            down_cmd = {
                f"{j}.pos": float(q_down[i]) for i, j in enumerate(JOINT_NAMES)
            }
            down_cmd["gripper.pos"] = GRASP_OPEN
            move_joints_smooth(follower, down_cmd, steps=25)
            print("하강 완료")
        else:
            print("하강 IK 실패, 접근 위치에서 파지 시도")
            q_down = q_above
        time.sleep(0.3)

        # 전진 (물체 안쪽으로, pitch_sum 보정 유지)
        print("전진 중...")
        grasp_pos = point_base.copy()
        grasp_pos[0] += GRASP_X_OFFSET
        grasp_pos[2] = grasp_z
        q_forward = ik.solve(grasp_pos, q_down)
        if q_forward is not None:
            q_forward[WF] = target_pitch_sum - q_forward[SL] - q_forward[EF]
            print(f"전진 wrist_flex 보정: {q_forward[WF]:.1f}도")
            fwd_cmd = {
                f"{j}.pos": float(q_forward[i]) for i, j in enumerate(JOINT_NAMES)
            }
            fwd_cmd["gripper.pos"] = GRASP_OPEN
            move_joints_smooth(follower, fwd_cmd, steps=15)
            print("전진 완료")
            q_down = q_forward
        time.sleep(0.3)

        # 그리퍼 닫기 (관절 고정, 그리퍼만 닫기)
        print("그리퍼 닫기...")
        obs_before_close = follower.get_observation()
        close_cmd = {k: obs_before_close[k] for k in obs_before_close if ".pos" in k}
        close_cmd["gripper.pos"] = GRASP_CLOSE
        move_joints_smooth(follower, close_cmd, steps=30)
        time.sleep(0.5)
        print("파지 완료!")

        # 파지 성공 확인
        grasp_succeeded = check_grasp_success(follower)
        if not grasp_succeeded:
            print("\n[실패] 파지 실패. 안전 자세로 복귀합니다.")
            obs_fail = follower.get_observation()
            open_fail = {k: obs_fail[k] for k in obs_fail if ".pos" in k}
            open_fail["gripper.pos"] = GRASP_OPEN
            move_joints_smooth(follower, open_fail, steps=20)
            time.sleep(0.3)
            safe_return(follower)
            return

        # 파지 직후 관절값 저장 (내려놓기 시 동일 높이 재현용)
        obs_grasp = follower.get_observation()
        q_grasp_pose = {k: obs_grasp[k] for k in obs_grasp if ".pos" in k}
        print(f"  파지 자세 저장: { {k: round(v,1) for k,v in q_grasp_pose.items()} }")

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # PHASE 2: 시연 웨이포인트 순서대로 이동 (비울 위치까지)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        follower.config.max_relative_target = 10.0
        _camera_phase = "Phase 2: MOVING TO DUMP"
        print("\n[Phase 2] 웨이포인트 따라 이동 중...")

        for i, wp in enumerate(DUMP_WAYPOINTS):
            current_wp_index = i
            print(f"  웨이포인트 {i + 1}/{len(DUMP_WAYPOINTS)} 이동 중...")
            wp_cmd = {f"{j}.pos": float(wp[j]) for j in JOINT_NAMES}
            wp_cmd["gripper.pos"] = GRASP_CLOSE
            move_joints_smooth(follower, wp_cmd, steps=80)
            time.sleep(1.0)

            # 웨이포인트 도달 확인 (경고만, 중단하지 않음)
            if not check_joints_arrived(follower, wp_cmd):
                print(f"  [경고] 웨이포인트 {i + 1} 미도달. 재전송...")
                move_joints_smooth(follower, wp_cmd, steps=40)
                time.sleep(1.0)

        print("비울 위치 도착")
        time.sleep(1.0)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # PHASE 3: wrist_roll 회전으로 쓰레기 비우기
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        _camera_phase = "Phase 3: DUMPING"
        print("\n[Phase 3] 쓰레기 비우기 (wrist_roll 회전)...")
        obs_at_dump = follower.get_observation()
        wrist_roll_before = obs_at_dump["wrist_roll.pos"]

        dump_roll_cmd = {k: obs_at_dump[k] for k in obs_at_dump if ".pos" in k}
        dump_roll_cmd["wrist_roll.pos"] = wrist_roll_before + DUMP_ROLL_ANGLE
        dump_roll_cmd["gripper.pos"] = GRASP_CLOSE
        move_joints_smooth(follower, dump_roll_cmd, steps=30)
        time.sleep(1.0)
        print("비우기 완료!")

        # wrist_roll 원복
        print("wrist_roll 원복 중...")
        obs_after_dump = follower.get_observation()
        restore_roll_cmd = {k: obs_after_dump[k] for k in obs_after_dump if ".pos" in k}
        restore_roll_cmd["wrist_roll.pos"] = wrist_roll_before
        restore_roll_cmd["gripper.pos"] = GRASP_CLOSE
        move_joints_smooth(follower, restore_roll_cmd, steps=30)
        time.sleep(0.3)
        print("wrist_roll 원복 완료")

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # PHASE 4: 역순 복귀 → 제자리 내려놓기 → 그리퍼 열기
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        _camera_phase = "Phase 4: RETURNING"
        print("\n[Phase 4] 역순 복귀 중...")

        # 웨이포인트 역순으로 복귀
        for i, wp in enumerate(reversed(DUMP_WAYPOINTS)):
            idx = len(DUMP_WAYPOINTS) - i
            print(f"  웨이포인트 {idx} 복귀 중...")
            wp_cmd = {f"{j}.pos": float(wp[j]) for j in JOINT_NAMES}
            wp_cmd["gripper.pos"] = GRASP_CLOSE
            move_joints_smooth(follower, wp_cmd, steps=80)
            time.sleep(1.0)

        print("원래 위치 복귀 완료")
        time.sleep(0.3)

        # 하강 (파지 시 저장한 관절값으로 동일 높이 재현)
        print("내려놓기 중...")
        place_cmd = dict(q_grasp_pose)
        place_cmd["gripper.pos"] = GRASP_CLOSE
        move_joints_smooth(follower, place_cmd, steps=80)
        print("내려놓기 위치 도달")
        time.sleep(1.0)

        # 그리퍼 열기 (관절 고정)
        print("그리퍼 열기...")
        obs_before_open = follower.get_observation()
        open_cmd = {k: obs_before_open[k] for k in obs_before_open if ".pos" in k}
        open_cmd["gripper.pos"] = GRASP_OPEN
        move_joints_smooth(follower, open_cmd, steps=20)
        time.sleep(0.5)

        # 홈 자세로 복귀
        _camera_phase = "RETURNING HOME"
        print("홈 자세로 복귀 중...")
        move_joints_smooth(follower, SAFE_POSE, steps=80)
        time.sleep(1.0)

        print("\n" + "=" * 50)
        print("완료! 쓰레기통 파지 → 비우기 → 제자리 놓기 → 홈 복귀 성공")
        print("=" * 50)

    except Exception as e:
        print(f"\n[예외 발생] {type(e).__name__}: {e}")
        print("안전 복귀를 시도합니다...")
        try:
            safe_return(follower, q_grasp_pose, current_wp_index)
        except Exception as e2:
            print(f"[안전 복귀 실패] {type(e2).__name__}: {e2}")
            print("그리퍼만 열기 시도...")
            try:
                obs_emergency = follower.get_observation()
                emergency_cmd = {k: obs_emergency[k] for k in obs_emergency if ".pos" in k}
                emergency_cmd["gripper.pos"] = GRASP_OPEN
                follower.send_action(emergency_cmd)
            except Exception:
                pass

    finally:
        cv2.destroyAllWindows()
        pipeline.stop()
        follower.disconnect()


if __name__ == "__main__":
    main()
