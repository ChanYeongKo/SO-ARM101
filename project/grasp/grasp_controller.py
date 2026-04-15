"""
메인 파지 루프

D405로 쓰레기통 검출 -> 좌표 변환 -> IK -> 접근 -> 파지 -> 들어올리기

사용법:
  cd project
  python grasp/grasp_controller.py
"""

import time
import numpy as np
import cv2
import pyrealsense2 as rs

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.model.kinematics import RobotKinematics

from detect_target import get_trash_can_3d_cam
from coord_transform import cam_to_base, JOINT_NAMES
from ik_solver import IKSolver

# ─── 설정 ──────────────────────────────────────────────
FOLLOWER_PORT = "/dev/ttyACM1"
LEADER_PORT = "/dev/ttyACM0"
URDF_PATH = "so101_new_calib.urdf"
EEF_FRAME = "gripper_frame_link"
MAX_REL_TARGET = 5.0  # 한 스텝 최대 관절 이동량 [deg]
GRASP_OPEN = 100.0  # 그리퍼 열림 (0~100, 100=최대 열림)
GRASP_CLOSE = 0.0  # 그리퍼 닫힘
GRASP_Y_OFFSET = 0.03  # 파지 위치 Y축 오프셋 [m] (양수=왼쪽, 음수=오른쪽)
GRASP_X_OFFSET = 0.04  # 파지 전 전진 오프셋 [m]
WRIST_FLEX_EXTRA = -15  # wrist_flex 추가 보정 [deg] (음수=위로 꺾기)
GRASP_Z_LOWER = 0.17   # 파지 시 추가 하강 [m]
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
        time.sleep(0.05)


def main():
    # ── 로봇 연결 ─────────────────────────────────────────
    follower = SO101Follower(
        SO101FollowerConfig(
            port=FOLLOWER_PORT,
            max_relative_target=MAX_REL_TARGET,
        )
    )
    follower.connect()

    leader = SO101Leader(SO101LeaderConfig(port=LEADER_PORT))
    leader.connect()

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

    # 카메라 안정화 대기
    for _ in range(30):
        pipeline.wait_for_frames()

    print("=== 파지 시작 ===")
    print("리더암으로 카메라가 쓰레기통을 볼 수 있도록 위치를 잡으세요.")

    try:
        # 1. 쓰레기통 검출 (카메라 프레임 3D 좌표)
        print("쓰레기통 검출 중... (화면에서 검출 상태 확인, ENTER로 파지 시작, q로 종료)")
        point_cam = None
        while True:
            # 리더암 -> 팔로워암 텔레오퍼레이션 유지하면서 검출
            action = leader.get_action()
            follower.send_action(action)

            # 검출 시도
            candidate = get_trash_can_3d_cam(pipeline, align, depth_scale, color_intr)

            # 시각화: 현재 카메라 영상 표시
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            if color_frame:
                vis = np.asanyarray(color_frame.get_data())
                vis = cv2.cvtColor(vis, cv2.COLOR_RGB2BGR)
                if candidate is not None:
                    point_cam = candidate
                    cv2.putText(vis, f"DETECTED: {point_cam}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(vis, "ENTER to grasp, q to quit", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    cv2.putText(vis, "NOT DETECTED - move arm", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("Grasp Controller", vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("종료합니다.")
                return
            elif key == 13 and point_cam is not None:  # ENTER
                break

        cv2.destroyAllWindows()

        if point_cam is None:
            print("쓰레기통을 검출하지 못했습니다.")
            return

        print(f"카메라 프레임 좌표: {point_cam}")

        # 3. 현재 팔로워암 관절각 읽기
        obs = follower.get_observation()
        q_current = np.array([obs[f"{j}.pos"] for j in JOINT_NAMES])

        # 4. 카메라 -> 베이스 좌표 변환 + 파지 오프셋 적용
        point_base = cam_to_base(point_cam, q_current, kin)
        print(f"베이스 프레임 좌표 (원본): {point_base}")
        point_base[1] += GRASP_Y_OFFSET  # 왼쪽으로 오프셋
        print(f"베이스 프레임 좌표 (오프셋): {point_base}")

        # 5. 목표 바로 위 → 수직 하강 → 후퇴 → wrist_flex 꺾기 → 전진 → 파지

        # 5-1. 목표 바로 위 (목표 XY + 10cm 위)
        above_pos = point_base.copy()
        above_pos[2] += 0.10
        print("목표 위 접근 IK 계산 중...")
        q_above = ik.solve(above_pos, q_current)
        if q_above is None:
            print("접근 위치 IK 실패.")
            return

        above_cmd = {
            f"{j}.pos": float(q_above[i]) for i, j in enumerate(JOINT_NAMES)
        }
        above_cmd["gripper.pos"] = GRASP_OPEN
        move_joints_smooth(follower, above_cmd, steps=30)
        print("목표 위 접근 완료")
        time.sleep(0.3)

        # 5-3. 수직 하강 + wrist_flex 보정
        #
        # 원리: shoulder_lift + elbow_flex + wrist_flex 합이 일정하면
        #       그리퍼의 절대 각도(pitch)가 유지됩니다.
        #       하강 시 shoulder_lift, elbow_flex가 변하면
        #       wrist_flex를 보정해서 그리퍼 각도를 유지합니다.
        SL = JOINT_NAMES.index("shoulder_lift")
        EF = JOINT_NAMES.index("elbow_flex")
        WF = JOINT_NAMES.index("wrist_flex")

        # 접근 위치에서의 pitch 합 기억 + 추가 보정
        pitch_sum_ref = q_above[SL] + q_above[EF] + q_above[WF] + WRIST_FLEX_EXTRA

        print("수직 하강 IK 계산 중...")
        lower_pos = point_base.copy()
        lower_pos[2] -= GRASP_Z_LOWER  # 목표보다 더 아래로 하강
        q_down = ik.solve(lower_pos, q_above)
        if q_down is not None:
            # 하강 후 pitch 보정: wrist_flex = 기준합 - (현재 shoulder_lift + elbow_flex)
            q_down[WF] = pitch_sum_ref - q_down[SL] - q_down[EF]
            print(f"wrist_flex 보정: {q_down[WF]:.1f}도 (pitch합 유지 + {WRIST_FLEX_EXTRA}도)")

            down_cmd = {
                f"{j}.pos": float(q_down[i]) for i, j in enumerate(JOINT_NAMES)
            }
            down_cmd["gripper.pos"] = GRASP_OPEN
            move_joints_smooth(follower, down_cmd, steps=20)
            print("하강 완료")
        else:
            print("하강 IK 실패, 접근 위치에서 파지 시도")
            q_down = q_above
        time.sleep(0.3)

        # 5-4. 앞으로 전진 (물체 안쪽으로)
        print("전진 중...")
        grasp_pos = point_base.copy()
        grasp_pos[0] += GRASP_X_OFFSET  # X축 앞으로
        q_forward = ik.solve(grasp_pos, q_down)
        if q_forward is not None:
            # 전진 후에도 동일하게 보정
            q_forward[WF] = pitch_sum_ref - q_forward[SL] - q_forward[EF]
            print(f"wrist_flex 보정: {q_forward[WF]:.1f}도")

            fwd_cmd = {
                f"{j}.pos": float(q_forward[i]) for i, j in enumerate(JOINT_NAMES)
            }
            fwd_cmd["gripper.pos"] = GRASP_OPEN
            move_joints_smooth(follower, fwd_cmd, steps=15)
            print("전진 완료")
            q_down = q_forward
        time.sleep(0.3)

        # 5-5. 후퇴 (2cm 뒤로) → wrist_flex 꺾기 → 다시 전진
        #      IK로 전체 자세를 계산해서 자연스럽게 후퇴
        retreat_pos = point_base.copy()
        retreat_pos[0] -= 0.02  # X축 뒤로 2cm
        if q_forward is not None:
            retreat_pos[0] += GRASP_X_OFFSET  # 전진한 상태에서 후퇴
        print("후퇴 중 (2cm)...")
        q_retreat = ik.solve(retreat_pos, q_down)
        if q_retreat is not None:
            # 후퇴 시에도 pitch 보정 유지
            q_retreat[WF] = pitch_sum_ref - q_retreat[SL] - q_retreat[EF]
            retreat_cmd = {
                f"{j}.pos": float(q_retreat[i]) for i, j in enumerate(JOINT_NAMES)
            }
            retreat_cmd["gripper.pos"] = GRASP_OPEN
            move_joints_smooth(follower, retreat_cmd, steps=15)
            print("후퇴 완료")
            q_down = q_retreat
        time.sleep(0.2)

        # wrist_flex 꺾기
        q_down[WF] -= 10
        print(f"파지 직전 wrist_flex 올림: {q_down[WF]:.1f}도")
        tilt_cmd = {
            f"{j}.pos": float(q_down[i]) for i, j in enumerate(JOINT_NAMES)
        }
        tilt_cmd["gripper.pos"] = GRASP_OPEN
        move_joints_smooth(follower, tilt_cmd, steps=10)
        time.sleep(0.2)

        # 다시 전진 (원래 파지 위치로)
        print("파지 위치로 전진 중...")
        grasp_final_pos = point_base.copy()
        grasp_final_pos[0] += GRASP_X_OFFSET
        q_grasp_fwd = ik.solve(grasp_final_pos, q_down)
        if q_grasp_fwd is not None:
            # wrist_flex는 꺾은 상태 유지
            q_grasp_fwd[WF] = q_down[WF]
            fwd2_cmd = {
                f"{j}.pos": float(q_grasp_fwd[i]) for i, j in enumerate(JOINT_NAMES)
            }
            fwd2_cmd["gripper.pos"] = GRASP_OPEN
            move_joints_smooth(follower, fwd2_cmd, steps=15)
            print("파지 위치 전진 완료")
            q_down = q_grasp_fwd
        time.sleep(0.2)

        # 6. 그리퍼 닫기 (파지)
        q_final = q_down
        close_cmd = {
            f"{j}.pos": float(q_final[i]) for i, j in enumerate(JOINT_NAMES)
        }
        close_cmd["gripper.pos"] = GRASP_CLOSE
        move_joints_smooth(follower, close_cmd, steps=30)
        time.sleep(0.5)
        print("파지 완료!")

        # 7. 들어올리기
        lift_pos = point_base.copy()
        lift_pos[2] = 0.20
        print("들어올리기 IK 계산 중...")
        q_lift = ik.solve(lift_pos, q_final)
        if q_lift is not None:
            lift_cmd = {
                f"{j}.pos": float(q_lift[i]) for i, j in enumerate(JOINT_NAMES)
            }
            lift_cmd["gripper.pos"] = GRASP_CLOSE
            move_joints_smooth(follower, lift_cmd, steps=20)
            print("들어올리기 완료!")

    finally:
        pipeline.stop()
        follower.disconnect()
        leader.disconnect()


if __name__ == "__main__":
    main()
