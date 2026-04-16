""" 
Hand-Eye Calibration 데이터 수집 (eye-in-hand)

바닥에 ChArUco 보드를 고정하고, 리더암으로 팔로워암을 다양한 자세로 움직이면서
카메라-보드 pose + FK(관절각) 데이터를 수집합니다.

사용법:
  python calibration/step2_collect_handeye.py

조작:
  - 리더암을 잡고 팔로워암을 움직임
  - 보드가 초록색으로 검출되면 ENTER로 저장
  - 최소 15개 이상, 다양한 각도/거리에서 수집
  - 'q'로 종료
"""

import numpy as np
import cv2
import pyrealsense2 as rs

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.model.kinematics import RobotKinematics

# ─── 설정 ──────────────────────────────────────────────
FOLLOWER_PORT = "/dev/ttyACM1"
LEADER_PORT = "/dev/ttyACM0"
URDF_PATH = "so101_new_calib.urdf"
EEF_FRAME = "gripper_frame_link"
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# ChArUco 보드 설정 (인쇄 후 실측값으로 수정)
SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LENGTH = 0.03 # 40mm
MARKER_LENGTH = 0.022  # 30mm
# ────────────────────────────────────────────────────────


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
    obj_points, img_points = board.matchImagePoints(corners, ids)
    if obj_points is None or len(obj_points) < 6:
        return None, None
    ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
    if not ok:
        return None, None
    return rvec, tvec


def main():
    # ── 로봇 연결 ─────────────────────────────────────────
    follower = SO101Follower(SO101FollowerConfig(port=FOLLOWER_PORT))
    follower.connect()

    leader = SO101Leader(SO101LeaderConfig(port=LEADER_PORT))
    leader.connect()

    kin = RobotKinematics(
        urdf_path=URDF_PATH,
        target_frame_name=EEF_FRAME,
        joint_names=JOINT_NAMES,
    )

    # ── D405 연결 ─────────────────────────────────────────
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    profile = pipeline.start(cfg)
    # 카메라 안정화 대기
    for _ in range(30):
        pipeline.wait_for_frames()

    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    camera_matrix = np.array([
        [intr.fx, 0, intr.ppx],
        [0, intr.fy, intr.ppy],
        [0, 0, 1],
    ])
    dist_coeffs = np.array(intr.coeffs)

    # ── 수집 루프 ─────────────────────────────────────────
    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list = []
    t_target2cam_list = []

    print("=" * 50)
    print("Hand-Eye Calibration 데이터 수집")
    print("리더암을 잡고 팔로워암을 원하는 자세로 이동하세요.")
    print("보드가 검출되면 ENTER로 저장, 'q'로 종료.")
    print("최소 15개 이상, 다양한 방향/거리에서 수집하세요.")
    print("=" * 50)

    try:
        while True:
            # 리더암 → 팔로워암 텔레오퍼레이션
            action = leader.get_action()
            follower.send_action(action)

            # D405 프레임 읽기
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)

            rvec, tvec = detect_board_pose(gray, camera_matrix, dist_coeffs)

            # 시각화
            vis = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            status = f"saved: {len(R_gripper2base_list)}"
            if rvec is not None:
                cv2.drawFrameAxes(vis, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                cv2.putText(vis, f"[DETECTED] {status}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(vis, f"[NOT DETECTED] {status}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("Hand-Eye Calibration", vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == 13:  # ENTER
                if rvec is None:
                    print("보드가 검출되지 않았습니다. 보드가 카메라에 보이는지 확인하세요.")
                    continue

                # 팔로워암 현재 관절각 → FK → T_base→gripper
                obs = follower.get_observation()
                q = np.array([obs[f"{name}.pos"] for name in JOINT_NAMES])
                T_base_gripper = kin.forward_kinematics(q)

                R_g2b = T_base_gripper[:3, :3]
                t_g2b = T_base_gripper[:3, 3].reshape(3, 1)
                R_t2c, _ = cv2.Rodrigues(rvec)

                R_gripper2base_list.append(R_g2b)
                t_gripper2base_list.append(t_g2b)
                R_target2cam_list.append(R_t2c)
                t_target2cam_list.append(tvec)
                print(
                    f"[{len(R_gripper2base_list)}] 저장 완료. "
                    f"그리퍼 위치: {t_g2b.flatten()}, 보드 거리: {tvec[2][0]:.3f}m"
                )
    finally:
        pipeline.stop()
        follower.disconnect()
        leader.disconnect()
        cv2.destroyAllWindows()

    if len(R_gripper2base_list) < 3:
        print("데이터가 너무 적습니다. 최소 15개 이상 수집하세요.")
        return

    np.savez(
        "calibration/handeye_data.npz",
        R_gripper2base=np.array(R_gripper2base_list),
        t_gripper2base=np.array(t_gripper2base_list),
        R_target2cam=np.array(R_target2cam_list),
        t_target2cam=np.array(t_target2cam_list),
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
    )
    print(f"\n수집 완료: {len(R_gripper2base_list)}개 자세 → calibration/handeye_data.npz")


if __name__ == "__main__":
    main()
