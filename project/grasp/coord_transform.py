"""
좌표 변환: 카메라 프레임 -> 로봇 베이스 프레임 (eye-in-hand)

카메라가 그리퍼에 달려 있으므로 변환이 2단계:
  1. p_cam x T_cam->gripper  -> p_gripper  (Hand-Eye 캘리브 결과, 고정값)
  2. p_gripper x FK(q)       -> p_base     (현재 관절각으로 실시간 계산)
"""

import numpy as np
from lerobot.model.kinematics import RobotKinematics

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# 캘리브레이션 결과 로드
_cal = None
_T_CAM_TO_GRIPPER = None


def _load_calibration():
    global _cal, _T_CAM_TO_GRIPPER
    if _T_CAM_TO_GRIPPER is None:
        _cal = np.load("calibration/hand_eye_result.npz")
        _T_CAM_TO_GRIPPER = _cal["T_cam_to_gripper"]
    return _T_CAM_TO_GRIPPER


def cam_to_base(
    point_cam: np.ndarray,
    current_joints_deg: np.ndarray,
    kinematics: RobotKinematics,
) -> np.ndarray:
    """
    카메라 프레임 3D 좌표를 로봇 베이스 프레임으로 변환합니다.

    Args:
        point_cam: 카메라 프레임 3D 좌표 [Xc, Yc, Zc] meters
        current_joints_deg: 팔로워암 현재 관절각 [deg]
        kinematics: RobotKinematics 인스턴스
    Returns:
        p_base: 로봇 베이스 프레임 3D 좌표 [Xb, Yb, Zb] meters
    """
    T_cam_to_gripper = _load_calibration()

    # 단계 1: 카메라 프레임 -> 그리퍼 프레임
    p_cam_h = np.append(point_cam, 1.0)
    p_gripper_h = T_cam_to_gripper @ p_cam_h
    p_gripper = p_gripper_h[:3]

    # 단계 2: 그리퍼 프레임 -> 베이스 프레임
    T_base_gripper = kinematics.forward_kinematics(current_joints_deg)
    p_gripper_h2 = np.append(p_gripper, 1.0)
    p_base_h = T_base_gripper @ p_gripper_h2

    return p_base_h[:3]
