"""
Hand-Eye Calibration 변환행렬 계산

수집된 데이터로 T_cam→gripper (카메라→그리퍼 변환행렬)를 계산합니다.

사용법:
  python calibration/step2_run_handeye.py
"""

import numpy as np
import cv2

data = np.load("calibration/handeye_data.npz")

print(f"수집된 자세 수: {len(data['R_gripper2base'])}")

# eye-in-hand 방식: T_cam→gripper 계산
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    list(data["R_gripper2base"]),
    list(data["t_gripper2base"]),
    list(data["R_target2cam"]),
    list(data["t_target2cam"]),
    method=cv2.CALIB_HAND_EYE_TSAI,
)

T_cam_to_gripper = np.eye(4)
T_cam_to_gripper[:3, :3] = R_cam2gripper
T_cam_to_gripper[:3, 3] = t_cam2gripper.flatten()

print("\nT_cam_to_gripper (카메라 -> 그리퍼 변환행렬):")
print(np.round(T_cam_to_gripper, 4))
print(f"\n카메라 위치 (그리퍼 기준): {t_cam2gripper.flatten()} [m]")

# 검증: 회전행렬이 유효한지 확인
det = np.linalg.det(R_cam2gripper)
print(f"회전행렬 det = {det:.6f} (1.0에 가까워야 정상)")

np.savez(
    "calibration/hand_eye_result.npz",
    T_cam_to_gripper=T_cam_to_gripper,
    camera_matrix=data["camera_matrix"],
    dist_coeffs=data["dist_coeffs"],
)
print("\n결과 저장: calibration/hand_eye_result.npz")
