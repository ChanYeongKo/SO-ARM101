"""
IK 래퍼: lerobot 내장 kinematics(placo) 기반

SO-ARM101은 5DOF이므로 임의의 6DOF 자세를 만들 수 없습니다.
위치를 우선하되, 위에서 내려오는 자세를 유도합니다.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from lerobot.model.kinematics import RobotKinematics

URDF_PATH = "so101_new_calib.urdf"
EEF_FRAME = "gripper_frame_link"
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# 위에서 수직으로 내려오는 자세 초기값 후보들
# shoulder_lift를 크게 숙이고 elbow/wrist로 그리퍼를 아래로 향하게
_TOP_DOWN_CANDIDATES = [
    np.array([-13, 44, 58, -83, 0]),     # 실측 파지 자세 (기준)
    np.array([0, 44, 58, -83, 0]),
    np.array([-20, 44, 58, -83, 0]),
    np.array([-13, 35, 50, -70, 0]),
    np.array([-13, 50, 65, -90, 0]),
    np.array([0, -45, 90, -45, 0]),
    np.array([0, -60, 90, -30, 0]),
    np.array([0, -30, 80, -50, 0]),
    np.array([10, 44, 58, -83, 0]),
    np.array([-30, 44, 58, -83, 0]),
    np.array([0, 40, 60, -80, 0]),
]


class IKSolver:
    def __init__(self):
        self.kin = RobotKinematics(
            urdf_path=URDF_PATH,
            target_frame_name=EEF_FRAME,
            joint_names=JOINT_NAMES,
        )
        # 위에서 내려오는 자세: 그리퍼 Z축이 아래(-Z)를 향하도록
        self._topdown_rot = R.from_euler("y", -90, degrees=True).as_matrix()

    def check_eef_axes(self):
        """홈 자세에서 FK 실행 -> 그리퍼 축 방향 확인."""
        T = self.kin.forward_kinematics(np.zeros(len(JOINT_NAMES)))
        print("=== 홈 자세 EEF 정보 ===")
        print(f"위치 (베이스 기준): {T[:3, 3]}")
        print(f"X축 방향: {T[:3, 0]}")
        print(f"Y축 방향: {T[:3, 1]}")
        print(f"Z축 방향: {T[:3, 2]}  <- 보통 그리퍼 접근 방향")
        return T

    def _try_ik(self, target_pos, init_joints_deg, orientation_hint=None, ori_weight=0.0):
        """한 번의 IK 시도. (q, error_mm) 반환."""
        T_target = np.eye(4)
        T_target[:3, 3] = target_pos

        if orientation_hint is not None:
            T_target[:3, :3] = orientation_hint
        else:
            T_current = self.kin.forward_kinematics(init_joints_deg)
            T_target[:3, :3] = T_current[:3, :3]

        try:
            q = self.kin.inverse_kinematics(
                init_joints_deg, T_target,
                position_weight=1.0,
                orientation_weight=ori_weight,
            )
            T_result = self.kin.forward_kinematics(q)
            err_mm = np.linalg.norm(T_result[:3, 3] - target_pos) * 1000

            # 그리퍼 Z축이 아래를 향하는 정도 체크 (내적: -1이면 완전 아래)
            gripper_z = T_result[:3, 2]
            downward_score = -gripper_z[2]  # 양수일수록 아래를 향함

            return q, err_mm, downward_score
        except Exception:
            return None, float("inf"), -1.0

    def solve(
        self,
        target_pos: np.ndarray,
        current_joints_deg: np.ndarray,
        err_threshold_mm: float = 25.0,
        prefer_topdown: bool = True,
    ) -> np.ndarray | None:
        """
        여러 초기값으로 IK를 시도. 위에서 내려오는 자세를 우선합니다.

        Args:
            target_pos: 목표 EEF 위치 [x, y, z] meters, 베이스 프레임
            current_joints_deg: 현재 관절각 [deg]
            err_threshold_mm: 허용 오차 [mm]
            prefer_topdown: True이면 위에서 내려오는 자세를 선호
        """
        results = []

        # 1) 위에서 내려오는 자세 후보들 (자세 힌트 + 약간의 자세 가중치)
        for init in _TOP_DOWN_CANDIDATES:
            q, err, down_score = self._try_ik(
                target_pos, init,
                orientation_hint=self._topdown_rot,
                ori_weight=0.01,
            )
            if q is not None and err <= err_threshold_mm:
                results.append((q, err, down_score))

        # 2) 현재 자세 기반 (자세 가중치 없이)
        q, err, down_score = self._try_ik(target_pos, current_joints_deg)
        if q is not None and err <= err_threshold_mm:
            results.append((q, err, down_score))

        # 3) 홈 자세 기반
        q, err, down_score = self._try_ik(target_pos, np.zeros(len(JOINT_NAMES)))
        if q is not None and err <= err_threshold_mm:
            results.append((q, err, down_score))

        if not results:
            # 최후의 시도: 자세 무시, 오차 기준만
            best_err = float("inf")
            best_q = None
            best_down = -1.0
            for init in _TOP_DOWN_CANDIDATES + [current_joints_deg, np.zeros(len(JOINT_NAMES))]:
                q, err, down_score = self._try_ik(target_pos, init)
                if err < best_err:
                    best_err = err
                    best_q = q
                    best_down = down_score
            if best_q is not None and best_err <= err_threshold_mm:
                print(f"IK 성공 (fallback): 위치 오차 {best_err:.1f}mm, 하향도 {best_down:.2f}, 관절각: {np.round(best_q, 1)}")
                return best_q
            print(f"IK 실패: 최소 오차 {best_err:.1f}mm (허용: {err_threshold_mm}mm)")
            return None

        if prefer_topdown:
            # 위에서 내려오는 자세 선호: downward_score 높고 오차 적은 것
            # 오차 범위 내에서 가장 아래를 향하는 자세 선택
            results.sort(key=lambda x: (-x[2], x[1]))  # down_score 높은 것 우선, 같으면 오차 적은 것
        else:
            results.sort(key=lambda x: x[1])  # 오차 적은 것 우선

        best_q, best_err, best_down = results[0]
        print(f"IK 성공: 위치 오차 {best_err:.1f}mm, 하향도 {best_down:.2f}, 관절각: {np.round(best_q, 1)}")
        return best_q

    def compute_approach_pos(
        self, target_pos: np.ndarray, approach_offset: float = 0.08
    ) -> np.ndarray:
        """목표 위치 위쪽 approach_offset[m]에서 접근."""
        approach_pos = target_pos.copy()
        approach_pos[2] += approach_offset
        return approach_pos
