"""
so101_new_calib.urdf에 시각화용 geometry를 추가하여 so101_visual.urdf를 생성합니다.
현재 URDF는 FK/IK 전용으로 <visual> 태그가 없으므로 Isaac Sim에서 보이지 않습니다.

사용법 (Ubuntu 또는 Windows에서 한 번만 실행):
    cd ~/lerobot2/project
    python isaac_sim/add_visuals_to_urdf.py

출력: project/so101_visual.urdf
"""

import xml.etree.ElementTree as ET
import numpy as np
import os


def rotation_z_to_vec(v: np.ndarray) -> np.ndarray:
    """Z축 방향 실린더를 벡터 v 방향으로 정렬하는 3×3 회전행렬."""
    v = v / np.linalg.norm(v)
    z = np.array([0.0, 0.0, 1.0])
    if np.allclose(v, z):
        return np.eye(3)
    if np.allclose(v, -z):
        # 180° around X
        return np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)
    axis = np.cross(z, v)
    axis /= np.linalg.norm(axis)
    angle = float(np.arccos(np.clip(np.dot(z, v), -1.0, 1.0)))
    K = np.array([
        [0,        -axis[2],  axis[1]],
        [axis[2],   0,       -axis[0]],
        [-axis[1],  axis[0],  0],
    ])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    return R


def rpy_from_matrix(R: np.ndarray):
    """3×3 회전행렬 → (roll, pitch, yaw) [rad]."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll  = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll  = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = 0.0
    return roll, pitch, yaw


def add_visual(link_el, geometry_el, xyz=(0, 0, 0), rpy=(0, 0, 0), rgba="0.5 0.5 0.5 1"):
    vis = ET.SubElement(link_el, "visual")
    origin = ET.SubElement(vis, "origin")
    origin.set("xyz", f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}")
    origin.set("rpy", f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}")
    geom = ET.SubElement(vis, "geometry")
    geom.append(geometry_el)
    mat = ET.SubElement(vis, "material")
    mat.set("name", f"mat_{id(vis)}")
    color = ET.SubElement(mat, "color")
    color.set("rgba", rgba)
    return vis


def sphere_el(radius: float):
    e = ET.Element("sphere")
    e.set("radius", str(radius))
    return e


def cylinder_el(radius: float, length: float):
    e = ET.Element("cylinder")
    e.set("radius", str(radius))
    e.set("length", str(length))
    return e


def box_el(size_xyz):
    e = ET.Element("box")
    e.set("size", f"{size_xyz[0]} {size_xyz[1]} {size_xyz[2]}")
    return e


def add_link_cylinder(link_el, joint_xyz, radius=0.018, rgba="0.3 0.5 0.9 1"):
    """링크 원점 → child joint 위치를 잇는 실린더를 추가합니다."""
    v = np.array(joint_xyz, float)
    length = float(np.linalg.norm(v))
    if length < 1e-4:
        return
    midpoint = v / 2.0
    R = rotation_z_to_vec(v)
    roll, pitch, yaw = rpy_from_matrix(R)
    add_visual(
        link_el,
        cylinder_el(radius, length),
        xyz=tuple(midpoint),
        rpy=(roll, pitch, yaw),
        rgba=rgba,
    )


def main(src_urdf: str = None, dst_urdf: str = None):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(script_dir)
    if src_urdf is None:
        src_urdf = os.path.join(project_dir, "so101_new_calib.urdf")
    if dst_urdf is None:
        dst_urdf = os.path.join(project_dir, "so101_visual.urdf")

    ET.register_namespace("", "")
    tree = ET.parse(src_urdf)
    root = tree.getroot()

    # link 이름 → <link> 요소 매핑
    links = {el.get("name"): el for el in root.findall("link")}

    # ── base_link: 직사각형 베이스 ───────────────────────────────
    add_visual(links["base_link"], box_el((0.10, 0.08, 0.06)),
               xyz=(0.014, 0.0, 0.030), rgba="0.25 0.25 0.25 1")

    # ── shoulder_link → upper_arm 방향 실린더 + 구체 ─────────────
    # shoulder_lift joint origin: xyz=(-0.0304, -0.0183, -0.0542)
    add_link_cylinder(links["shoulder_link"],
                      (-0.0304, -0.0183, -0.0542), radius=0.022,
                      rgba="0.25 0.25 0.25 1")
    add_visual(links["shoulder_link"], sphere_el(0.028),
               xyz=(0, 0, 0), rgba="0.15 0.15 0.15 1")

    # ── upper_arm_link → elbow 방향 실린더 ──────────────────────
    # elbow_flex joint origin: xyz=(-0.11257, -0.028, 0)
    add_link_cylinder(links["upper_arm_link"],
                      (-0.11257, -0.028, 0.0), radius=0.018,
                      rgba="0.20 0.40 0.80 1")
    add_visual(links["upper_arm_link"], sphere_el(0.022),
               xyz=(0, 0, 0), rgba="0.15 0.30 0.70 1")

    # ── lower_arm_link → wrist 방향 실린더 ──────────────────────
    # wrist_flex joint origin: xyz=(-0.1349, 0.0052, 0)
    add_link_cylinder(links["lower_arm_link"],
                      (-0.1349, 0.0052, 0.0), radius=0.016,
                      rgba="0.20 0.40 0.80 1")
    add_visual(links["lower_arm_link"], sphere_el(0.020),
               xyz=(0, 0, 0), rgba="0.15 0.30 0.70 1")

    # ── wrist_link → gripper 방향 실린더 ────────────────────────
    # wrist_roll joint origin: xyz=(0, -0.0611, 0.0181)
    add_link_cylinder(links["wrist_link"],
                      (0.0, -0.0611, 0.0181), radius=0.020,
                      rgba="0.20 0.65 0.35 1")
    add_visual(links["wrist_link"], sphere_el(0.024),
               xyz=(0, 0, 0), rgba="0.15 0.55 0.25 1")

    # ── gripper_link: 그리퍼 박스 ────────────────────────────────
    add_visual(links["gripper_link"], box_el((0.040, 0.055, 0.090)),
               xyz=(0.0, 0.0, -0.045), rgba="0.85 0.45 0.10 1")
    add_visual(links["gripper_link"], sphere_el(0.022),
               xyz=(0, 0, 0), rgba="0.80 0.35 0.05 1")

    # ── moving_jaw: 작은 박스 ─────────────────────────────────────
    add_visual(links["moving_jaw_so101_v1_link"], box_el((0.015, 0.040, 0.035)),
               xyz=(0.0, -0.015, 0.0), rgba="0.85 0.55 0.15 1")

    # gripper_frame_link는 더미 링크이므로 시각화 불필요

    # 들여쓰기 적용 후 저장
    ET.indent(tree, space="  ")
    tree.write(dst_urdf, xml_declaration=True, encoding="utf-8")
    print(f"Visual URDF 생성 완료: {dst_urdf}")
    print("Isaac Sim 시각화 스크립트에서 이 파일을 사용하세요.")


if __name__ == "__main__":
    main()
