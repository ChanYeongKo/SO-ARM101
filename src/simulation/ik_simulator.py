"""
SO-ARM101 IK 시뮬레이터
=======================
목표 위치 (x, y, z) 를 입력하면 역기구학으로 관절 각도를 계산하고
로봇팔이 이동하는 과정을 3D 애니메이션으로 보여줍니다.

실행:
    pip install ikpy matplotlib numpy
    python ik_simulator.py
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox, Button
from mpl_toolkits.mplot3d import Axes3D
import ikpy.chain
import ikpy.link
import matplotlib

# 한글 폰트 설정 (Windows)
matplotlib.rc("font", family="Malgun Gothic")
matplotlib.rcParams["axes.unicode_minus"] = False


# ──────────────────────────────────────────────────────────
# 1. SO-ARM101 로봇 모델 정의 (실제 치수 근사값, 단위: m)
# ──────────────────────────────────────────────────────────
L_BASE  = 0.100   # 베이스 높이
L_UPPER = 0.130   # 상완 (어깨 → 팔꿈치)
L_LOWER = 0.120   # 하완 (팔꿈치 → 손목)
L_WRIST = 0.080   # 손목 → 그리퍼
REACH   = L_BASE + L_UPPER + L_LOWER + L_WRIST   # 최대 도달 거리

arm = ikpy.chain.Chain(
    name="so_arm101",
    links=[
        ikpy.link.OriginLink(),
        # 관절 1: 베이스 회전 (Z축)
        ikpy.link.URDFLink(
            name="base_rotation",
            origin_translation=[0, 0, L_BASE],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
            bounds=(-np.pi, np.pi),
        ),
        # 관절 2: 어깨 (Y축)
        ikpy.link.URDFLink(
            name="shoulder",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
            bounds=(-np.pi / 2, np.pi / 2),
        ),
        # 관절 3: 팔꿈치 (Y축)
        ikpy.link.URDFLink(
            name="elbow",
            origin_translation=[L_UPPER, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
            bounds=(-np.pi * 0.9, np.pi * 0.9),
        ),
        # 관절 4: 손목 (Y축)
        ikpy.link.URDFLink(
            name="wrist",
            origin_translation=[L_LOWER, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
            bounds=(-np.pi / 2, np.pi / 2),
        ),
        # 엔드이펙터 (고정)
        ikpy.link.URDFLink(
            name="end_effector",
            origin_translation=[L_WRIST, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0],
        ),
    ],
)


# ──────────────────────────────────────────────────────────
# 2. 유틸리티 함수
# ──────────────────────────────────────────────────────────
def get_positions(joint_angles: np.ndarray) -> np.ndarray:
    """관절 각도 → 각 관절의 3D 위치 (N × 3)"""
    matrices = arm.forward_kinematics(joint_angles, full_kinematics=True)
    return np.array([m[:3, 3] for m in matrices])


def compute_ik(target_xyz: list, current_angles: np.ndarray) -> np.ndarray:
    """목표 위치 → 관절 각도 (IK 계산)"""
    return arm.inverse_kinematics(
        target_position=target_xyz,
        initial_position=current_angles,
        max_iter=1000,
    )


def interpolate(start: np.ndarray, end: np.ndarray, n: int = 40) -> list:
    """두 관절 상태 사이를 n 프레임으로 선형 보간"""
    return [start + (end - start) * t for t in np.linspace(0, 1, n)]


# ──────────────────────────────────────────────────────────
# 3. 화면 레이아웃
# ──────────────────────────────────────────────────────────
fig = plt.figure(figsize=(15, 8))
fig.suptitle("SO-ARM101  IK 시뮬레이터", fontsize=15, fontweight="bold")

gs = gridspec.GridSpec(
    3, 3,
    figure=fig,
    left=0.05, right=0.97,
    top=0.92, bottom=0.08,
    wspace=0.35, hspace=0.55,
)

ax3d  = fig.add_subplot(gs[:, 0:2], projection="3d")   # 왼쪽: 3D 뷰
ax_bar = fig.add_subplot(gs[0:2, 2])                    # 오른쪽 위: 관절 각도 바
ax_info = fig.add_subplot(gs[2, 2])                     # 오른쪽 아래: 정보
ax_info.axis("off")

# 입력 위젯
ax_x  = fig.add_axes([0.70, 0.30, 0.10, 0.04])
ax_y  = fig.add_axes([0.70, 0.24, 0.10, 0.04])
ax_z  = fig.add_axes([0.70, 0.18, 0.10, 0.04])
ax_btn = fig.add_axes([0.82, 0.23, 0.12, 0.06])

tb_x  = TextBox(ax_x,  "X (m): ", initial="0.20")
tb_y  = TextBox(ax_y,  "Y (m): ", initial="0.00")
tb_z  = TextBox(ax_z,  "Z (m): ", initial="0.15")
btn   = Button(ax_btn, "이동 실행", color="royalblue", hovercolor="steelblue")
btn.label.set_color("white")
btn.label.set_fontsize(11)


# ──────────────────────────────────────────────────────────
# 4. 그리기 함수
# ──────────────────────────────────────────────────────────
JOINT_NAMES  = ["θ1\n베이스", "θ2\n어깨", "θ3\n팔꿈치", "θ4\n손목"]
JOINT_COLORS = ["#4C72B0", "#DD8452", "#55A868", "#C44E52"]


def draw_3d(current_angles, target_angles, target_xyz, anim_angles=None):
    ax3d.clear()
    ax3d.set_xlim(-REACH, REACH)
    ax3d.set_ylim(-REACH, REACH)
    ax3d.set_zlim(0, REACH * 1.2)
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.set_title("로봇팔 3D 시각화")

    # 바닥면
    xx, yy = np.meshgrid([-REACH, REACH], [-REACH, REACH])
    ax3d.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.08, color="gray")

    # 도달 범위 원 (바닥)
    theta = np.linspace(0, 2 * np.pi, 100)
    ax3d.plot(
        REACH * np.cos(theta), REACH * np.sin(theta),
        np.zeros(100), "--", color="gray", alpha=0.3, linewidth=1,
    )

    # 시작 위치 (반투명 회색)
    p0 = get_positions(current_angles)
    ax3d.plot(p0[:, 0], p0[:, 1], p0[:, 2], "o-",
              color="lightgray", linewidth=2, markersize=6, alpha=0.5, label="시작")

    # 애니메이션 중간 또는 목표 위치 (파란색)
    angles_to_draw = anim_angles if anim_angles is not None else target_angles
    p1 = get_positions(angles_to_draw)
    ax3d.plot(p1[:, 0], p1[:, 1], p1[:, 2], "o-",
              color="royalblue", linewidth=3, markersize=9, label="로봇팔")

    # 관절 번호 표시
    joint_labels = ["Base", "J1", "J2", "J3", "J4", "EE"]
    for i, (pos, lbl) in enumerate(zip(p1, joint_labels)):
        ax3d.text(pos[0], pos[1], pos[2] + 0.01, lbl, fontsize=7, color="navy")

    # 목표 위치 (빨간 별)
    ax3d.scatter(*target_xyz, color="red", s=250, marker="*", zorder=10,
                 label=f"목표 ({target_xyz[0]:.2f}, {target_xyz[1]:.2f}, {target_xyz[2]:.2f})")

    # 엔드이펙터 현재 위치
    ax3d.scatter(*p1[-1], color="blue", s=80, zorder=9)

    ax3d.legend(loc="upper left", fontsize=8)


def draw_bars(current_angles, target_angles):
    ax_bar.clear()

    cur_deg = np.rad2deg(current_angles[1:5])
    tgt_deg = np.rad2deg(target_angles[1:5])
    x = np.arange(len(JOINT_NAMES))
    w = 0.35

    b1 = ax_bar.bar(x - w / 2, cur_deg, w, label="현재", color="lightgray",  alpha=0.9)
    b2 = ax_bar.bar(x + w / 2, tgt_deg, w, label="목표", color=JOINT_COLORS, alpha=0.9)

    for bar in list(b1) + list(b2):
        v = bar.get_height()
        ax_bar.text(
            bar.get_x() + bar.get_width() / 2,
            v + (1 if v >= 0 else -4),
            f"{v:.1f}°", ha="center", va="bottom", fontsize=8,
        )

    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels(JOINT_NAMES, fontsize=9)
    ax_bar.set_ylabel("각도 (°)")
    ax_bar.set_title("관절 각도")
    ax_bar.legend(fontsize=8)
    ax_bar.axhline(0, color="black", linewidth=0.5)
    ax_bar.grid(axis="y", alpha=0.3)


def draw_info(current_angles, target_angles, target_xyz, error_mm):
    ax_info.clear()
    ax_info.axis("off")

    cur_pos = get_positions(current_angles)[-1]
    tgt_pos = get_positions(target_angles)[-1]

    lines = [
        f"목표 위치  : ({target_xyz[0]:.3f}, {target_xyz[1]:.3f}, {target_xyz[2]:.3f}) m",
        f"실제 도달  : ({tgt_pos[0]:.3f}, {tgt_pos[1]:.3f}, {tgt_pos[2]:.3f}) m",
        f"IK 오차    : {error_mm:.2f} mm",
        "",
        "관절 각도 (목표)",
    ] + [
        f"  {n.replace(chr(10),' ')}: {np.rad2deg(target_angles[i+1]):.1f}°"
        for i, n in enumerate(JOINT_NAMES)
    ]

    ax_info.text(
        0.02, 0.95, "\n".join(lines),
        transform=ax_info.transAxes,
        fontsize=9, verticalalignment="top",
        fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8),
    )


# ──────────────────────────────────────────────────────────
# 5. 상태 및 이벤트
# ──────────────────────────────────────────────────────────
state = {
    "current_angles": np.zeros(len(arm.links)),
    "anim": None,
}


def on_move(event=None):
    try:
        x = float(tb_x.text)
        y = float(tb_y.text)
        z = float(tb_z.text)
    except ValueError:
        print("[오류] 숫자를 올바르게 입력하세요.")
        return

    target = [x, y, z]
    dist   = np.linalg.norm(target)

    if dist > REACH:
        print(f"[경고] 목표({dist:.3f}m)가 팔 도달 범위({REACH:.3f}m)를 벗어납니다.")
        return
    if z < 0:
        print("[경고] Z는 0 이상이어야 합니다.")
        return

    print(f"\n[IK 계산] 목표: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

    current = state["current_angles"]
    target_angles = compute_ik(target, current)

    actual  = get_positions(target_angles)[-1]
    error_mm = np.linalg.norm(np.array(target) - actual) * 1000

    print(f"  실제 도달: ({actual[0]:.3f}, {actual[1]:.3f}, {actual[2]:.3f})")
    print(f"  IK 오차 : {error_mm:.2f} mm")
    for i, name in enumerate(JOINT_NAMES):
        print(f"  {name.replace(chr(10),' ')}: {np.rad2deg(target_angles[i+1]):.2f}°")

    # 애니메이션 프레임 생성
    frames = interpolate(current, target_angles, n=50)

    # 기존 애니메이션 중지
    if state["anim"] and state["anim"].event_source is not None:
        state["anim"].event_source.stop()
    state["anim"] = None

    def update(i):
        draw_3d(current, target_angles, target, anim_angles=frames[i])
        if i == 0:
            draw_bars(current, target_angles)
            draw_info(current, target_angles, target, error_mm)
        fig.canvas.draw_idle()
        return []

    def on_done(event):
        state["current_angles"] = target_angles

    state["anim"] = FuncAnimation(
        fig, update,
        frames=len(frames),
        interval=40,
        repeat=False,
    )
    state["anim"]._stop = on_done  # 완료 후 상태 업데이트

    # 완료 후 current 업데이트
    def finish():
        state["current_angles"] = target_angles

    fig.canvas.mpl_connect(
        "draw_event",
        lambda e: finish() if state["anim"] and not state["anim"].event_source else None,
    )


btn.on_clicked(on_move)


# ──────────────────────────────────────────────────────────
# 6. 초기 화면 그리기
# ──────────────────────────────────────────────────────────
init_angles = np.zeros(len(arm.links))
draw_3d(init_angles, init_angles, [0.20, 0.0, 0.15])
draw_bars(init_angles, init_angles)
draw_info(init_angles, init_angles, [0.20, 0.0, 0.15], 0.0)

print("=" * 50)
print("SO-ARM101 IK 시뮬레이터 시작")
print("=" * 50)
print(f"팔 최대 도달 거리: {REACH:.3f} m")
print("X, Y, Z 값을 입력하고 [이동 실행] 버튼을 클릭하세요.")
print()
print("예시 좌표 (범위 내)")
print(f"  앞쪽:   X=0.25, Y=0.00, Z=0.10")
print(f"  왼쪽:   X=0.10, Y=0.20, Z=0.15")
print(f"  위쪽:   X=0.10, Y=0.00, Z=0.30")

plt.show()
