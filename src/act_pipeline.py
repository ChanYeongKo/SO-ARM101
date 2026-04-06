"""
SO-ARM101 통합 파이프라인: IK 기반 이동 + ACT 데이터 수집/추론
════════════════════════════════════════════════════════════════

【단계 1】 좌표계 변환 + IK 이동
  YOLO + D405 Depth → cam_to_base() → IK → hover(물체 10cm 위)

  cam_to_base() 변환 수식:
    p_base = T_base_gripper @ T_gripper_cam @ p_cam

    T_gripper_cam (URDF gripper_to_d405 기반):
      위치: [0.08, 0.00, 0.05] m  (앞 8cm, 위 5cm)
      방향: Y축 pitch +0.698 rad (아래로 40도)

         R_y(θ) = [ cosθ   0   sinθ ]
                  [   0    1     0  ]   θ = 0.698 rad
                  [-sinθ   0   cosθ ]

    T_base_gripper: IKPy FK(현재 관절 각도)

【단계 2】 리더-팔 데이터 수집
  hover 도달 → ENTER → 리더 팔 시연 → LeRobotDataset(HDF5) 저장

【단계 3】 ACT 추론
  hover 도달 → ACT 정책 로드 → select_action() 루프 → 팔로워 실시간 제어

실행:
  python src/act_pipeline.py --dry-run
  python src/act_pipeline.py --collect --episodes 30
  python src/act_pipeline.py --collect --resume
  python src/act_pipeline.py --infer --checkpoint outputs/act_trash_bin/checkpoints/last/pretrained_model
"""

import argparse
import math
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from ikpy.chain import Chain
from ultralytics import YOLO

# ─────────────────────────────────────────────────────────────
# 설정값
# ─────────────────────────────────────────────────────────────
FOLLOWER_PORT   = "/dev/ttyACM1"
LEADER_PORT     = "/dev/ttyACM0"
URDF_PATH       = "/home/chan/SO-ARM101/src/so101.urdf"
MODEL_PATH      = "/home/chan/SO-ARM101/src/best.pt"
DATASET_REPO_ID = "local/trash_bin_pickup"

IMG_W, IMG_H     = 848, 480
FPS              = 30
CONF_THRESH      = 0.25
ROI_BOTTOM       = 0.60     # 화면 상단 60%만 감지 (그리퍼 영역 제외)
HOVER_Z_OFFSET   = 0.05     # hover 높이 (카메라 기준, 실제 그리퍼 간격 ~10cm)
HOVER_BACK_EXTRA = 0.10     # 그리퍼 가림 방지용 추가 후퇴 거리 (10cm)

# 카메라 장착 오프셋 (URDF gripper_to_d405 joint 기준)
CAM_X    = 0.08    # gripper_link 전방 +8cm
CAM_Y    = 0.00
CAM_Z    = 0.05    # gripper_link 상방 +5cm
CAM_TILT = 0.698   # Y축 pitch: 아래로 40도 (0.698 rad)

# Visual Servoing
CENTER_TOL = 30    # px
PAN_GAIN   = 0.02
TILT_GAIN  = 0.015
MAX_ITER   = 60

GRIPPER_OPEN     = 0.0
GRIPPER_CLOSE    = 80.0
EPISODE_TIME_SEC = 60

# IKPy 체인
# URDF 체인 끝: gripper_frame_link (fixed joint)
# active_links_mask: base(고정) + 5관절(활성) + gripper_frame(고정)
MOTOR_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ACTIVE_MASK = [False, True, True, True, True, True, False]

HOME = {
    "shoulder_pan.pos":   0.0,
    "shoulder_lift.pos": -96.6,
    "elbow_flex.pos":     30.0,
    "wrist_flex.pos":     83.2,
    "wrist_roll.pos":    -11.9,   # 원래 -101.9에서 좌측으로 90도 회전
    "gripper.pos":         0.0,
}

WIN_NAME = "SO-ARM101 Pipeline"


# ═════════════════════════════════════════════════════════════
# 【단계 1】 좌표계 변환 + IK
# ═════════════════════════════════════════════════════════════

def build_chain() -> Chain:
    """IKPy 체인 생성. 끝점 = gripper_frame_link."""
    return Chain.from_urdf_file(URDF_PATH, active_links_mask=ACTIVE_MASK)


def obs_to_ikpy(obs: dict, chain: Chain) -> list:
    """로봇 관찰값(도) → IKPy 관절 각도 리스트(라디안).

    shoulder_pan, wrist_roll은 로봇 부호 규칙이 IKPy와 반대이므로 반전.
    (ik_solve 결과 반전과 대칭 — FK가 실제 자세를 올바르게 계산하기 위함)
    """
    angles = []
    for i, m in enumerate(MOTOR_NAMES):
        rad    = math.radians(obs.get(f"{m}.pos", 0.0))
        lo, hi = chain.links[i + 1].bounds
        if lo is not None:
            rad = max(lo, min(hi, rad))
        angles.append(rad)
    angles[0] *= -1   # shoulder_pan: 로봇 음(-) = IKPy 양(+)
    angles[4] *= -1   # wrist_roll:   로봇 음(-) = IKPy 양(+)
    return [0.0] + angles + [0.0]   # IKPy: [base] + [joints] + [end]


def cam_to_base(p_cam: np.ndarray, chain: Chain, obs: dict) -> np.ndarray:
    """
    D405 광학 좌표 → 로봇 base_link 좌표.

    IKPy 체인 끝점이 d405_link 이므로
    FK 결과 자체가 T_base_to_d405 (카메라 오프셋 + pitch 이미 포함).

        p_base = T_base_d405 @ [px, py, pz, 1]^T

    Args:
        p_cam: rs2_deproject_pixel_to_point() 결과 (광학계: X=우, Y=하, Z=전방)
        chain: IKPy 체인 (끝점 = d405_link)
        obs:   로봇 관찰값 (degree 단위)

    Returns:
        p_base: base_link 기준 3D 좌표 [m]
    """
    T_base_d405 = chain.forward_kinematics(obs_to_ikpy(obs, chain))
    p_base      = (T_base_d405 @ np.append(p_cam, 1.0))[:3]
    return p_base


def ik_solve(chain: Chain, target: np.ndarray, initial: list = None) -> dict | None:
    """
    목표 위치(base 좌표 [m])에 대한 IK.

    Returns:
        {motor_name: degrees}  성공
        None                   도달 불가
    """
    if initial is None:
        initial = [0.0] * len(chain.links)
    for seed in [initial, [0.0] * len(chain.links)]:
        try:
            result = chain.inverse_kinematics(
                target_position=target, initial_position=seed, max_iter=300)
            if result is not None:
                angles_deg    = [math.degrees(a) for a in result[1:-1]]
                angles_deg[0] *= -1   # shoulder_pan 방향 보정 (실측)
                angles_deg[4] *= -1   # wrist_roll 방향 보정 (실측)
                return dict(zip(MOTOR_NAMES, angles_deg))
        except ValueError:
            continue
    return None


# ─────────────────────────────────────────────────────────────
# 카메라 + YOLO 헬퍼
# ─────────────────────────────────────────────────────────────

def init_camera() -> tuple:
    """D405 파이프라인 초기화. (pipeline, align, intrinsics) 반환."""
    for attempt in range(3):
        try:
            pipeline = rs.pipeline()
            cfg      = rs.config()
            cfg.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.bgr8, FPS)
            cfg.enable_stream(rs.stream.depth, IMG_W, IMG_H, rs.format.z16,  FPS)
            pipeline.start(cfg)
            align      = rs.align(rs.stream.color)
            intrinsics = (pipeline.get_active_profile()
                          .get_stream(rs.stream.depth)
                          .as_video_stream_profile()
                          .get_intrinsics())
            print(f"  D405 준비 완료 (fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f})")
            return pipeline, align, intrinsics
        except Exception as e:
            print(f"  카메라 초기화 실패 ({attempt+1}/3): {e}")
            time.sleep(2.0)
    raise RuntimeError("D405 연결 실패. USB 직결 여부 및 권한 확인.")


def get_frames(pipeline, align):
    for _ in range(3):
        try:
            frames  = pipeline.wait_for_frames(timeout_ms=10000)
            aligned = align.process(frames)
            c, d    = aligned.get_color_frame(), aligned.get_depth_frame()
            if c and d:
                return c, d
        except RuntimeError:
            pass
    return None, None


def get_depth_median(depth_frame, cx: int, cy: int, radius: int = 5) -> float:
    """중심 픽셀 주변의 median depth [m]. 유효값 없으면 0.0."""
    depths = [
        depth_frame.get_distance(
            max(0, min(IMG_W-1, cx+dx)),
            max(0, min(IMG_H-1, cy+dy)))
        for dx in range(-radius, radius+1)
        for dy in range(-radius, radius+1)
    ]
    valid = [d for d in depths if 0.01 < d < 2.0]
    return float(np.median(valid)) if valid else 0.0


def detect_best_box(model, color_image: np.ndarray, full_frame: bool = False) -> dict | None:
    """최고 conf 박스 반환. full_frame=True 시 ROI 없이 전체 화면 탐색."""
    if full_frame:
        region = color_image
        y_offset = 0
    else:
        roi_h    = int(IMG_H * ROI_BOTTOM)
        region   = color_image[:roi_h, :]
        y_offset = 0
    results = model.predict(region, conf=CONF_THRESH, verbose=False)
    boxes   = results[0].boxes
    if len(boxes) == 0:
        return None
    best     = boxes[int(boxes.conf.argmax())]
    x1,y1,x2,y2 = map(int, best.xyxy[0].tolist())
    y1 += y_offset; y2 += y_offset
    cx = (x1 + x2) // 2
    cy = y1 + (y2 - y1) * 2 // 3   # 바운딩박스 세로 2/3 지점
    return {"xyxy": (x1,y1,x2,y2), "cx": cx, "cy": cy,
            "conf": float(best.conf)}


# ─────────────────────────────────────────────────────────────
# 로봇 제어 헬퍼
# ─────────────────────────────────────────────────────────────

def send(robot, action: dict, delay: float = 0.05):
    robot.send_action(action)
    time.sleep(delay)


def go_to(robot, target: dict, duration: float = 1.5, step: float = 0.05,
          pipeline=None, align=None, status: str = "이동 중..."):
    """선형 보간으로 목표 포지션까지 이동. pipeline/align 전달 시 카메라 실시간 표시."""
    obs   = robot.get_observation()
    steps = max(1, int(duration / step))
    for i in range(1, steps + 1):
        action = {k: obs.get(k, v) + (v - obs.get(k, v)) * (i / steps)
                  for k, v in target.items()}
        robot.send_action(action)
        if pipeline is not None and align is not None:
            cf, _ = get_frames(pipeline, align)
            if cf is not None:
                _show(np.asanyarray(cf.get_data()), status=status)
        cv2.waitKey(1)
        time.sleep(step)


# ─────────────────────────────────────────────────────────────
# 감지 → Visual Servoing → hover 이동  (단계 1 통합)
# ─────────────────────────────────────────────────────────────

def detect_and_hover(
    robot, model, pipeline, align, intrinsics, chain
) -> tuple:
    """
    YOLO 감지 → Visual Servoing → Depth 측정 → cam_to_base() →
    IK 계산 → hover(물체 10cm 위) 이동.

    Returns:
        (q_hover, q_pick, p_base)  성공
        (None,    None,   None)    실패
    """
    # ── 1. 감지 대기 ──────────────────────────────────────────
    print("  [감지] YOLO 대기 중...")
    color_image = depth_frame = None
    for _ in range(150):
        cf, df = get_frames(pipeline, align)
        if cf is None:
            continue
        color_image = np.asanyarray(cf.get_data())
        depth_frame = df
        box = detect_best_box(model, color_image)
        _show(color_image, status="감지 중...", box=box)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return None, None, None
        if box:
            print(f"  감지 성공: conf={box['conf']:.2f} pixel=({box['cx']},{box['cy']})")
            break
    else:
        print("  [감지] 실패 — 쓰레기통을 카메라 앞에 놓으세요.")
        return None, None, None

    # ── 2. Visual Servoing (화면 중앙 정렬) ──────────────────
    print("  [서보] 화면 중앙 정렬 중...")
    last_cx, last_cy = box["cx"], box["cy"]
    lost_count = 0
    for _ in range(MAX_ITER):
        cf, df = get_frames(pipeline, align)
        if cf is None:
            continue
        color_image = np.asanyarray(cf.get_data())
        depth_frame = df
        b = detect_best_box(model, color_image)

        if b:
            lost_count = 0
            last_cx, last_cy = b["cx"], b["cy"]
        else:
            lost_count += 1
            if lost_count > 15:
                print("  [서보] 감지 복구 불가 — 재시도")
                return None, None, None

        err_x = last_cx - IMG_W // 2
        err_y = last_cy - IMG_H // 2
        _show(color_image, status=f"서보 err=({err_x:+d},{err_y:+d})", box=b)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return None, None, None

        if abs(err_x) < CENTER_TOL and abs(err_y) < CENTER_TOL:
            print(f"  [서보] 정렬 완료 (err_x={err_x}, err_y={err_y})")
            break

        obs    = robot.get_observation()
        action = {
            "shoulder_pan.pos":  obs["shoulder_pan.pos"]  + err_x * PAN_GAIN,
            "shoulder_lift.pos": obs["shoulder_lift.pos"] + err_y * TILT_GAIN,
            **{f"{m}.pos": obs.get(f"{m}.pos", 0.0)
               for m in ["elbow_flex", "wrist_flex", "wrist_roll"]},
            "gripper.pos": GRIPPER_OPEN,
        }
        send(robot, action, delay=0.05)

    # ── 3. Depth → base_link 좌표 변환 ───────────────────────
    print("  [좌표] Depth → base_link 변환...")
    p_base = None
    for _ in range(5):
        cf, df = get_frames(pipeline, align)
        if cf is None:
            continue
        b  = detect_best_box(model, np.asanyarray(cf.get_data()))
        cx = b["cx"] if b else last_cx
        cy = b["cy"] if b else last_cy

        depth = get_depth_median(df, cx, cy)
        if depth < 0.01:
            continue

        obs    = robot.get_observation()
        # RealSense 픽셀 + depth → 광학 좌표계 xyz [m]
        p_cam  = np.array(rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth))
        p_base = cam_to_base(p_cam, chain, obs)

        print(f"  base_link 좌표: "
              f"x={p_base[0]:.3f}  y={p_base[1]:.3f}  z={p_base[2]:.3f} m  "
              f"(depth={depth*100:.1f}cm)")
        break

    if p_base is None:
        print("  [좌표] 깊이 측정 실패")
        return None, None, None

    # ── 4. IK 계산 ────────────────────────────────────────────
    # hover 위치: 물체 위 HOVER_Z_OFFSET + 카메라 틸트 보정
    # 카메라가 40도 아래로 기울어져 있으므로, 물체를 카메라 중앙에 두려면
    # 카메라(d405_link)를 물체 방향으로 수평 후퇴시켜야 함.
    # 수평 후퇴 거리 = HOVER_Z_OFFSET / tan(CAM_TILT)
    p_hover = p_base.copy()
    p_hover[2] += HOVER_Z_OFFSET

    forward_2d = p_base[:2].copy()
    norm_2d = np.linalg.norm(forward_2d)
    if norm_2d > 0.01:
        forward_2d /= norm_2d
        horizontal_back = HOVER_Z_OFFSET / math.tan(CAM_TILT) + HOVER_BACK_EXTRA
        p_hover[:2] -= forward_2d * horizontal_back  # 로봇 방향으로 수평 후퇴

    obs     = robot.get_observation()
    initial = obs_to_ikpy(obs, chain)
    q_hover = ik_solve(chain, p_hover, initial)
    q_pick  = ik_solve(chain, p_base,  initial)

    if q_hover is None or q_pick is None:
        print(f"  [IK] 도달 불가  target={p_hover.round(3)}")
        return None, None, None

    # wrist_roll만 현재 관찰값으로 고정 (카메라 좌우 회전 유지)
    current_wrist_roll = obs.get("wrist_roll.pos", HOME["wrist_roll.pos"])
    q_hover["wrist_roll"] = current_wrist_roll
    q_pick["wrist_roll"]  = current_wrist_roll

    print(f"  [IK] hover target: {p_hover.round(3)}")
    for m in MOTOR_NAMES:
        print(f"       {m:20s}  hover={q_hover[m]:7.2f}°  pick={q_pick[m]:7.2f}°")

    # ── 5. Hover 이동 ─────────────────────────────────────────
    print("  [이동] Hover 위치로 이동 중...")
    go_to(robot,
          {**{f"{k}.pos": v for k, v in q_hover.items()}, "gripper.pos": GRIPPER_OPEN},
          duration=2.5,
          pipeline=pipeline, align=align,
          status=f"Hover 이동... {depth*100:.1f}cm")

    # ── 6. Hover 도달 후 재정렬 (shoulder_pan만 조정) ─────────
    print("  [재정렬] 좌우 정렬 중...")
    for _ in range(MAX_ITER):
        cf, df = get_frames(pipeline, align)
        if cf is None:
            continue
        color_image = np.asanyarray(cf.get_data())
        b = detect_best_box(model, color_image, full_frame=True)

        if b is None:
            _show(color_image, status="★ HOVER — 물체 소실...", box=None)
            cv2.waitKey(1)
            continue

        err_x = b["cx"] - IMG_W // 2
        _show(color_image, status=f"★ HOVER 재정렬  err_x={err_x:+d}", box=b)
        cv2.waitKey(1)

        if abs(err_x) < CENTER_TOL:
            print(f"  [재정렬] 완료 err_x={err_x}")
            break

        obs    = robot.get_observation()
        action = {
            "shoulder_pan.pos": obs["shoulder_pan.pos"] + err_x * PAN_GAIN,
            # 나머지 관절 고정 (hover 높이 유지)
            **{f"{m}.pos": obs.get(f"{m}.pos", 0.0)
               for m in ["shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]},
            "gripper.pos": GRIPPER_OPEN,
        }
        send(robot, action, delay=0.05)

    print("  ★ HOVER 도달")
    return q_hover, q_pick, p_base


def _show(color_image: np.ndarray, status: str = "", box: dict = None):
    """화면에 바운딩박스 + 상태 텍스트 오버레이 후 imshow."""
    vis = color_image.copy()
    # 화면 중앙 십자선
    cx_img, cy_img = IMG_W // 2, IMG_H // 2
    cv2.line(vis, (cx_img - 20, cy_img), (cx_img + 20, cy_img), (100, 100, 100), 1)
    cv2.line(vis, (cx_img, cy_img - 20), (cx_img, cy_img + 20), (100, 100, 100), 1)
    # 바운딩박스
    if box is not None:
        x1, y1, x2, y2 = box["xyxy"]
        cx, cy = box["cx"], box["cy"]
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(vis, (cx, cy), 5, (0, 255, 255), -1)
        cv2.line(vis, (cx - 15, cy), (cx + 15, cy), (0, 255, 255), 1)
        cv2.line(vis, (cx, cy - 15), (cx, cy + 15), (0, 255, 255), 1)
        label = f"trash_bin {box['conf']:.2f}"
        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(vis, (x1, y1 - lh - 8), (x1 + lw + 4, y1), (0, 200, 0), -1)
        cv2.putText(vis, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    cv2.putText(vis, status, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow(WIN_NAME, vis)


# ═════════════════════════════════════════════════════════════
# 【단계 2】 리더-팔 데이터 수집
# ═════════════════════════════════════════════════════════════

def run_collect(args, robot, leader, model, pipeline, align, intrinsics, chain):
    """
    리더-팔 시연 데이터 수집 루프.
    흐름:
      HOME → detect_and_hover() → ENTER → record_loop() →
      LeRobotDataset.save_episode() → 반복

    저장 포맷: LeRobotDataset (내부적으로 HDF5 + MP4 비디오)
    학습 명령: lerobot-train --policy.type=act --dataset.repo_id=<repo_id>
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.datasets.pipeline_features import (
        aggregate_pipeline_dataset_features, create_initial_features)
    from lerobot.datasets.utils import combine_feature_dicts
    from lerobot.processor.factory import make_default_processors
    from lerobot.scripts.lerobot_record import record_loop
    from lerobot.utils.control_utils import init_keyboard_listener
    from lerobot.utils.utils import log_say

    # ── 프로세서 + 데이터셋 초기화 ──────────────────────────
    print("[데이터 수집] 프로세서 + 데이터셋 초기화...")
    teleop_proc, robot_proc, obs_proc = make_default_processors()

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_proc,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=True,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=obs_proc,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=True,
        ),
    )

    if args.resume:
        dataset = LeRobotDataset(args.repo_id)
        print(f"  기존 데이터셋 로드: {args.repo_id}")
    else:
        dataset = LeRobotDataset.create(
            repo_id=args.repo_id,
            fps=FPS,
            robot_type=robot.name,
            features=dataset_features,
            use_videos=True,
            image_writer_threads=4,
        )
        print(f"  새 데이터셋 생성: {args.repo_id}")

    listener, events = init_keyboard_listener()
    episode_idx = 0

    try:
        while episode_idx < args.episodes and not events.get("stop_recording"):
            print(f"\n{'='*55}")
            print(f"  에피소드 {episode_idx + 1} / {args.episodes}")
            print(f"{'='*55}")

            # 홈 복귀 후 hover 이동
            go_to(robot, HOME, duration=2.0)
            q_hover, q_pick, p_base = detect_and_hover(
                robot, model, pipeline, align, intrinsics, chain)

            if q_hover is None:
                print("  위치 결정 실패 — 재시도")
                continue

            # 녹화 시작 확인
            print("\n  [리더 팔] pick → dump → return 동작을 시연하세요.")
            print("  ENTER = 녹화 시작  /  s = 건너뜀  /  q = 종료")
            key = input("  > ").strip().lower()
            if key == 'q':
                break
            if key == 's':
                continue

            log_say(f"에피소드 {episode_idx + 1} 녹화 시작")
            print("  녹화 중... (Right Arrow = 완료, Esc = 중단)")

            record_loop(
                robot=robot,
                events=events,
                fps=FPS,
                teleop=leader,
                dataset=dataset,
                control_time_s=EPISODE_TIME_SEC,
                single_task="Pick up the trash bin and dump it",
                display_data=False,
                teleop_action_processor=teleop_proc,
                robot_action_processor=robot_proc,
                robot_observation_processor=obs_proc,
            )

            if events.get("rerecord_episode"):
                print("  → 재녹화 (이전 버퍼 삭제)")
                events["rerecord_episode"] = False
                events["exit_early"]       = False
                dataset.clear_episode_buffer()
                # hover 위치로 복귀
                go_to(robot,
                      {**{f"{k}.pos": v for k, v in q_hover.items()},
                       "gripper.pos": GRIPPER_OPEN},
                      duration=2.0)
                continue

            dataset.save_episode()
            episode_idx += 1
            print(f"  → 에피소드 {episode_idx} 저장 완료")

            go_to(robot, HOME, duration=2.0)
            if episode_idx < args.episodes and not events.get("stop_recording"):
                input("  쓰레기통 원래 위치로 복귀 후 ENTER > ")

    finally:
        listener.stop()
        dataset.finalize()
        print(f"\n저장 완료: {args.repo_id}  ({episode_idx}개 에피소드)")
        print()
        print("=== ACT 학습 명령어 ===")
        print(f"lerobot-train \\")
        print(f"  --policy.type=act \\")
        print(f"  --dataset.repo_id={args.repo_id} \\")
        print(f"  --training.num_epochs=200")


# ═════════════════════════════════════════════════════════════
# 【단계 3】 ACT 모델 추론
# ═════════════════════════════════════════════════════════════

def load_act_policy(checkpoint_path: str, device: str):
    """
    학습된 ACT 정책 로드.

    checkpoint_path 예:
      outputs/act_trash_bin/checkpoints/last/pretrained_model
      (lerobot-train 출력 디렉터리)
    """
    # lerobot 버전에 따라 임포트 경로가 다를 수 있음
    try:
        from lerobot.policies.act.modeling_act import ACTPolicy
    except ImportError:
        from lerobot.common.policies.act.modeling_act import ACTPolicy

    print(f"  ACT 정책 로드: {checkpoint_path}")
    policy = ACTPolicy.from_pretrained(checkpoint_path)
    policy.eval()
    policy = policy.to(device)
    print(f"  디바이스: {device}")
    return policy


def frame_to_tensor(color_image: np.ndarray, device: str) -> torch.Tensor:
    """
    BGR uint8 (H,W,3) → float32 텐서 (1,3,H,W), 값 범위 [0,1].
    lerobot 기본 정규화 규칙과 동일.
    """
    rgb    = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    tensor = torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0)  # (1,3,H,W)
    return tensor.to(device)


def obs_to_state_tensor(obs: dict, device: str) -> torch.Tensor:
    """
    로봇 관찰값 → 상태 벡터 텐서 (1, n_joints).
    순서: [pan, lift, elbow, wrist_flex, wrist_roll, gripper]
    ※ 학습 시 사용한 observation.state 순서와 반드시 일치해야 함.
    """
    state = [obs.get(f"{m}.pos", 0.0) for m in MOTOR_NAMES]
    state.append(obs.get("gripper.pos", 0.0))
    return torch.tensor(state, dtype=torch.float32).unsqueeze(0).to(device)


def action_tensor_to_dict(action_tensor: torch.Tensor) -> dict:
    """
    ACT 출력 텐서 → 로봇 action dict.
    출력 순서: [pan, lift, elbow, wrist_flex, wrist_roll, gripper]
    ※ 학습 시 action feature 순서와 반드시 일치해야 함.
    """
    if action_tensor.dim() > 1:
        action_tensor = action_tensor.squeeze(0)
    values = action_tensor.cpu().numpy()

    action = {}
    for i, m in enumerate(MOTOR_NAMES):
        if i < len(values):
            action[f"{m}.pos"] = float(values[i])
    if len(values) > len(MOTOR_NAMES):
        action["gripper.pos"] = float(values[len(MOTOR_NAMES)])
    return action


def run_infer(args, robot, model, pipeline, align, intrinsics, chain):
    """
    ACT 추론 실행 루프.
    흐름:
      HOME → detect_and_hover() → ACT 추론 루프(select_action) → HOME → 반복
    """
    device = "cuda" if torch.cuda.is_available() else "cpu"
    policy = load_act_policy(args.checkpoint, device)

    go_to(robot, HOME, duration=2.0)
    print("\n[ACT 추론] 시작. 쓰레기통을 카메라 앞에 놓으세요.")

    while True:
        # ── hover 위치까지 IK 이동 ────────────────────────────
        q_hover, q_pick, p_base = detect_and_hover(
            robot, model, pipeline, align, intrinsics, chain)

        if q_hover is None:
            print("  위치 결정 실패 — 재시도")
            time.sleep(1.0)
            continue

        print("\n  ★ HOVER 완료 — ACT 추론 시작")

        # ACT 청크 버퍼 초기화 (지원하는 경우)
        if hasattr(policy, "reset"):
            policy.reset()

        # ── ACT 추론 루프 ─────────────────────────────────────
        # ACT는 일정 길이의 action chunk를 한 번에 예측함.
        # select_action()이 내부적으로 chunk를 관리하며
        # 호출마다 다음 액션을 순서대로 반환함.
        infer_duration = 15.0   # 추론 최대 시간 [초] (동작 완료 전 상한)
        start_t        = time.time()

        while time.time() - start_t < infer_duration:
            cf, df = get_frames(pipeline, align)
            if cf is None:
                continue

            color_image = np.asanyarray(cf.get_data())
            obs         = robot.get_observation()

            # ── 관찰값 딕셔너리 구성 ──────────────────────────
            # 키 이름은 LeRobotDataset 학습 시 features에서 정의된 것과 일치해야 함.
            obs_dict = {
                "observation.images.wrist_cam": frame_to_tensor(color_image, device),
                "observation.state":            obs_to_state_tensor(obs, device),
            }

            # ── 추론 ─────────────────────────────────────────
            with torch.no_grad():
                action_tensor = policy.select_action(obs_dict)

            robot_action = action_tensor_to_dict(action_tensor)
            send(robot, robot_action, delay=1.0 / FPS)

            # 화면 표시
            elapsed = time.time() - start_t
            vis = color_image.copy()
            cv2.putText(vis, "ACT INFERENCE", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(vis, f"t={elapsed:.1f}s / {infer_duration}s", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.imshow(WIN_NAME, vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return   # 완전 종료

        print("  ACT 추론 완료 (시간 초과 또는 정상 종료)")
        go_to(robot, HOME, duration=2.0)

        print("\n  다음 에피소드 준비 — ENTER  /  'q' 종료")
        key = input("  > ").strip().lower()
        if key == 'q':
            break


# ═════════════════════════════════════════════════════════════
# 메인
# ═════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="SO-ARM101: IK 이동 + ACT 데이터 수집/추론 통합 파이프라인")
    parser.add_argument("--dry-run",    action="store_true",
                        help="로봇 없이 카메라 + YOLO 감지만 테스트")
    parser.add_argument("--collect",    action="store_true",
                        help="리더-팔 데이터 수집 모드")
    parser.add_argument("--infer",      action="store_true",
                        help="학습된 ACT 정책 추론 모드")
    parser.add_argument("--resume",     action="store_true",
                        help="기존 데이터셋에 이어서 수집")
    parser.add_argument("--episodes",   type=int, default=30,
                        help="수집할 에피소드 수 (default: 30)")
    parser.add_argument("--repo_id",    type=str, default=DATASET_REPO_ID,
                        help=f"데이터셋 repo_id (default: {DATASET_REPO_ID})")
    parser.add_argument("--checkpoint", type=str, default="",
                        help="ACT 체크포인트 경로 (--infer 필수)")
    args = parser.parse_args()

    if args.infer and not args.checkpoint:
        parser.error("--infer 모드는 --checkpoint 경로가 필요합니다.\n"
                     "예: --checkpoint outputs/act_trash_bin/checkpoints/last/pretrained_model")

    mode = ("DRY-RUN" if args.dry_run else
            "COLLECT" if args.collect else
            "INFER"   if args.infer   else None)
    if mode is None:
        parser.print_help()
        return

    print(f"[모드] {mode}")

    # ── 공통 초기화 ──────────────────────────────────────────
    print("\n[1/3] YOLO + IKPy 로딩...")
    model = YOLO(MODEL_PATH)
    chain = build_chain()
    print(f"  YOLO 클래스  : {model.names}")
    print(f"  IKPy 링크 수 : {len(chain.links)}  (active: {sum(ACTIVE_MASK)})")

    print("\n[2/3] D405 카메라 초기화...")
    pipeline, align, intrinsics = init_camera()

    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)

    # ── DRY-RUN ──────────────────────────────────────────────
    if args.dry_run:
        print("\n[DRY-RUN] 'q' = 종료")
        try:
            while True:
                cf, df = get_frames(pipeline, align)
                if cf is None:
                    continue
                color_image = np.asanyarray(cf.get_data())
                box = detect_best_box(model, color_image)
                vis = color_image.copy()
                if box:
                    x1,y1,x2,y2 = box["xyxy"]
                    dist = get_depth_median(df, box["cx"], box["cy"]) * 100
                    cv2.rectangle(vis, (x1,y1), (x2,y2), (0,255,0), 2)
                    label = f"conf={box['conf']:.2f}  d={dist:.1f}cm"
                    cv2.putText(vis, label, (x1, y1-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.imshow(WIN_NAME, vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except KeyboardInterrupt:
            pass
        finally:
            pipeline.stop()
            cv2.destroyAllWindows()
        return

    # ── 로봇 연결 ─────────────────────────────────────────────
    from lerobot.robots.so_follower.so_follower import SOFollower
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig

    print("\n[3/3] Follower 연결...")
    robot_cfg = SOFollowerRobotConfig(port=FOLLOWER_PORT, id="follower", use_degrees=True)
    robot     = SOFollower(robot_cfg)
    robot.connect()

    leader = None
    if args.collect:
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
        leader_cfg = SO101LeaderConfig(port=LEADER_PORT, id="leader")
        leader     = SO101Leader(leader_cfg)
        leader.connect()

    go_to(robot, HOME, duration=2.0)

    try:
        if args.collect:
            run_collect(args, robot, leader, model, pipeline, align, intrinsics, chain)
        elif args.infer:
            run_infer(args, robot, model, pipeline, align, intrinsics, chain)
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()
        if leader:
            leader.disconnect()
        robot.disconnect()
        print("종료")


if __name__ == "__main__":
    main()
