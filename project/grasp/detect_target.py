"""
YOLO + D405 -> 쓰레기통 3D 좌표 검출

커스텀 학습된 best.pt 모델로 종이컵 모양 쓰레기통을 검출하고
D405 depth로 카메라 프레임 3D 좌표를 계산합니다.
"""

import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# 커스텀 학습된 쓰레기통 검출 모델
MODEL_PATH = "best.pt"
CONFIDENCE_THRESHOLD = 0.5

_model = None


def get_model():
    global _model
    if _model is None:
        _model = YOLO(MODEL_PATH)
    return _model


def get_robust_depth(x1, y1, x2, y2, depth_image):
    """
    bbox 중앙 80% 영역의 유효 depth 중앙값을 반환합니다.
    엣지 픽셀(깊이 불연속)과 0값(미검출)을 제외합니다.
    """
    mx = max(1, (x2 - x1) // 10)
    my = max(1, (y2 - y1) // 10)
    roi = depth_image[y1 + my : y2 - my, x1 + mx : x2 - mx]
    valid = roi[roi > 0.01]  # 1cm 이상만
    if len(valid) == 0:
        return None
    return float(np.median(valid))


def get_trash_can_3d_cam(pipeline, align, depth_scale, color_intr):
    """
    YOLO로 쓰레기통 검출 후 카메라 프레임 3D 좌표를 반환합니다.

    Returns:
        np.array([Xc, Yc, Zc]) meters (카메라 프레임) 또는 None
    """
    model = get_model()

    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()
    if not color_frame or not depth_frame:
        return None

    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data()).astype(float) * depth_scale

    results = model(color_image, verbose=False)

    best = None
    best_conf = 0.0
    for box in results[0].boxes:
        conf = float(box.conf[0])
        if conf < CONFIDENCE_THRESHOLD or conf <= best_conf:
            continue
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
        d = get_robust_depth(x1, y1, x2, y2, depth_image)
        if d is None:
            continue
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        point = rs.rs2_deproject_pixel_to_point(color_intr, [cx, cy], d)
        best = np.array(point)
        best_conf = conf

    return best
