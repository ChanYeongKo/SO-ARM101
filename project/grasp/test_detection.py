"""
YOLO 검출 테스트: D405 카메라로 쓰레기통이 검출되는지 실시간 확인

사용법:
  cd ~/lerobot2/project
  python grasp/test_detection.py

'q'로 종료
"""

import numpy as np
import pyrealsense2 as rs
import cv2
from ultralytics import YOLO

MODEL_PATH = "best.pt"

def main():
    model = YOLO(MODEL_PATH)

    # D405 연결
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    # 카메라 안정화
    for _ in range(30):
        pipeline.wait_for_frames()

    print("YOLO 검출 테스트 시작 (q로 종료)")
    print(f"모델: {MODEL_PATH}")
    print(f"클래스 목록: {model.names}")

    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data()).astype(float) * depth_scale

        results = model(color_image, verbose=False)

        # 결과 시각화
        vis = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        for box in results[0].boxes:
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = model.names[cls_id]
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            # bbox 중앙 depth
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            depth_val = depth_image[cy, cx] if 0 <= cy < 480 and 0 <= cx < 640 else 0

            # bbox 그리기
            color = (0, 255, 0) if conf > 0.5 else (0, 0, 255)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            label = f"{cls_name} {conf:.2f} d={depth_val:.3f}m"
            cv2.putText(vis, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        count = len(results[0].boxes)
        cv2.putText(vis, f"Detections: {count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.imshow("YOLO Detection Test", vis)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    pipeline.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
