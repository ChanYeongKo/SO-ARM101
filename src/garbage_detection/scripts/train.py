"""
YOLOv8 쓰레기 감지 모델 학습 스크립트
"""

import argparse
from pathlib import Path
from ultralytics import YOLO


DATASET_DIR  = Path(__file__).parent.parent / "data"
MODEL_DIR    = Path(__file__).parent.parent / "models"
DATA_YAML    = DATASET_DIR / "data.yaml"

MODEL_DIR.mkdir(exist_ok=True)


def train(
    base_model: str = "yolov8n.pt",
    epochs: int = 100,
    imgsz: int = 640,
    batch: int = 16,
    device: str = "0",
):
    """
    YOLOv8 모델 학습

    Args:
        base_model: 베이스 모델 (n=nano, s=small, m=medium)
                    GPU 메모리 부족 시 yolov8n.pt 사용
        epochs:     학습 에폭 수
        imgsz:      입력 이미지 크기
        batch:      배치 사이즈 (GPU 메모리에 따라 조절)
        device:     '0' = GPU, 'cpu' = CPU
    """
    if not DATA_YAML.exists():
        print(f"[ERROR] 데이터셋이 없습니다: {DATA_YAML}")
        print("먼저 download_dataset.py를 실행하세요.")
        return

    print(f"[INFO] 베이스 모델: {base_model}")
    print(f"[INFO] 데이터셋: {DATA_YAML}")
    print(f"[INFO] 학습 설정: epochs={epochs}, imgsz={imgsz}, batch={batch}")
    print()

    model = YOLO(base_model)

    results = model.train(
        data=str(DATA_YAML),
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        device=device,
        project=str(MODEL_DIR),
        name="garbage_detect",
        exist_ok=True,
        # 학습 최적화 설정
        optimizer="AdamW",
        lr0=0.001,
        patience=20,          # 20 에폭 동안 성능 향상 없으면 조기 종료
        save=True,
        save_period=10,       # 10 에폭마다 체크포인트 저장
        val=True,
        plots=True,
    )

    best_model = MODEL_DIR / "garbage_detect" / "weights" / "best.pt"
    print(f"\n[INFO] 학습 완료!")
    print(f"[INFO] 최적 모델 저장 위치: {best_model}")
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLOv8 쓰레기 감지 모델 학습")
    parser.add_argument("--model",  type=str, default="yolov8n.pt",
                        help="베이스 모델 (yolov8n/s/m.pt)")
    parser.add_argument("--epochs", type=int, default=100,
                        help="학습 에폭 수")
    parser.add_argument("--imgsz",  type=int, default=640,
                        help="입력 이미지 크기")
    parser.add_argument("--batch",  type=int, default=16,
                        help="배치 사이즈")
    parser.add_argument("--device", type=str, default="0",
                        help="'0'=GPU, 'cpu'=CPU")
    args = parser.parse_args()

    train(
        base_model=args.model,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
    )
