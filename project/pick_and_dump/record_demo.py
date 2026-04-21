"""
리더암 시연 동작 기록

리더암을 잡고 쓰레기통 비우기 동작을 시연하면
관절값을 실시간으로 기록하여 저장합니다.

사용법:
  cd ~/lerobot2/project
  python pick_and_dump/record_demo.py

조작:
  - 리더암을 잡고 원하는 동작 수행
  - ENTER: 현재 자세를 웨이포인트로 저장
  - q: 종료 및 저장
"""

import time
import numpy as np
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

LEADER_PORT = "/dev/ttyACM0"
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def main():
    leader = SO101Leader(SO101LeaderConfig(port=LEADER_PORT))
    leader.connect()

    waypoints = []

    print("=" * 50)
    print("리더암 시연 기록")
    print("=" * 50)
    print("리더암을 잡고 동작을 수행하세요.")
    print("  ENTER: 현재 자세를 웨이포인트로 저장")
    print("  q + ENTER: 종료 및 저장")
    print("=" * 50)

    try:
        while True:
            user_input = input(f"\n[웨이포인트 {len(waypoints) + 1}] ENTER로 저장, q로 종료: ").strip()

            if user_input == "q":
                break

            obs = leader.get_action()
            wp = {j: round(obs[f"{j}.pos"], 2) for j in JOINT_NAMES}
            wp["gripper"] = round(obs["gripper.pos"], 2)
            waypoints.append(wp)

            print(f"  저장됨: {wp}")

    finally:
        leader.disconnect()

    if not waypoints:
        print("저장된 웨이포인트가 없습니다.")
        return

    # 결과 출력
    print("\n" + "=" * 50)
    print(f"총 {len(waypoints)}개 웨이포인트 저장")
    print("=" * 50)

    for i, wp in enumerate(waypoints):
        print(f"\n[웨이포인트 {i + 1}]")
        for j in JOINT_NAMES:
            print(f"  {j}: {wp[j]}")
        print(f"  gripper: {wp['gripper']}")

    # 코드에 붙여넣기 쉬운 형태로 출력
    print("\n" + "=" * 50)
    print("코드용 출력 (복사해서 사용):")
    print("=" * 50)
    print("DEMO_WAYPOINTS = [")
    for i, wp in enumerate(waypoints):
        joints = ", ".join(f'"{j}": {wp[j]}' for j in JOINT_NAMES)
        print(f"    {{{joints}, \"gripper\": {wp['gripper']}}},  # {i + 1}")
    print("]")

    # npz로도 저장
    save_path = "pick_and_dump/demo_waypoints.npz"
    np.savez(
        save_path,
        waypoints=[{k: v for k, v in wp.items()} for wp in waypoints],
        joint_names=JOINT_NAMES,
    )
    print(f"\n파일 저장: {save_path}")


if __name__ == "__main__":
    main()
