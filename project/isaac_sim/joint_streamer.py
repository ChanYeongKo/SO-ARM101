"""
SO-ARM101 관절 상태를 Isaac Sim(Windows PC)으로 UDP 스트리밍합니다.
Ubuntu 24.04 + LeRobot 환경에서 실행합니다.

사용법:
    conda activate lerobot
    cd ~/lerobot2/project
    python isaac_sim/joint_streamer.py --target-ip <Windows_IP>

옵션:
    --target-ip   Isaac Sim이 실행 중인 Windows PC의 IP 주소 (필수)
    --port        UDP 포트 (기본값: 5005)
    --robot-port  팔로워암 시리얼 포트 (기본값: /dev/ttyACM0)
    --hz          전송 주파수 Hz (기본값: 30)
    --no-robot    실제 로봇 없이 sin파 테스트 신호를 전송

예시:
    # 실제 로봇 연결 후 스트리밍
    python isaac_sim/joint_streamer.py --target-ip 192.168.1.100

    # 로봇 없이 Isaac Sim 연결 테스트
    python isaac_sim/joint_streamer.py --target-ip 192.168.1.100 --no-robot
"""

import argparse
import json
import math
import signal
import socket
import sys
import time

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
DEFAULT_PORT = 5005
DEFAULT_HZ = 30
DEFAULT_ROBOT_PORT = "/dev/ttyACM0"

_running = True


def _sigint_handler(sig, frame):
    global _running
    print("\n스트리밍 중지 중...")
    _running = False


def make_test_signal(t: float) -> dict:
    """실제 로봇 없이 sin파 테스트 신호 생성."""
    joints = {
        "shoulder_pan":  30.0 * math.sin(0.5 * t),
        "shoulder_lift": 20.0 * math.sin(0.3 * t + 1.0),
        "elbow_flex":    40.0 * math.sin(0.4 * t + 0.5),
        "wrist_flex":    25.0 * math.sin(0.6 * t + 2.0),
        "wrist_roll":    60.0 * math.sin(0.2 * t),
        "gripper":       50.0 + 50.0 * math.sin(0.1 * t),  # 0~100°
    }
    return joints


def stream_from_robot(robot, sock, target: tuple, hz: float):
    """레알 로봇에서 관절각을 읽어 스트리밍합니다."""
    interval = 1.0 / hz
    frame = 0
    while _running:
        t_start = time.monotonic()

        obs = robot.get_observation()
        joints = {name: float(obs[f"{name}.pos"]) for name in JOINT_NAMES}
        joints["gripper"] = float(obs.get("gripper.pos", 0.0))

        data = json.dumps(joints).encode()
        sock.sendto(data, target)

        frame += 1
        if frame % (int(hz) * 5) == 0:
            print(f"  [{frame}프레임] shoulder_pan={joints['shoulder_pan']:.1f}°  "
                  f"shoulder_lift={joints['shoulder_lift']:.1f}°  "
                  f"elbow_flex={joints['elbow_flex']:.1f}°")

        elapsed = time.monotonic() - t_start
        remaining = interval - elapsed
        if remaining > 0:
            time.sleep(remaining)


def stream_test_signal(sock, target: tuple, hz: float):
    """sin파 테스트 신호를 스트리밍합니다."""
    interval = 1.0 / hz
    t0 = time.monotonic()
    frame = 0
    while _running:
        t_start = time.monotonic()
        t = t_start - t0

        joints = make_test_signal(t)
        data = json.dumps(joints).encode()
        sock.sendto(data, target)

        frame += 1
        if frame % (int(hz) * 5) == 0:
            print(f"  [테스트 {t:.1f}s] shoulder_pan={joints['shoulder_pan']:.1f}°  "
                  f"elbow_flex={joints['elbow_flex']:.1f}°")

        elapsed = time.monotonic() - t_start
        remaining = interval - elapsed
        if remaining > 0:
            time.sleep(remaining)


def main():
    parser = argparse.ArgumentParser(description="SO-ARM101 → Isaac Sim UDP 스트리머")
    parser.add_argument("--target-ip", required=True,
                        help="Isaac Sim Windows PC의 IP 주소")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"UDP 포트 (기본값: {DEFAULT_PORT})")
    parser.add_argument("--robot-port", default=DEFAULT_ROBOT_PORT,
                        help=f"팔로워암 시리얼 포트 (기본값: {DEFAULT_ROBOT_PORT})")
    parser.add_argument("--hz", type=float, default=DEFAULT_HZ,
                        help=f"전송 주파수 Hz (기본값: {DEFAULT_HZ})")
    parser.add_argument("--no-robot", action="store_true",
                        help="실제 로봇 없이 테스트 sin파 신호 전송")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _sigint_handler)
    target = (args.target_ip, args.port)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    if args.no_robot:
        print(f"[테스트 모드] sin파 신호 → {args.target_ip}:{args.port} @ {args.hz}Hz")
        print("Isaac Sim에서 로봇이 부드럽게 움직이는지 확인하세요.")
        try:
            stream_test_signal(sock, target, args.hz)
        finally:
            sock.close()
        return

    # 실제 로봇 연결
    try:
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    except ImportError:
        print("[오류] LeRobot을 찾을 수 없습니다. conda activate lerobot 후 실행하세요.")
        sys.exit(1)

    print(f"팔로워암 연결 중: {args.robot_port}")
    robot = SO101Follower(SO101FollowerConfig(
        port=args.robot_port,
        max_relative_target=None,
    ))
    robot.connect(calibrate=False)
    print(f"연결 완료. {args.target_ip}:{args.port}으로 스트리밍 시작 @ {args.hz}Hz")
    print("Ctrl+C로 종료합니다.\n")

    try:
        stream_from_robot(robot, sock, target, args.hz)
    finally:
        sock.close()
        robot.disconnect()
        print("종료 완료.")


if __name__ == "__main__":
    main()
