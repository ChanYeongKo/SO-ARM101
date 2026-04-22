"""
Isaac Sim 2023.1.1 - SO-ARM101 실시간 미러 시각화기

Ubuntu의 pick_and_dump.py (또는 joint_streamer.py)에서 UDP로 관절각을 받아
Isaac Sim 안의 가상 SO-ARM101을 실시간으로 구동합니다.

사용법 (Windows):
    cd D:\\isaac-sim
    python.bat D:\\isaac-sim\\so-arm101\\SO-ARM101\\project\\isaac_sim\\so_arm_visualizer.py

포트 방화벽 허용 (관리자 권한 CMD):
    netsh advfirewall firewall add rule name="IsaacSimUDP" protocol=UDP dir=in localport=5005 action=allow
"""

from omni.isaac.kit import SimulationApp

kit = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
    "title": "SO-ARM101 실시간 미러",
})

import json
import math
import os
import socket
import threading
import time

import carb
import omni.kit.commands
import omni.timeline
import omni.usd
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics

# ─── 경로 ────────────────────────────────────────────────────────────────────
_THIS    = os.path.dirname(os.path.abspath(__file__))
_PROJECT = os.path.dirname(_THIS)
_REPO    = os.path.dirname(_PROJECT)

SIM_URDF = os.path.join(_REPO, "SO-ARM100", "Simulation", "SO101", "so101_new_calib.urdf")

# ─── 설정 ────────────────────────────────────────────────────────────────────
UDP_PORT           = 5005
STREAM_TIMEOUT_SEC = 5.0
DRIVE_STIFFNESS    = 1e8
DRIVE_DAMPING      = 1e6

JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
]

JOINT_LIMITS_DEG = {
    "shoulder_pan":  (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex":    ( -97.0,  97.0),
    "wrist_flex":    ( -95.0,  95.0),
    "wrist_roll":    (-157.0, 163.0),
    "gripper":       ( -10.0, 100.0),
}


# ─── UDP 수신기 ───────────────────────────────────────────────────────────────

class UDPReceiver:
    def __init__(self, port: int):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.settimeout(0.2)
        self._lock   = threading.Lock()
        self._latest: dict = {}
        self._last_t = 0.0
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        carb.log_info(f"UDP 수신 대기 중 (포트 {port})")

    def _loop(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(512)
                joints = json.loads(data.decode())
                with self._lock:
                    self._latest = joints
                    self._last_t = time.monotonic()
            except socket.timeout:
                pass
            except (json.JSONDecodeError, OSError):
                pass

    def get(self) -> dict:
        with self._lock:
            return dict(self._latest)

    def age(self) -> float:
        with self._lock:
            return time.monotonic() - self._last_t if self._last_t else float("inf")

    def stop(self):
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass


# ─── DriveAPI 탐색 및 설정 ────────────────────────────────────────────────────

def find_and_setup_drives(stage, articulation_root: str) -> dict:
    robot_ns = "/" + articulation_root.strip("/").split("/")[0]
    drives = {}
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(robot_ns + "/"):
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            name  = prim.GetName()
            stiff = drive.GetStiffnessAttr()
            damp  = drive.GetDampingAttr()
            (stiff if stiff else drive.CreateStiffnessAttr()).Set(DRIVE_STIFFNESS)
            (damp  if damp  else drive.CreateDampingAttr()).Set(DRIVE_DAMPING)
            drives[name] = drive
    return drives


def set_drive_target(drive, deg: float):
    attr = drive.GetTargetPositionAttr()
    (attr if attr else drive.CreateTargetPositionAttr()).Set(deg)


# ─── 메인 ─────────────────────────────────────────────────────────────────────

def main():
    if not os.path.exists(SIM_URDF):
        carb.log_error(f"URDF 없음: {SIM_URDF}")
        kit.close()
        return
    carb.log_info(f"URDF 임포트: {SIM_URDF}")

    # URDF 임포트 (드라이브 타입 설정 없음 - 공식 예제 패턴)
    _, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    cfg.merge_fixed_joints    = False
    cfg.convex_decomp         = False
    cfg.import_inertia_tensor = True
    cfg.fix_base              = True
    cfg.self_collision        = False
    cfg.create_physics_scene  = False
    cfg.make_default_prim     = True
    cfg.distance_scale        = 100   # URDF[m] → stage[cm]

    ok, stage_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=SIM_URDF,
        import_config=cfg,
        get_articulation_root=True,
    )
    kit.update()   # DriveAPI 프림 등록 대기

    stage = omni.usd.get_context().get_stage()

    if not ok or not stage_path:
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                stage_path = str(prim.GetPath())
                break
    if not stage_path:
        carb.log_error("URDF 임포트 실패")
        kit.close()
        return
    carb.log_info(f"URDF prim: {stage_path}")

    # Physics Scene
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(981.0)   # cm/s²
    physx = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
    physx.CreateEnableCCDAttr(True)
    physx.CreateEnableStabilizationAttr(True)

    # Ground plane
    PhysicsSchemaTools.addGroundPlane(
        stage, "/groundPlane", "Z", 1500.0,
        Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.4, 0.4, 0.4),
    )

    # 조명
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/DomeLight"))
    dome.CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/KeyLight"))
    key.CreateIntensityAttr(3000.0)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-50, 45, 0))

    # DriveAPI 탐색
    drives  = find_and_setup_drives(stage, stage_path)
    missing = [n for n in JOINT_NAMES if n not in drives]
    if missing:
        carb.log_warn(f"누락 관절: {missing}")
    else:
        carb.log_info("6개 관절 DriveAPI 모두 확인됨 ✓")

    # 타임라인 시작
    omni.timeline.get_timeline_interface().play()
    kit.update()

    # 카메라
    cam = stage.GetPrimAtPath("/OmniverseKit_Persp")
    if cam.IsValid():
        xf  = UsdGeom.Xformable(cam)
        ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
        (ops.get("xformOp:translate") or xf.AddTranslateOp()).Set(Gf.Vec3d(60, -50, 40))
        (ops.get("xformOp:rotateXYZ") or xf.AddRotateXYZOp()).Set(Gf.Vec3f(68, 0, 50))

    # UDP 수신기 시작
    receiver = UDPReceiver(UDP_PORT)
    warned   = False

    carb.log_info(
        "준비 완료. Ubuntu에서 pick_and_dump.py를 실행하면 미러링이 시작됩니다.\n"
        f"  (수동 테스트: python isaac_sim/joint_streamer.py --target-ip <Windows IP>)"
    )

    try:
        while kit.is_running():
            joints_deg = receiver.get()

            # 데이터 타임아웃 경고
            if receiver.age() > STREAM_TIMEOUT_SEC:
                if not warned:
                    carb.log_warn(
                        f"{STREAM_TIMEOUT_SEC}초 동안 UDP 데이터 없음 - "
                        "Ubuntu에서 pick_and_dump.py가 실행 중인지 확인하세요."
                    )
                    warned = True
            else:
                warned = False

            # 관절각 적용
            if joints_deg:
                for name in JOINT_NAMES:
                    if name not in drives or name not in joints_deg:
                        continue
                    deg = joints_deg[name]
                    if name in JOINT_LIMITS_DEG:
                        lo, hi = JOINT_LIMITS_DEG[name]
                        deg = max(lo, min(hi, deg))
                    set_drive_target(drives[name], deg)

            kit.update()

    except KeyboardInterrupt:
        carb.log_info("사용자 종료")
    finally:
        receiver.stop()
        omni.timeline.get_timeline_interface().stop()
        kit.close()


if __name__ == "__main__":
    main()
