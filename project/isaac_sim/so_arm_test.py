"""
Isaac Sim 2023.1.1 - SO-ARM101 독립 동작 테스트 (Ubuntu/로봇 불필요)

SO-ARM100/Simulation/SO101/so101_new_calib.urdf 사용
  - 모든 링크에 개별 STL 메시 포함 (onshape-to-robot 생성)
  - assets/ 폴더의 부품별 STL이 각 관절에 붙어서 애니메이션됨
  - DriveAPI angular position drive로 관절 제어

사용법:
    cd D:\\isaac-sim
    python.bat D:\\isaac-sim\\so-arm101\\SO-ARM101\\project\\isaac_sim\\so_arm_test.py
"""

from omni.isaac.kit import SimulationApp

kit = SimulationApp({
    "headless": False,
    "width": 1440,
    "height": 900,
    "title": "SO-ARM101 테스트",
})

import math
import os
import time

import omni.kit.commands
import omni.timeline
import omni.usd
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics

# ─── 경로 ─────────────────────────────────────────────────────────────────────
_THIS    = os.path.dirname(os.path.abspath(__file__))
_PROJECT = os.path.dirname(_THIS)
_REPO    = os.path.dirname(_PROJECT)

# Simulation 폴더의 URDF: 링크별 STL 메시 + 정확한 origin/rpy 포함
SIM_URDF = os.path.join(_REPO, "SO-ARM100", "Simulation", "SO101", "so101_new_calib.urdf")

# ─── 관절 설정 ────────────────────────────────────────────────────────────────
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

SAFE_POSE_DEG = {
    "shoulder_pan":   -13.67,
    "shoulder_lift": -102.07,
    "elbow_flex":      35.21,
    "wrist_flex":     104.53,
    "wrist_roll":      -0.75,
    "gripper":        100.00,
}

SIN_PARAMS = {
    # (진폭 deg, 주파수 Hz, 위상 rad, 중심 deg)
    "shoulder_pan":  (25.0, 0.20, 0.0,  0.0),
    "shoulder_lift": (20.0, 0.15, 1.0,  0.0),
    "elbow_flex":    (35.0, 0.25, 2.0,  0.0),
    "wrist_flex":    (30.0, 0.30, 0.5,  0.0),
    "wrist_roll":    (60.0, 0.12, 1.5,  0.0),
    "gripper":       (50.0, 0.10, 0.0, 50.0),
}

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e6


# ─── DriveAPI 탐색 및 설정 ────────────────────────────────────────────────────

def find_and_setup_drives(stage, articulation_root: str) -> dict:
    """
    articulation_root 예: /so101_new_calib/base_link
    조인트 DriveAPI 프림은 /so101_new_calib/ 하위 전체에 분산되어 있음.
    """
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


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ─── 애니메이션 ───────────────────────────────────────────────────────────────

class Animator:
    PHASES    = ["hold_home", "to_safe", "hold_safe", "to_neutral", "sin"]
    DURATIONS = {"hold_home": 2.0, "to_safe": 3.5, "hold_safe": 2.0, "to_neutral": 3.5}

    def __init__(self):
        self._idx  = 0
        self._t0   = time.monotonic()
        self._from = {k: 0.0 for k in JOINT_NAMES}
        print("[INFO] 애니메이션: HOME(2s) → SAFE(3.5s) → NEUTRAL(3.5s) → SIN파")

    @property
    def phase(self): return self.PHASES[self._idx]

    def _elapsed(self): return time.monotonic() - self._t0

    def _advance(self, from_j):
        self._idx  = min(self._idx + 1, len(self.PHASES) - 1)
        self._t0   = time.monotonic()
        self._from = dict(from_j)
        print(f"[INFO] Phase → {self.phase}")

    def _lerp(self, a, b, t):
        t = t * t * (3 - 2 * t)
        return {k: a[k] + (b[k] - a[k]) * t for k in JOINT_NAMES}

    def get(self) -> dict:
        home = {k: 0.0 for k in JOINT_NAMES}
        safe = SAFE_POSE_DEG
        t    = self._elapsed()

        if self.phase == "hold_home":
            if t >= self.DURATIONS["hold_home"]: self._advance(home)
            joints = home
        elif self.phase == "to_safe":
            dur = self.DURATIONS["to_safe"]
            joints = self._lerp(self._from, safe, min(t / dur, 1.0))
            if t >= dur: self._advance(safe)
        elif self.phase == "hold_safe":
            if t >= self.DURATIONS["hold_safe"]: self._advance(safe)
            joints = safe
        elif self.phase == "to_neutral":
            dur = self.DURATIONS["to_neutral"]
            joints = self._lerp(self._from, home, min(t / dur, 1.0))
            if t >= dur: self._advance(home)
        else:
            joints = {}
            for name in JOINT_NAMES:
                amp, freq, phase, center = SIN_PARAMS[name]
                joints[name] = center + amp * math.sin(2 * math.pi * freq * t + phase)

        for name in JOINT_NAMES:
            if name in JOINT_LIMITS_DEG:
                lo, hi = JOINT_LIMITS_DEG[name]
                joints[name] = clamp(joints[name], lo, hi)
        return joints


# ─── 메인 ─────────────────────────────────────────────────────────────────────

def main():
    if not os.path.exists(SIM_URDF):
        print(f"[오류] URDF 없음: {SIM_URDF}")
        kit.close()
        return
    print(f"[INFO] URDF 임포트: {SIM_URDF}")

    # URDF 임포트 (드라이브 타입 설정 없음 - 공식 예제 패턴)
    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints    = False
    import_config.convex_decomp         = False
    import_config.import_inertia_tensor = True
    import_config.fix_base              = True
    import_config.self_collision        = False
    import_config.create_physics_scene  = False
    import_config.make_default_prim     = True
    import_config.distance_scale        = 100   # URDF[m] → stage[cm]

    ok, stage_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=SIM_URDF,
        import_config=import_config,
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
        print("[오류] URDF 임포트 실패")
        kit.close()
        return
    print(f"[OK] URDF prim: {stage_path}")

    # Physics Scene
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(981.0)   # cm/s² (distance_scale=100)
    physx = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
    physx.CreateEnableCCDAttr(True)
    physx.CreateEnableStabilizationAttr(True)

    # Ground plane
    PhysicsSchemaTools.addGroundPlane(
        stage, "/groundPlane", "Z", 1500.0,
        Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5, 0.5, 0.5),
    )

    # 조명
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/DomeLight"))
    dome.CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/KeyLight"))
    key.CreateIntensityAttr(3000.0)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-50, 45, 0))

    # DriveAPI 탐색 및 stiffness 설정
    drives  = find_and_setup_drives(stage, stage_path)
    missing = [n for n in JOINT_NAMES if n not in drives]
    print(f"[검증] 드라이브 발견 {len(drives)}개: {list(drives.keys())}")
    if missing:
        print(f"[경고] 누락: {missing}")
    else:
        print("[검증 통과] 6개 관절 DriveAPI 모두 확인됨 ✓")

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

    # 애니메이션 루프
    anim      = Animator()
    frame     = 0
    LOG_EVERY = 90

    print("[INFO] 시뮬레이션 시작. 창 닫기 또는 Ctrl+C로 종료.")

    try:
        while kit.is_running():
            joints_deg = anim.get()
            for name, val in joints_deg.items():
                if name in drives:
                    set_drive_target(drives[name], val)
            kit.update()
            frame += 1

            if frame % LOG_EVERY == 0:
                vals = {n: drives[n].GetTargetPositionAttr().Get()
                        for n in JOINT_NAMES if n in drives}
                print(
                    f"[F{frame:05d}|{anim.phase}] "
                    f"pan={vals.get('shoulder_pan', 0):.1f}° "
                    f"lift={vals.get('shoulder_lift', 0):.1f}° "
                    f"elbow={vals.get('elbow_flex', 0):.1f}° "
                    f"wrist_f={vals.get('wrist_flex', 0):.1f}° "
                    f"wrist_r={vals.get('wrist_roll', 0):.1f}° "
                    f"gripper={vals.get('gripper', 0):.1f}°"
                )

    except KeyboardInterrupt:
        print("[INFO] 사용자 종료")
    finally:
        omni.timeline.get_timeline_interface().stop()
        kit.close()


if __name__ == "__main__":
    main()
