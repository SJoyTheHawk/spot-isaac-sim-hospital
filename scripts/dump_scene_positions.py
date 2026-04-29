"""Dump world-space pose (position + orientation) of prims in a USD stage.

Boots a headless SimulationApp so that pxr/USD libs are available, opens the
stage, traverses it, and writes a table of prim paths + world poses.

Orientation is reported as:
  - quaternion (x, y, z, w)
  - intrinsic XYZ Euler angles in degrees (roll, pitch, yaw)

Usage (from the repo root):
    cd /path/to/spot-isaac-lab-hospital
    ./scripts/run_isaac.sh ./scripts/dump_scene_positions.py \
        [--lights-only] [--xformable-only] [--usd <path>] [--out <file>]

Default USD: <repo-root>/assets/isaac_hospital_scene_spot.usd
Default out: /tmp/scene_positions.txt
"""
import argparse
import math
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom, UsdLux  # noqa: E402

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_USD = os.path.join(_REPO_ROOT, "assets", "isaac_hospital_scene_spot.usd")

LIGHT_TYPES = (
    UsdLux.SphereLight, UsdLux.RectLight, UsdLux.DiskLight,
    UsdLux.DistantLight, UsdLux.CylinderLight, UsdLux.DomeLight,
)


def is_light(prim):
    return any(prim.IsA(lt) for lt in LIGHT_TYPES)


def quat_to_euler_xyz_deg(qx, qy, qz, qw):
    """Convert quaternion to intrinsic XYZ Euler (roll, pitch, yaw) in degrees."""
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def extract_pose(matrix):
    """Return (translation_xyz, quat_xyzw) from a Gf.Matrix4d."""
    t = matrix.ExtractTranslation()
    rot = matrix.ExtractRotationQuat()
    qw = rot.GetReal()
    im = rot.GetImaginary()
    return (t[0], t[1], t[2]), (im[0], im[1], im[2], qw)


def main():
    ap = argparse.ArgumentParser(
        description="Dump world-space pose of all prims in a USD stage."
    )
    ap.add_argument("--usd", default=DEFAULT_USD,
                    help="Path to the USD file (default: assets/isaac_hospital_scene_spot.usd)")
    ap.add_argument("--lights-only", action="store_true",
                    help="Only output light prims")
    ap.add_argument("--xformable-only", action="store_true",
                    help="Only output prims with an Xformable schema")
    ap.add_argument("--out", default="/tmp/scene_positions.txt",
                    help="Output file path (default: /tmp/scene_positions.txt)")
    args = ap.parse_args()

    ctx = omni.usd.get_context()
    if not ctx.open_stage(args.usd):
        print(f"Failed to open {args.usd}", file=sys.stderr)
        simulation_app.close()
        sys.exit(1)
    stage = ctx.get_stage()

    rows = []  # (path, type, pos|None|str, quat|None, rpy|None)
    for prim in stage.Traverse():
        if args.lights_only and not is_light(prim):
            continue
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            if args.xformable_only or args.lights_only:
                continue
            rows.append((str(prim.GetPath()), prim.GetTypeName(), None, None, None))
            continue
        try:
            m = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            pos, quat = extract_pose(m)
            rpy = quat_to_euler_xyz_deg(*quat)
            rows.append((str(prim.GetPath()), prim.GetTypeName(), pos, quat, rpy))
        except Exception as e:  # noqa: BLE001
            rows.append((str(prim.GetPath()), prim.GetTypeName(), f"ERR: {e}", None, None))

    header = (
        f"{'TYPE':<30} "
        f"{'X':>9} {'Y':>9} {'Z':>9}   "
        f"{'QX':>7} {'QY':>7} {'QZ':>7} {'QW':>7}   "
        f"{'R_deg':>7} {'P_deg':>7} {'Y_deg':>7}  PATH"
    )

    lines = [
        f"# Scene: {args.usd}",
        f"# Total prims listed: {len(rows)}",
        header,
        "-" * len(header),
    ]
    for path, typ, pos, quat, rpy in rows:
        if pos is None:
            line = f"{typ:<30} {'(no xform)':<60}  {path}"
        elif isinstance(pos, str):
            line = f"{typ:<30} {pos}  {path}"
        else:
            x, y, z = pos
            qx, qy, qz, qw = quat
            r, p, yw = rpy
            line = (
                f"{typ:<30} "
                f"{x:>9.3f} {y:>9.3f} {z:>9.3f}   "
                f"{qx:>7.3f} {qy:>7.3f} {qz:>7.3f} {qw:>7.3f}   "
                f"{r:>7.2f} {p:>7.2f} {yw:>7.2f}  {path}"
            )
        lines.append(line)

    out_text = "\n".join(lines) + "\n"
    with open(args.out, "w") as f:
        f.write(out_text)
    print(f"[dump_scene_positions] Wrote {len(rows)} rows to {args.out}")

    simulation_app.close()


if __name__ == "__main__":
    main()
