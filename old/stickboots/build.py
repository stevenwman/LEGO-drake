from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import NamedTuple
import json
import shutil

from .config import StickbotParams
from .io import params_hash, prepare_output_dirs
from .generator import build_urdf
from stickbot.geom_gen import generate_feet_geom


class BuildResult(NamedTuple):
    urdf_path: Path
    robot_dir: Path
    mesh_dir: Path
    key: str


def build_stickbot(params: StickbotParams, base_out: Path | str = "stickbots/out") -> BuildResult:
    """Generate meshes and URDF for a given StickbotParams. Returns paths.

    Errors raise exceptions (callers can catch); we do not fallback silently.
    """
    base_out = Path(base_out)
    key = params_hash(params)
    robot_dir, mesh_dir = prepare_output_dirs(base_out, key)

    # Cache: if URDF exists and both meshes exist, return quickly
    urdf_path = robot_dir / "stickbot.urdf"
    left_obj = mesh_dir / "left_foot_geom.obj"
    right_obj = mesh_dir / "right_foot_geom.obj"
    if urdf_path.exists() and left_obj.exists() and right_obj.exists():
        return BuildResult(urdf_path=urdf_path, robot_dir=robot_dir, mesh_dir=mesh_dir, key=key)

    # Use the same mesh generation strategy as existing stickbot/geom_gen.py
    feet = params.feet_vars
    file_id = f"X_{feet.X}_Y_{feet.Y}_Z_{feet.Z}_box_x_{feet.box_x}_box_y_{feet.box_y}_scad_fn_{feet.scad_fn}"
    ft_prm = {
        "X": feet.X,
        "Y": feet.Y,
        "Z": feet.Z,
        "box_x": feet.box_x,
        "box_y": feet.box_y,
        "scad_fn": feet.scad_fn,
        "file_id": file_id,
    }

    # Generate under stickbot/<file_id>/ using original generator
    stickbot_dir = Path(__file__).resolve().parents[1] / "stickbot"
    generate_feet_geom(ft_prm, str(stickbot_dir))

    # Copy meshes from stickbot/<file_id>/ into this build's mesh_dir (self-contained)
    src_mesh_dir = stickbot_dir / file_id
    for name in ["left_foot_geom.obj", "right_foot_geom.obj"]:
        src = src_mesh_dir / name
        dst = mesh_dir / name
        if not dst.exists():
            shutil.copy2(src, dst)

    # Build and write URDF, with URDF mesh paths referencing stickbot/<file_id>/ like original
    urdf_path = build_urdf(params, robot_dir, mesh_dir=None, urdf_prefix_dir=stickbot_dir, file_id=file_id)

    # Save params (for provenance)
    with open(robot_dir / "params.json", "w") as f:
        json.dump(params.__dict__, f, indent=2)

    return BuildResult(urdf_path=urdf_path, robot_dir=robot_dir, mesh_dir=src_mesh_dir, key=key)
