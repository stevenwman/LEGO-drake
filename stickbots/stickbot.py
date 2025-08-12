from dataclasses import asdict
from pathlib import Path
from typing import NamedTuple
from .config import StickbotParams
from .io import params_hash, prepare_output_dirs
# from urdf_builder import build_urdf
from stickbots.geom_builder import generate_feet_geom


class BuildResult(NamedTuple):
    urdf_path: Path     
    robot_dir: Path
    mesh_dir: Path
    key: str


def build_stickbot(params: StickbotParams, base_out_dir: Path | str = "robots") -> BuildResult:
    """
    Generate a URDF and meshes given robot parameters
    """
    base_out_path = Path(base_out_dir)
    key = params_hash(params.feet_vars)
    robot_dir, mesh_dir = prepare_output_dirs(Path(base_out_path), key)

    # Cache: if URDF exists and both meshes exist, return quickly
    urdf_path = robot_dir / "stickbot.urdf"
    left_obj = mesh_dir / "left_foot_geom.obj"
    right_obj = mesh_dir / "right_foot_geom.obj"
    if urdf_path.exists() and left_obj.exists() and right_obj.exists():
        return BuildResult(urdf_path=urdf_path, 
                           robot_dir=robot_dir, 
                           mesh_dir=mesh_dir, 
                           key=key)

    stickbot_dir = Path(__file__).resolve().parents[0]
    generate_feet_geom(params.feet_vars, str(stickbot_dir), mesh_dir)



