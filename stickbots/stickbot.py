from dataclasses import asdict
from pathlib import Path
from typing import NamedTuple
from .config import *
from .io import *
from .urdf_builder import build_urdf
from .geom_builder import generate_feet_geom
import json, os


class BuildResult(NamedTuple):
    urdf_path: Path     
    robot_dir: Path
    mesh_dir: Path
    urdf_key: str
    mesh_key: str


def build_stickbot(
        urdf_params: StickbotParams, 
        mesh_params: FeetVars,
        base_out_dir: Path | str = "robots"
) -> BuildResult:
    """
    Generate a URDF and meshes given robot parameters
    """
    base_out_path = Path(base_out_dir)
    urdf_key = link_params_hash(urdf_params)
    mesh_key = mesh_params_hash(mesh_params)
    robot_dir, mesh_dir = prepare_output_dirs(Path(base_out_path), mesh_key)

    # Cache: if URDF exists and both meshes exist, return quickly
    urdf_path = robot_dir / f"stickbot_{urdf_key}.urdf"
    left_obj = mesh_dir / "left_foot_geom.obj"
    right_obj = mesh_dir / "right_foot_geom.obj"
    
    if left_obj.exists() and right_obj.exists():
        print("Mesh already exists.")
    else:
        stickbot_dir = Path(__file__).resolve().parents[0]
        generate_feet_geom(mesh_params, str(stickbot_dir), mesh_dir)
        with open(mesh_dir / f"mesh_params_{mesh_key}.json", 'w') as f:
            json.dump(asdict(mesh_params), f, indent=2)

    if urdf_path.exists():
        print("URDF already exists.")
    else:
        urdf_path = build_urdf(urdf_params, mesh_params, robot_dir, urdf_key)
        with open(robot_dir / f"urdf_params_{urdf_key}.json", 'w') as f:
            json.dump(asdict(urdf_params), f, indent=2)

    return BuildResult(
        robot_dir=robot_dir, 
        urdf_path=urdf_path, 
        mesh_dir=mesh_dir, 
        urdf_key=urdf_key,
        mesh_key=mesh_key
    )