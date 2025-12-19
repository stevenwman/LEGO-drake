from pathlib import Path
from typing import NamedTuple
from .config import *
from .io import *
from .urdf_builder import build_urdf
from .geom_builder import generate_feet_geom
import json, os
from filelock import FileLock


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
    urdf_json = robot_dir / f"urdf_params_{urdf_key}.json"
    mesh_json = mesh_dir / f"mesh_params_{mesh_key}.json"
    
    mesh_lock = mesh_dir / "mesh.lock"
    with FileLock(str(mesh_lock)):
        if left_obj.exists() and right_obj.exists():
            #print("Mesh already exists.", end=" ")
            None
        else:
            stickbot_dir = Path(__file__).resolve().parents[0]
            generate_feet_geom(mesh_params, str(stickbot_dir), mesh_dir)
            with open(mesh_json, 'w') as f:
                json.dump(mesh_params.__dict__, f, indent=2)

    if urdf_path.exists():
        print("URDF already exists.")
    else:
        print("")
        urdf_path = build_urdf(urdf_params, mesh_params, robot_dir, urdf_key)
        with open(urdf_json, 'w') as f:
            json.dump(urdf_params.__dict__, f, indent=2)

    return BuildResult(
        robot_dir=robot_dir, 
        urdf_path=urdf_path, 
        mesh_dir=mesh_dir, 
        urdf_key=urdf_key,
        mesh_key=mesh_key
    )