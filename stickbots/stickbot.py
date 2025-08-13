from dataclasses import asdict
from pathlib import Path
from typing import NamedTuple
from .config import StickbotParams, SimParams
from .io import params_hash, prepare_output_dirs
from .urdf_builder import build_urdf
from .geom_builder import generate_feet_geom
import json


class BuildResult(NamedTuple):
    urdf_path: Path     
    robot_dir: Path
    mesh_dir: Path
    key: str


def build_stickbot(
        params: StickbotParams, 
        sim_params: SimParams,
        base_out_dir: Path | str = "robots"
) -> BuildResult:
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
        print("Mesh already exists, updating URDF.")
    else:
        stickbot_dir = Path(__file__).resolve().parents[0]
        generate_feet_geom(params.feet_vars, str(stickbot_dir), mesh_dir)

    urdf_path = build_urdf(params, sim_params, robot_dir)

    with open(robot_dir / "params.json", 'w') as f:
        json_dict = {
            "robot_params" : asdict(params),
            "sim_params" : asdict(sim_params)
        }
        json.dump(json_dict, f, indent=2)

    return BuildResult(urdf_path=urdf_path, 
                       robot_dir=robot_dir, 
                       mesh_dir=mesh_dir, 
                       key=key)


