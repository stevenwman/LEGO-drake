import hashlib
import json 
from dataclasses import asdict
from pathlib import Path
from stickbots.config import *


def link_params_hash(params: StickbotParams) -> str:
    """
    Generates a hash for the given parameters.
    """
    payload = asdict(params)
    blob = json.dumps(payload, sort_keys=True).encode("utf-8")
    return hashlib.md5(blob).hexdigest()

def mesh_params_hash(feet_params: FeetVars) -> str:
    """
    Generates a hash for the given mesh parameters.
    """
    payload = asdict(feet_params)
    blob = json.dumps(payload, sort_keys=True).encode("utf-8")
    return hashlib.md5(blob).hexdigest()

def prepare_output_dirs(base_out: Path, mesh_key: str) -> tuple[Path, Path]:
    """
    Prepares the output directories for the given key.
    """
    base_out = Path(base_out)
    robot_dir = base_out / mesh_key
    mesh_dir = robot_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    return robot_dir, mesh_dir