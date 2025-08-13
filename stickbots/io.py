import hashlib
import json 
from dataclasses import asdict
from pathlib import Path
from stickbots.config import FeetVars


def params_hash(params: FeetVars) -> str:
    """
    Generates a hash for the given parameters.
    """
    payload = asdict(params)
    blob = json.dumps(payload, sort_keys=True).encode("utf-8")
    return hashlib.md5(blob).hexdigest()[:12]


def prepare_output_dirs(base_out: Path, key: str) -> tuple[Path, Path]:
    """
    Prepares the output directories for the given key.
    """
    base_out = Path(base_out)
    robot_dir = base_out / key
    mesh_dir = robot_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    return robot_dir, mesh_dir