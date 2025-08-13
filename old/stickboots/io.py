from __future__ import annotations

import hashlib
import json
from dataclasses import asdict
from pathlib import Path
from typing import Tuple

from .config import StickbotParams


def params_hash(params: StickbotParams) -> str:
    payload = asdict(params)
    blob = json.dumps(payload, sort_keys=True).encode("utf-8")
    return hashlib.sha1(blob).hexdigest()[:12]


def prepare_output_dirs(base_out: Path, key: str) -> Tuple[Path, Path]:
    base_out = Path(base_out)
    robot_dir = base_out / key
    mesh_dir = robot_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    return robot_dir, mesh_dir
