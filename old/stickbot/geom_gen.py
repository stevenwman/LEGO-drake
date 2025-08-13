import os
import subprocess
import yaml
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

def _generate_single_foot(output_dir: Path, scad_file: Path, prefix: str, ft_prm: dict, left_foot_val: int):
    """Generates geometry for a single foot by calling OpenSCAD."""
    output_obj = output_dir / f"{prefix}foot_geom.obj"
    scad_fn = ft_prm.get("scad_fn", 100)  # Default resolution of 100 if not specified

    # Define variables to pass to OpenSCAD via the command line
    scad_vars = {
        "X": ft_prm["X"], "Y": ft_prm["Y"], "Z": ft_prm["Z"],
        "box_x": ft_prm["box_x"], "box_y": ft_prm["box_y"],
        "left_foot": left_foot_val, "fn": scad_fn
    }

    # Construct the command to call OpenSCAD, exporting directly to OBJ
    command = ["openscad-nightly"] 
    for key, value in scad_vars.items():
        command.extend(["-D", f"{key}={value}"])
    command.extend(["-o", str(output_obj), "--export-format", "obj"])
    command.append(str(scad_file))

    try:
        # Execute the OpenSCAD command
        subprocess.run(command, check=True, capture_output=True, text=True)
        print(f"✅ Generated {output_obj.name}")
    except subprocess.CalledProcessError as e:
        print(f"❌ Error generating {prefix}foot:\n{e.stderr}")
        raise

def generate_feet_geom(ft_prm: dict, script_dir: str) -> None:
    """
    Generates left and right foot geometries in parallel.
    Skips generation if the files already exist.
    """
    script_path = Path(script_dir)
    output_dir = script_path / ft_prm['file_id']
    scad_file = script_path / "feet_generator.scad"

    output_dir.mkdir(mode=0o777, exist_ok=True)

    # Caching: Skip if geometries already exist
    if (output_dir / "left_foot_geom.obj").exists() and \
       (output_dir / "right_foot_geom.obj").exists():
        print(f"Geometries already exist in {output_dir}. Skipping generation.")
        return

    # Use a ThreadPoolExecutor to run OpenSCAD instances in parallel
    tasks = [
        {'prefix': 'left_', 'left_foot_val': 1},
        {'prefix': 'right_', 'left_foot_val': -1}
    ]
    with ThreadPoolExecutor(max_workers=2) as executor:
        futures = [executor.submit(_generate_single_foot, output_dir, scad_file, t['prefix'], ft_prm, t['left_foot_val']) for t in tasks]
        for future in futures:
            future.result()  # Wait for all tasks to complete

    # Save the parameters used for this geometry
    with open(output_dir / "feet_params.yaml", "w") as f:
        yaml.dump(ft_prm, f, default_flow_style=False)