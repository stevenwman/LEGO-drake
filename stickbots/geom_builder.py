import subprocess
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from .config import FeetVars
from dataclasses import fields


def _generate_single_foot(
        output_dir: Path, 
        scad_file: Path, 
        prefix: str, 
        feet_params: FeetVars, 
        left_foot_flag: int,
        verbose: bool = False,
) -> None:
    """
    Generates geometry for a single foot by calling OpenSCAD.
    """
    output_dir_obj = output_dir / f"{prefix}foot_geom.obj"
    command = ["openscad"]
    # Iterate through params to be added to the openSCAD command
    for field in fields(feet_params):
        command.extend(["-D", f"{field.name}={getattr(feet_params, field.name)}"])
    command.extend(["-D", f"left_foot={left_foot_flag}"])
    command.extend(["-o", str(output_dir_obj), "--export-format", "obj", str(scad_file)])
    
    try: 
        subprocess.run(command, check=True, capture_output=True, text=True)
        if verbose: print(f"✅ Generated {output_dir_obj.name}")
    except subprocess.CalledProcessError as e:
        print(f"❌ Error generating {output_dir_obj.name}: {e.stderr}")
        raise


def generate_feet_geom(
        feet_params: FeetVars, 
        module_dir: str,
        output_dir: str
) -> None:
    """
    Generates left and right foot geometries in parallel.    
    """
    module_path = Path(module_dir)
    output_dir = Path(output_dir)
    scad_file = module_path / "feet_generator.scad"

    output_dir.mkdir(mode=0o777, exist_ok=True)

    if (output_dir / "left_foot_geom.obj").exists() and \
       (output_dir / "right_foot_geom.obj").exists():
        print("✅ Both foot geometries already exist.")
        return

    tasks = [
        {'prefix': 'left_', 'left_foot_flag': 1},
        {'prefix': 'right_', 'left_foot_flag': -1}
    ]

    with ThreadPoolExecutor(max_workers=2) as executor:
        futures = [executor.submit(_generate_single_foot, 
                                   output_dir, scad_file, 
                                   t['prefix'], 
                                   feet_params, 
                                   t['left_foot_flag']) for t in tasks]
        for future in futures:
            future.result()  # Wait for all tasks to complete


