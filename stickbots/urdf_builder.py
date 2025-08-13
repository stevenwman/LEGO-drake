from pathlib import Path
import numpy as np
from .config import *
from .xml_helper import *


def build_urdf(
        urdf_params: StickbotParams,
        mesh_params: FeetVars,
        urdf_dir: Path,
        urdf_key: str
) -> Path:
    """
    Generate URDF file for the stickbot.
    """
    robot = ET.Element('robot', name="walker")
    left_leg = ET.SubElement(robot, 'link', name='left_leg')
    right_leg = ET.SubElement(robot, 'link', name='right_leg')
    side_dict = {'left': left_leg, 'right': right_leg}

    mass_link_parents = dict()
    comp_config = CompConfig(urdf_params).comp_config

    for side, link in side_dict.items():
        for comp_name, comp_params in comp_config.items():
            color = CompConfig.left_color if side == 'left' else CompConfig.right_color
            # Define y-offset for left and right sides of robot
            y_val = comp_params['xyz'][1] if side == 'left' else -comp_params['xyz'][1]
            xyz = np.array([comp_params['xyz'][0], y_val, comp_params['xyz'][2]])
            visual_root = link

            if 'mass' in comp_name:
                color = CompConfig.mass_color
                J = 0 * np.eye(3)
                mass_link, mass_link_name = add_mass_link(
                    robot, side, comp_name, J, 0, comp_params['mass']
                )
                mass_link_parents[mass_link_name] = {'parent': link.get('name'), 'xyz': xyz}
                xyz = np.zeros(3)
                visual_root = mass_link

            add_box_visual(visual_root, f"{side}_{comp_name}", xyz=' '.join(map(str, xyz)), 
                           size=' '.join(map(str, comp_params['size'])), color=color)
            
        # Add feet link with mesh
        color = CompConfig.left_color if side == 'left' else CompConfig.right_color
        y_val = urdf_params.gap_ft/2 if side == 'left' else -urdf_params.gap_ft/2
        xyz = np.array([-urdf_params.hip_offset, y_val, -urdf_params.l_leg])
        mass_y_offset = mesh_params.box_y * ((side == 'left') - 0.5)
        J = 0 * np.eye(3)
        
        mass_link, mass_link_name = add_mass_link(
            robot, side, 'foot', J, mass_y_offset, urdf_params.feet_mass
            )       
        mass_link_parents[mass_link_name] = {'parent': link.get('name'), 'xyz': xyz}
        
        for mesh_type in ['visual', 'collision']:
            mesh_tag = add_mesh(mass_link, mesh_type, f"{side}_leg_foot_{mesh_type}",
                                xyz="0 0 0", filename=f"meshes/{side}_foot_geom.obj",
                                color=color)
            if mesh_type == 'collision': add_drake_tag(mesh_tag, urdf_params)

    add_rev_joint(robot, 'hip', parent='left_leg', child='right_leg', pos="0 0 0")
    for link, val in mass_link_parents.items():
        xyz = val['xyz']
        add_fixed_joint(
            robot, f"{link}_joint", parent=val['parent'], child=link, pos=' '.join(map(str, xyz))
            )
        
    add_transmission(robot)
    add_ground(robot, urdf_params)

    # Export
    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ", level=0)
    urdf_path = Path(urdf_dir) / f"stickbot_{urdf_key}.urdf"
    tree.write(urdf_path, encoding='utf-8', xml_declaration=True)
    return urdf_path
