from pathlib import Path
import numpy as np
from .config import StickbotParams, CompConfig, SimParams
from .xml_helper import *


def build_urdf(
        params: StickbotParams,
        sim_params: SimParams,
        urdf_dir: Path,
) -> Path:
    """
    Generate URDF file for the stickbot.
    """
    robot = ET.Element('robot', name="walker")
    left_leg = ET.SubElement(robot, 'link', name='left_leg')
    right_leg = ET.SubElement(robot, 'link', name='right_leg')

    side_dict = {
        'left': left_leg,
        'right': right_leg
    }

    mass_link_parents = dict()
    comp_config = CompConfig(params).comp_config    

    for side, link in side_dict.items():
        color = CompConfig.left_color if side == 'left_leg' else CompConfig.right_color
        for comp_name, comp_params in comp_config.items():
            # Define y-offset for left and right sides of robot
            y_val = comp_params['xyz'][1] if side == 'left_leg' else -comp_params['xyz'][1]
            xyz = np.array([comp_params['xyz'][0], y_val, comp_params['xyz'][2]])

            if 'mass' in comp_name:
                color = CompConfig.mass_color

                link_name = f"{side}_{comp_name}"
                mass_link_name = f"{link_name}_link"
                mass_link = ET.SubElement(robot, 'link', name=mass_link_name)
                inertial = ET.SubElement(mass_link, 'inertial', name=f"{link_name}_inertial")
                mass_link_parents[mass_link_name] = {'parent': link.get('name'), 'xyz': xyz}
                J = 0 * np.eye(3)
                ET.SubElement(inertial, 'origin', xyz="0 0 0", rpy="0 0 0")
                ET.SubElement(inertial, 'mass', value=str(comp_params['mass']))
                ET.SubElement(inertial, 'inertia', 
                              ixx=f"{J[0,0]}", ixy=f"{J[0,1]}", ixz=f"{J[0,2]}", 
                              iyy=f"{J[1,1]}", iyz=f"{J[1,2]}", izz=f"{J[2,2]}")
                xyz = np.zeros(3)
                link = mass_link

            add_box_visual(link, 
                            f"{side}_{comp_name}", xyz=' '.join(map(str, xyz)), 
                            size=' '.join(map(str, comp_params['size'])), 
                            color=color)
        # Add feet link with mesh

        # TODO: refactor mass generation code
        y_val = params.gap_ft/2 if side == 'left_leg' else -params.gap_ft/2
        xyz = np.array([-params.hip_offset, y_val, -params.l_leg])
        mass_y_offset = params.feet_vars.box_y/2
        mass_y_offset = mass_y_offset if side == 'left_leg' else -mass_y_offset

        link_name = f"{side}_foot"
        mass_link_name = f"{link_name}_link"
        mass_link = ET.SubElement(robot, 'link', name=mass_link_name)
        inertial = ET.SubElement(mass_link, 'inertial', name=f"{link_name}_inertial")
        mass_link_parents[mass_link_name] = {'parent': link.get('name'), 'xyz': xyz}
        J = 0 * np.eye(3)
        ET.SubElement(inertial, 'origin', xyz=f"0 {mass_y_offset} 0", rpy="0 0 0")
        ET.SubElement(inertial, 'mass', value=str(params.feet_mass))
        ET.SubElement(inertial, 'inertia', 
                      ixx=f"{J[0,0]}", ixy=f"{J[0,1]}", ixz=f"{J[0,2]}", 
                      iyy=f"{J[1,1]}", iyz=f"{J[1,2]}", izz=f"{J[2,2]}")
        
        for mesh_type in ['visual', 'collision']:
            mesh_tag = add_mesh(mass_link, mesh_type, f"{side}_leg_foot_{mesh_type}",
                                xyz="0 0 0", filename=f"meshes/{side}_foot_geom.obj",
                                color=color)
            
            if mesh_type == 'collision': add_drake_tag(mesh_tag, sim_params)

        add_rev_joint(robot, 'hip', parent='left_leg', child='right_leg', pos="0 0 0")
        for link, val in mass_link_parents.items():
            xyz = val['xyz']
            add_fixed_joint(robot, f"{link}_joint", 
                            parent=val['parent'], child=link, 
                            pos=' '.join(map(str, xyz)))

        add_ground(robot, sim_params)

        # Export
        tree = ET.ElementTree(robot)
        ET.indent(tree, space="  ", level=0)
        urdf_path = Path(urdf_dir) / "stickbot.urdf"
        tree.write(urdf_path, encoding='utf-8', xml_declaration=True)
        return urdf_path
