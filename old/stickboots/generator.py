from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

from .config import StickbotParams


def _add_box_visual(link: ET.Element, name: str, xyz: str, size: str, color: str) -> None:
    visual = ET.SubElement(link, 'visual', name=name)
    ET.SubElement(visual, 'origin', xyz=xyz, rpy="0 0 0")
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'box', size=size)
    ET.SubElement(ET.SubElement(visual, 'material'), 'color', rgba=color)


def _add_mesh(link: ET.Element, type: str, name: str, xyz: str, filename: str, color: str) -> ET.Element:
    mesh_tag = ET.SubElement(link, type, name=name)
    ET.SubElement(mesh_tag, 'origin', xyz=xyz, rpy="0 0 0")
    ET.SubElement(ET.SubElement(mesh_tag, 'geometry'), 'mesh', filename=filename)
    ET.SubElement(ET.SubElement(mesh_tag, 'material'), 'color', rgba=color)
    return mesh_tag


def _add_rev_joint(link: ET.Element, name: str, parent: str, child: str, pos: str) -> None:
    joint = ET.SubElement(link, 'joint', name=name, type='revolute')
    ET.SubElement(joint, 'origin', xyz=pos, rpy="0 0 0")
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'axis', xyz="0 1 0")
    ET.SubElement(joint, 'limit', lower=f"{-3.14/4}", upper=f"{3.14/4}")


def _add_fixed_joint(link: ET.Element, name: str, parent: str, child: str, pos: str) -> None:
    joint = ET.SubElement(link, 'joint', name=name, type='fixed')
    ET.SubElement(joint, 'origin', xyz=pos, rpy="0 0 0")
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)


def build_urdf(
    params: StickbotParams,
    out_dir: Path,
    mesh_dir: Path | None = None,
    urdf_prefix_dir: Path | None = None,
    file_id: str | None = None,
) -> Path:
    """Compose the URDF with links, visuals, collisions, dynamics, and ground.

    Mesh filenames can be referenced either via:
    - mesh_dir: local relative paths (build folder copies)
    - or urdf_prefix_dir + file_id: absolute file URI paths under stickbot/<file_id>/,
      matching original generate_stickbot behavior.
    """
    urdf_path = Path(out_dir) / "stickbot.urdf"

    left_color = "1 0 0 0.5"
    right_color = "0 0 1 0.5"
    mass_color = "0 1 0 0.5"

    # scaling visuals
    viz_scaling = params.l_leg / 0.153
    s = 0.005 * viz_scaling
    s_hand = 0.02 * viz_scaling
    mot_x = 0.02 * viz_scaling
    mot_y = 0.01 * viz_scaling
    mot_z = 0.02 * viz_scaling

    robot = ET.Element('robot', name='walker')
    left_leg = ET.SubElement(robot, 'link', name='left_leg')
    right_leg = ET.SubElement(robot, 'link', name='right_leg')

    side_dict = {'left': left_leg, 'right': right_leg}

    comp_config = {
        'leg_motor': { 'xyz': [0, mot_y/2, 0], 'size': [mot_x, mot_y, mot_z] },
        'leg_axel': { 'xyz': [0, params.gap_ft/4, 0], 'size': [s, params.gap_ft/2, s] },
        'leg_arm_axel': { 'xyz': [0, -params.w_arm/2, 0], 'size': [s, params.w_arm, s] },
        'leg_arm': { 'xyz': [0, -params.w_arm, -params.l_arm/2], 'size': [s, s, params.l_arm] },
        'leg_hand_mass': { 'xyz': [0, -params.w_arm, -params.l_arm], 'size': [s_hand, s_hand, s_hand], 'mass': params.hand_mass },
        'leg_link': { 'xyz': [0, params.gap_ft/2, -params.l_leg/2], 'size': [s, s, params.l_leg] },
        'leg_mass': { 'xyz': [0, params.gap_ft/2, -params.l_leg/2], 'size': [s_hand/2, s_hand/2, s_hand/2], 'mass': params.leg_mass },
    }

    mass_links_parents = {}

    for side, link in side_dict.items():
        for comp_name, comp_params in comp_config.items():
            y_val = comp_params['xyz'][1] if side == 'left' else -comp_params['xyz'][1]
            color = left_color if side == 'left' else right_color
            color = mass_color if 'mass' in comp_name else color
            xyz = np.array([comp_params['xyz'][0], y_val, comp_params['xyz'][2]])
            if 'mass' in comp_name:
                link_name = f"{side}_{comp_name}"
                mass_link_name = f"{link_name}_link"
                mass_link = ET.SubElement(robot, 'link', name=mass_link_name)
                inertial = ET.SubElement(mass_link, 'inertial', name=f"{link_name}_inertial")
                mass_links_parents[mass_link_name] = {'parent': link.get('name'), 'xyz': xyz}
                ET.SubElement(inertial, 'origin', xyz=f"0 0 0", rpy="0 0 0")
                ET.SubElement(inertial, 'mass', value=f"{comp_params['mass']}")
                J = 0 * np.eye(3)
                ET.SubElement(inertial, 'inertia', ixx=f"{J[0,0]}", ixy=f"{J[0,1]}", ixz=f"{J[0,2]}", iyy=f"{J[1,1]}", iyz=f"{J[1,2]}", izz=f"{J[2,2]}")
                _add_box_visual(mass_link, f"{side}_{comp_name}", xyz=f"0 0 0", size=f"{comp_params['size'][0]} {comp_params['size'][1]} {comp_params['size'][2]}", color=color)
            else:
                _add_box_visual(link, f"{side}_{comp_name}", xyz=f"{xyz[0]} {xyz[1]} {xyz[2]}", size=f"{comp_params['size'][0]} {comp_params['size'][1]} {comp_params['size'][2]}", color=color)

        # Foot link with mesh
        y_val = params.gap_ft/2 if side == 'left' else -params.gap_ft/2
        color = left_color if side == 'left' else right_color
        link_name = f"{side}_foot"
        mass_link = ET.SubElement(robot, 'link', name=link_name)
        inertial = ET.SubElement(mass_link, 'inertial', name=f"{link_name}_inertial")
        xyz = np.array([-params.hip_offset, y_val, -params.l_leg])
        mass_links_parents[link_name] = {'parent': link.get('name'), 'xyz': xyz}
        mass_y_offset = params.feet_vars.box_y/2
        mass_y_offset = mass_y_offset if side == 'left' else -mass_y_offset
        ET.SubElement(inertial, 'origin', xyz=f"0 {mass_y_offset} 0", rpy="0 0 0")
        ET.SubElement(inertial, 'mass', value=f"{params.feet_mass}")
        J = 0 * np.eye(3)
        ET.SubElement(inertial, 'inertia', ixx=f"{J[0,0]}", ixy=f"{J[0,1]}", ixz=f"{J[0,2]}", iyy=f"{J[1,1]}", iyz=f"{J[1,2]}", izz=f"{J[2,2]}")

        if mesh_dir is not None:
            mesh_path = (mesh_dir / f"{side}_foot_geom.obj").as_posix()
        elif urdf_prefix_dir is not None and file_id is not None:
            urdf_prefix = urdf_prefix_dir.as_uri() + "/"
            mesh_path = f"{urdf_prefix}{file_id}/{side}_foot_geom.obj"
        else:
            raise ValueError("Either mesh_dir or (urdf_prefix_dir and file_id) must be provided")

        mesh_tag = _add_mesh(mass_link, 'collision', f'{side}_leg_foot_collision', xyz=f"0 0 0", filename=mesh_path, color=color)
        drake_tag = ET.SubElement(mesh_tag, 'drake:proximity_properties')
        ET.SubElement(drake_tag, 'drake:compliant_hydroelastic')
        ET.SubElement(drake_tag, 'drake:mu_dynamic', value=str(0.9))
        ET.SubElement(drake_tag, 'drake:mu_static', value=str(0.9))
        ET.SubElement(drake_tag, 'drake:mesh_resolution_hint', value=str(0.01))
        ET.SubElement(drake_tag, 'drake:hydroelastic_modulus', value=str(5e7))

        _add_mesh(mass_link, 'visual', f'{side}_leg_foot_visual', xyz=f"0 0 0", filename=mesh_path, color=color)

    _add_rev_joint(robot, 'hip', parent='left_leg', child='right_leg', pos=f"{0} {0} {0}")
    for link, dic in mass_links_parents.items():
        xyz = dic['xyz']
        _add_fixed_joint(robot, f"fixed_{link}", parent=dic['parent'], child=link, pos=f"{xyz[0]} {xyz[1]} {xyz[2]}")

    # Ground
    ground = ET.SubElement(robot, 'link', name='ground')
    ground_visual = ET.SubElement(ground, 'visual')
    ET.SubElement(ground_visual, 'origin', xyz="0 0 -0.25", rpy="0 0 0")
    ET.SubElement(ET.SubElement(ground_visual, 'geometry'), 'box', size="100 100 0.5")
    ET.SubElement(ET.SubElement(ground_visual, 'material'), 'color', rgba="0.93 .74 .4 1")
    ground_collision = ET.SubElement(ground, 'collision')
    ET.SubElement(ground_collision, 'origin', xyz="0 0 -0.25", rpy="0 0 0")
    ET.SubElement(ET.SubElement(ground_collision, 'geometry'), 'box', size="100 100 0.5")
    ET.SubElement(ground_collision, 'drake:rigid_hydroelastic')
    ET.SubElement(ground_collision, 'drake:mu_dynamic', value=str(0.9))
    ET.SubElement(ground_collision, 'drake:mu_static', value=str(0.9))
    ET.SubElement(ground_collision, 'drake:mesh_resolution_hint', value=str(0.01))
    _add_fixed_joint(robot, 'fixed_ground', parent='world', child='ground', pos="0 0 0")

    # Export
    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ", level=0)
    tree.write(urdf_path, encoding='utf-8', xml_declaration=True)
    return urdf_path
