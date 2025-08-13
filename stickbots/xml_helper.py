import xml.etree.ElementTree as ET
from .config import SimParams
import numpy as np


def add_box_visual(link: ET.Element, name: str, xyz: str, size: str, color: str) -> None:
    """Add a visual element to a link."""
    visual = ET.SubElement(link, 'visual', name=name)
    ET.SubElement(visual, 'origin', xyz=xyz, rpy="0 0 0")
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'box', size=size)
    ET.SubElement(ET.SubElement(visual, 'material'), 'color', rgba=color)


def add_mesh(link: ET.Element, type: str, name: str, xyz: str, filename: str, color: str) -> None:
    """Add a visual element to a link."""
    mesh_tag = ET.SubElement(link, type, name=name)
    ET.SubElement(mesh_tag, 'origin', xyz=xyz, rpy="0 0 0")
    ET.SubElement(ET.SubElement(mesh_tag, 'geometry'), 'mesh', filename=filename)
    ET.SubElement(ET.SubElement(mesh_tag, 'material'), 'color', rgba=color)
    return mesh_tag


def add_rev_joint(link: ET.Element, name: str, parent: str, child: str, pos: str) -> None:
    """Add a revolute joint to a link."""
    joint = ET.SubElement(link, 'joint', name=name, type='revolute')
    ET.SubElement(joint, 'origin', xyz=pos, rpy="0 0 0")
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'axis', xyz="0 1 0")
    ET.SubElement(joint, 'limit', lower=f"{-3.14/4}", upper=f"{3.14/4}")


def add_fixed_joint(link: ET.Element, name: str, parent: str, child: str, pos: str) -> None:
    """Add a fixed joint to a link."""
    joint = ET.SubElement(link, 'joint', name=name, type='fixed')
    ET.SubElement(joint, 'origin', xyz=pos, rpy="0 0 0")
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)


def add_drake_tag(mesh_tag: ET.Element, sim_params: SimParams) -> None:
    drake_tag = ET.SubElement(mesh_tag, 'drake:proximity_properties')
    ET.SubElement(drake_tag, 'drake:compliant_hydroelastic')
    ET.SubElement(drake_tag, 'drake:mu_dynamic', value=str(sim_params.feet_mu_dyn))
    ET.SubElement(drake_tag, 'drake:mu_static', value=str(sim_params.feet_mu_stat))
    ET.SubElement(drake_tag, 'drake:mesh_resolution_hint', value=str(sim_params.feet_mesh_resolution_hint))
    ET.SubElement(drake_tag, 'drake:hydroelastic_modulus', value=str(sim_params.hydroelastic_modulus))


def add_transmission(link: ET.Element) -> None:
    transm = ET.SubElement(link, 'transmission', name='hip_joint_transmission')
    ET.SubElement(transm, 'type').text = 'transmission_interface/SimpleTransmission'
    joint = ET.SubElement(transm, 'joint', name='hip')
    ET.SubElement(joint, 'hardwareInterface').text = 'hardware_interface/EffortJointInterface'
    actuator = ET.SubElement(transm, 'actuator', name='hip_joint_motor')
    ET.SubElement(actuator, 'hardwareInterface').text = 'hardware_interface/EffortJointInterface'
    ET.SubElement(transm, 'mechanicalReduction').text = '1.0'


def add_ground(link: ET.Element, sim_params: SimParams) -> None:
    ground = ET.SubElement(link, 'link', name='ground')
    ground_visual = ET.SubElement(ground, 'visual')
    ET.SubElement(ground_visual, 'origin', xyz="0 0 -0.25", rpy="0 0 0")
    ET.SubElement(ET.SubElement(ground_visual, 'geometry'), 'box', size="100 100 0.5")
    ET.SubElement(ET.SubElement(ground_visual, 'material'), 'color', rgba="0.93 .74 .4 1")

    ground_collision = ET.SubElement(ground, 'collision')
    ET.SubElement(ground_collision, 'origin', xyz="0 0 -0.25", rpy="0 0 0")
    ET.SubElement(ET.SubElement(ground_collision, 'geometry'), 'box', size="100 100 0.5")
    ET.SubElement(ground_collision, 'drake:rigid_hydroelastic')
    ET.SubElement(ground_collision, 'drake:mu_dynamic', value=str(sim_params.feet_mu_dyn))
    ET.SubElement(ground_collision, 'drake:mu_static', value=str(sim_params.feet_mu_stat))
    ET.SubElement(ground_collision, 'drake:mesh_resolution_hint', value=str(sim_params.feet_mesh_resolution_hint))
    add_fixed_joint(link, 'fixed_ground', parent='world', child='ground', pos="0 0 0")


def add_mass_link(
        robot: ET.Element, 
        side: str, 
        comp_name: str,
        inertia: np.array,
        y_offset: float,
        mass: float,
) -> None:
    link_name = f"{side}_{comp_name}"
    mass_link_name = f"{link_name}_link"
    mass_link = ET.SubElement(robot, 'link', name=mass_link_name)
    inertial = ET.SubElement(mass_link, 'inertial', name=f"{link_name}_inertial")
    J = inertia
    ET.SubElement(inertial, 'origin', xyz=f"0 {y_offset} 0", rpy="0 0 0")
    ET.SubElement(inertial, 'mass', value=str(mass))
    ET.SubElement(inertial, 'inertia',
                  ixx=f"{J[0,0]}", ixy=f"{J[0,1]}", ixz=f"{J[0,2]}", 
                  iyy=f"{J[1,1]}", iyz=f"{J[1,2]}", izz=f"{J[2,2]}")
    
    return mass_link, mass_link_name