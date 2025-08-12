from dataclasses import dataclass


@dataclass
class FeetVars:
    """
    Parameters for foot geometry generation
    """
    X: float = 0.24
    Y: float = 0.24
    Z: float = 0.24
    box_x: float = 0.101
    box_y: float = 0.0527
    scad_fn: int = 100


@dataclass
class StickbotParams:
    """
    Parameters for robot URDF generation
    """
    gap_ft: float = 0.032
    w_arm: float = 0.0625
    l_arm: float = 0.104
    l_leg: float = 0.153
    hip_offset: float = -0.01
    leg_mass: float = 0.1
    feet_mass: float = 0.13
    hand_mass: float = 0.15
    feet_vars: FeetVars = FeetVars()


@dataclass
class CompositionConfig:
    """
    Configuration for the composition of the stickbot
    """
    params: StickbotParams = StickbotParams()
    left_color = "1 0 0 0.5"
    right_color = "0 0 1 0.5"
    mass_color = "0 1 0 0.5"

    # scaling visual parameters, harcoded to look roughly nice
    viz_scaling = params.l_leg / 0.153
    s = 0.005 * viz_scaling
    s_hand = 0.02 * viz_scaling
    # motor visual dimension
    mot_x = 0.02 * viz_scaling
    mot_y = 0.01 * viz_scaling
    mot_z = 0.02 * viz_scaling

    # dict to store config of robot from params
    comp_config = {
        'leg_motor': {
            'xyz': [0, mot_y/2, 0],
            'size': [mot_x, mot_y, mot_z]
        },
        'leg_axel': {
            'xyz': [0, params.gap_ft/4, 0],
            'size': [s, params.gap_ft/2, s]
        },
        'leg_arm_axel': {
            'xyz': [0, -params.w_arm/2, 0],
            'size': [s, params.w_arm, s]
        },
        'leg_arm': {
            'xyz': [0, -params.w_arm, -params.l_arm/2],
            'size': [s, s, params.l_arm]
        },
        'leg_hand_mass': {
            'xyz': [0, -params.w_arm, -params.l_arm],
            'size': [s_hand, s_hand, s_hand],
            'mass': params.hand_mass
        },
        'leg_link': {
            'xyz': [0, params.gap_ft/2, -params.l_leg/2],
            'size': [s, s, params.l_leg]
        },
        'leg_mass': {
            'xyz': [0, params.gap_ft/2, -params.l_leg/2],
            'size': [s_hand/2, s_hand/2, s_hand/2],
            'mass': params.leg_mass
        }
    }

    