# import omni
# import omni.kit.pipapi
# omni.kit.pipapi.install(“drake”, module=“pydrake”)
import csv
import os

curr_dir = os.getcwd()
print("Current directory:", curr_dir)
os.chdir(curr_dir)

import datetime
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy.integrate as integrate
from scipy.integrate import cumulative_trapezoid
from scipy.fft import fft, fftfreq
from scipy.optimize import curve_fit
from scipy.signal import find_peaks
from scipy.optimize import minimize
from numpy.random import rand
from IPython.display import HTML, display
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from itertools import product
from multiprocessing import Pool
from joblib import Parallel, delayed
import csv
# from scipy.optimize import differential_evolution
from pydrake.all import (
    Parser,
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    StartMeshcat,
    JointSliders,
    AddDefaultVisualization,
    ConstantVectorSource,
    MeshcatVisualizer,
    Simulator,
    LeafSystem,
    Linearize,
    DiscreteTimeLinearQuadraticRegulator,
    MatrixGain,
    LinearQuadraticRegulator,
    OutputPortSelection,
    InitializeAutoDiff,
    ExtractGradient,
    MultibodyPlant,
    RobotDiagramBuilder,
    DiscreteContactApproximation,
    ContactVisualizer,
    ContactVisualizerParams,
    Value,
    List,
    SpatialForce,
    ZeroOrderHold,
    RotationMatrix,
    RollPitchYaw,
    AddDefaultVisualization,
    Quaternion,
    AngleAxis,
    ScopedName,
    AbstractValue,
    ContactResults,
    LogVectorOutput,
    RigidTransform,
    Sphere,
    PointCloud,
    Rgba
)




def get_amplitude(array, array_name, stabilization_period):
    new_array = array[10000:]
    peak, _ = find_peaks(new_array)
    inverted_array= -new_array
    valley, _ = find_peaks(inverted_array)
    array_peaks = new_array[peak]
    array_valleys = new_array[valley]
    avg_array_amp = (np.mean(array_peaks) - np.mean(array_valleys))/2
    # print(f"Average {array_name} amplitude after gait is established (after 10 seconds) = {avg_array_amp}")
    array_amp = (np.max(new_array) - np.min(new_array))/2
    # print(f"Max {array_name} amplitude after gait is established (after 10 seconds) = {array_amp}")
    return avg_array_amp, array_amp

def angular_velocity_to_rpy_rates(omega_x, omega_y, omega_z, roll, pitch):
    rpy_rates = np.zeros((len(omega_x), 3))
    for i in range(len(omega_x)):
        # Compute the transformation matrix 
        T = np.array([
            [1, np.sin(roll[i])*np.tan(pitch[i]), np.cos(roll[i]) * np.tan(pitch[i])],
            [0, np.cos(roll[i]), -np.sin(roll[i])],
            [0, np.sin(roll[i])/np.cos(pitch[i]), np.cos(roll[i])/np.cos(pitch[i])]
        ])

        # Compute roll, pitch, yaw rates
        rpy_rates[i] = T @ np.array([omega_x[i], omega_y[i], omega_z[i]])
        roll_rates = rpy_rates[:, 0]
        pitch_rates = rpy_rates[:, 1]
        yaw_rates = rpy_rates[:, 2]
    
    return roll_rates, pitch_rates, yaw_rates

def get_sphere_inertia(mass, radius):
    inertia = (2/5) * mass * radius**2
    return inertia

def get_box_inertia(mass,dims):
    x = dims[0]
    y = dims[1]
    z = dims[2]
    ixx = (1/12) * mass * (y**2 + z**2)
    iyy = (1/12) * mass * (x**2 + z**2)
    izz = (1/12) * mass * (x**2 + y**2)
    inertia = [ixx, iyy, izz, x, y, z]
    # print("Inertia:", inertia)
    return inertia
