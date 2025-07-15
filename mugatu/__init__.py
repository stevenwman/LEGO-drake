"""
Mugatu Walker Simulation Package

This package contains modular components for simulating and analyzing
Mugatu.

Modules:
- utilities: Common scientific imports and helper functions
- model_definition: URDF generation, robot setup, contact classes, controller info
- plot_utilities: Functions for plotting COMs, forces, energy, etc.
- animate_utilities: Functions to generate animations and videos
- meshcat_setup: Launches Meshcat visualizer
"""

from .utilities import *
from .plot_utilities import *
from .animate_utilities import *
from .model_definition import *
from .meshcat_setup import *
