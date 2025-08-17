This simulation is for simulating Mugatu.

(Legacy) Before doing anything, you'll need Ubuntu 22.04, python3, Drake (Dragon), gcc = 11, pip, python venv maker, jinja2, typeguard, empy, numpy<2, pandas>=2.2,matplotlib=3.8.4, scipy >=1.11. Install `Ubuntu 22.04`, `python 3.10`, `python venv maker`, `gcc`, `jinja2`, `typeguard`, `empy`, and `Drake (Dragon)` first. 

Install in that order, run `requirements.txt` includes to get additional dependencies that you need (`pip install -r requirements.txt`)


## Simulation (WiP)
To just view URDF, use `python3 simulate.py`.
To run walker sim, use `python3 simulate.py --walk`.


## Design Optimization
Install `openscad` using the listed submodule in `\extern\openscad`

To run NES with default args, use `python3 NES.py` after installing all dependencies.
