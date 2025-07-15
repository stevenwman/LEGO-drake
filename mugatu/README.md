This simulation is for simulating Mugatu.

Before doing anything, you'll need Ubuntu 22.04, python3, Drake (Dragon), gcc = 11, pip, python venv maker, jinja2, typeguard, empy, numpy<2, pandas>=2.2,matplotlib=3.8.4, scipy >=1.11. Install `Ubuntu 22.04`, `python 3.10`, `python venv maker`, `gcc`, `jinja2`, `typeguard`, `empy`, and `Drake (Dragon)` first. Install in that order. Then run `requirements.txt` includes to get additional dependencies that you need (`pip install -r requirements.txt`)

`utiilities.py` includes functions that are referenced throughout the sim.

`plot_utilities` includes functions to plot data.

`animate_utilities` includes functions to animate plots. Right now, it's not used but you can use these functions by calling out generate_com_videos in the simulate.py script.

`meshcat_setup.py` sets up the visualization (MeshCat) of the simulation.

`simulate_.py` simulates the full script.

The `__init__.py` links all the scripts and functions together.


Get into the correct folder and your python environment that has Drake:
'source drake_gym_env/bin/activate' where 'drake_gym_env' is the name of my virtual environment
'cd Documents/Scaling_Analysis/mugatu' because that's where all my stuff is saved

Your command line should look something like this: 
(drake_gym_env) naomio@naomio-Alienware-Aurora-R16:~/Documents/Scaling_Analysis/mugatu$ 

Args explanation 

`--simulate_walker`: simulates the walker with actuation, control, and model parameters
`--duration`: used with `--simulate_walker` to specify sim time
`--visualize_coms`: shows COM trajectories in MeshCat visualization
`--save_data`: used with simulate_walker, saves data in CSV and plots data
`--scale`: scales up walker

If you just want to run joint sliders so you can look at the walker (URDF visualization), try '`python3 simulate_mugatu.py`'. You will have to hit 'Stop JointSliders' in the bottom right of the MeshCat sim to stop the URDF visualization.

If you want to simulate the walking dynamics, try '`python3 simulate_mugatu.py --simulate_walker`'. This simulates with a default duration of 30 seconds.

If you want to simulate the walking dynamics, try '`python3 simulate_mugatu.py --simulate_walker --duration 25`' where '25' is the duration of the sim in seconds. If you don't set the duration, it defaults to 7 seconds.

If you want to save the data from the sim, try '`python3 simulate_mugatu.py --simulate_walker --save_data --duration 25`'

If you want to visualize the COM trajectories of each link, try '`python3 simulate_mugatu.py --simulate_walker --visualize_coms`'

If you want a bigger walker try scaling up the robot with --scale. Try '`python3 simulate_mugatu.py --simulate_walker --visualize_coms --scale 2`'. If you don't set the scale, it defaults to 1x.

Example terminal command that does all of these things: '`python3 simulate_mugatu.py --simulate_walker --visualize_coms --save_data --duration 10 --scale 2`'


