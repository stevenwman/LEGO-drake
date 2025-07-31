# Click the link to open the animation. 
# While the simulation and calculations are going on, 
# the robot might look like it's lying face down dead in the floor. 
# This is fine. Just refresh once the animation is done simulating.

from pydrake.all import StartMeshcat


def start_meshcat():
    meshcat = StartMeshcat()
    print("_"*120)
    print(f" Meshcat is live. Click link: {meshcat.web_url()}")
    print("_"*120)
    return meshcat