from simulation.environment import setup_environment
from simulation.objects import load_table, load_cube
import pybullet as p
import time

def main():
    physics_client_id = setup_environment()

    # Load table
    load_table(physics_client_id)
    
    # Load cube
    load_cube(physics_client_id)

    # Simulation Loop
    for i in range(10000):
        p.stepSimulation()
        time.sleep(1./240.)

    p.disconnect()


if __name__ == "__main__":
    main()
