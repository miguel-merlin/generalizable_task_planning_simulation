from simulation.environment import setup_environment
from simulation.objects import load_table
import pybullet as p
import time

def main():
    physics_client_id = setup_environment()

    # Load objects to simulation
    load_table(physics_client_id)

    # Simulation Loop
    for i in range(10000):
        p.stepSimulation()
        time.sleep(1./240.)

    p.disconnect()


if __name__ == "__main__":
    main()
