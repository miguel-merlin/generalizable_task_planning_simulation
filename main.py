from simulation.environment import setup_environment
from simulation.objects import load_table, load_cube
import pybullet as p
import time


TABLE_HEIGHT = 1
CUBE_SIZE = 0.2

def main():
    physics_client_id = setup_environment()

    # Load table
    load_table(physics_client_id)
    
    # Load cubes on top of the table at different positions
    load_cube(
            physics_client_id,
            base_position = [0.2, 0, TABLE_HEIGHT + CUBE_SIZE/2],
            cube_size = CUBE_SIZE
    )

    load_cube(
        physics_client_id,
        base_position = [0, 0.2, TABLE_HEIGHT + CUBE_SIZE/2],
        cube_size = CUBE_SIZE
    )

    load_cube(
        physics_client_id,
        base_position = [-0.2, -0.2, TABLE_HEIGHT + CUBE_SIZE/2],
        cube_size = CUBE_SIZE
    )

    # Simulation Loop
    for i in range(10000):
        p.stepSimulation()
        time.sleep(1./240.)

    p.disconnect()


if __name__ == "__main__":
    main()
