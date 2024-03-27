import pybullet as p
import pybullet_data
import numpy as np
import h5py
import time
import os


def initialize_simulation():
    p.connect(p.DIRECT) # p.GUI for graphical simulation
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("franka_panda/panda.urdf", basePosition=[0,0,0], useFixedBase = True)
    table_id = p.loadURDF("table/table.urdf", basePosition = [0.5, 0, 0])

    base_position=[0,0,1]
    base_orientation=[0,0,0,1]
    cube_size=1
    mass=1
    collision_shape_id = p.createCollisionShape(shapeType = p.GEOM_BOX, halfExtents = [cube_size/2]*3)
    visual_shape_id = p.createVisualShape(
                shapeType = p.GEOM_BOX,
                halfExtents = [cube_size/2]*3,
                rgbaColor=[1,0,0,1]
            )

    # Create the Cube
    cube_id = p.createMultiBody(
                baseMass = mass,
                baseCollisionShapeIndex = collision_shape_id,
                baseVisualShapeIndex = visual_shape_id,
                basePosition = base_position,
                baseOrientation = base_orientation,
            )
    return robot_id, cube_id

def get_joint_states(robot_id):
    joint_states = p.getJointStates(robot_id, range(p.getNumJoints(robot_id)))
    joint_positions = [state[0] for state in joint_states]
    return joint_positions

def simulate_reach(robot_id, target_position):
    # Move robot's end effector to target position
    # TODO: Add motion planning/control logic
    simulation_steps = 240
    for _ in range(simulation_steps):
        p.stepSimulation()
        time.sleep(1/simulation_steps)

def create_dataset(num_samples = 10, robot_id = None, object_id = None):
    """
    Create dataset with
    - points clouds
    - binary_masks
    - joint_states
    - targets
    """
    point_clouds = []
    binary_masks = []
    joint_states = []
    targets = []

    for _ in range(num_samples):
        # Simulate and capture data
        simulate_reach(robot_id, [0.5, 0, 0.5])
        joint_state = get_joint_states(robot_id)

        # TODO: Replace with actual point cloud
        point_cloud = np.random.rand(100,3) 
        
        # TODO: Add correct binary_mask
        binary_mask = [1.0] * 3

        # TODO: Change to target joint states or positions
        target = joint_state

        point_clouds.append(point_cloud.flatten())
        binary_masks.append(binary_mask)
        joint_states.append(joint_state)
        targets.append(target)
    
    point_clouds = np.array(point_clouds)
    binary_masks = np.array(binary_masks)
    joint_states = np.array(joint_states)
    targets = np.array(targets)

    return point_clouds, binary_masks, joint_states, targets

def save_to_hdf5(filename, data):
    with h5py.File(filename, 'w') as hf:
        for key, value in data.items():
            hf.create_dataset(key, data=value)

def main():
    print("Initializing simulation")
    robot_id, object_id = initialize_simulation()

    print("Creating dataset")
    # Generate dataset
    data = create_dataset(num_samples=2, robot_id=robot_id, object_id=object_id)

    print("Saving dataset")
    # Save dataset to HDF5
    dataset = {
        "point_clouds": data[0],
        "binary_masks": data[1],
        "joint_states": data[2],
        "targets": data[3]
    }
    path = os.path.join("dataset", "robot_reach_dataset.h5")
    save_to_hdf5(path, dataset)

    print("Dataset saved to 'dataset/robot_reach_dataset.h5'")
    p.disconnect()


if __name__ == "__main__":
    main()