import pybullet as p
import pybullet_data

def load_arm(physics, start_position, start_orientation):
    robot_arm_id = p.loadURDF("ur5.urdf", 
                            startPosition = start_position,
                            startOrientation=start_orientation,
                            useFixedBase = 1)
    return robot_arm_id
