import pybullet as p
import pybullet_data

def setup_environment():
    # Initialize PyBullet in GUI mode
    physics_client_id = p.connect(p.GUI)

    # Set Gravity
    p.setGravity(0, 0, -10, physicsClientId=physics_client_id)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load plane
    p.loadURDF("plane.urdf", physicsClientId=physics_client_id)

    return physics_client_id
