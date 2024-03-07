import pybullet as p

def load_table(physics_client_id):
    # Load table
    table_id = p.loadURDF("table/table.urdf", basePosition=[0,0,0], physicsClientId=physics_client_id)
    return table_id


def load_cube(physics_client_id, base_position=[0,0,1], base_orientation=[0,0,0,1], cube_size=1, mass=1):
    """
    Load a cube into the PyBullet simulation.

    Parameters:
    - physics_client_id: The ID of the physics client where the cube will be loaded.
    - base_position: The starting position of the cube as a list [x,y,z].
    - base_orientation: The starting orientation of the cube as a quaternion [x, y, z, w].
    - cube_size: The size of the cube (length of each edge)
    - mass: The mass of the cube

    Returns:
    - cube_id: The ID of the loaded cube
    """

    # Collision shape and visual shape
    collision_shape_id = p.createCollisionShape(shapeType = p.GEOM_BOX, halfExtents = [cube_size/2]*3, physicsClientId=physics_client_id)
    visual_shape_id = p.createVisualShape(
                shapeType = p.GEOM_BOX,
                halfExtents = [cube_size/2]*3,
                rgbaColor=[1,0,0,1],
                physicsClientId = physics_client_id
            )

    # Create the Cube
    cube_id = p.createMultiBody(
                baseMass = mass,
                baseCollisionShapeIndex = collision_shape_id,
                baseVisualShapeIndex = visual_shape_id,
                basePosition = base_position,
                baseOrientation = base_orientation,
                physicsClientId = physics_client_id
            )
    return cube_id
