import pybullet as p

def load_table(physics_client_id):
    # Load table
    table_id = p.loadURDF("table/table.urdf", basePosition=[0,0,0], physicsClientId=physics_client_id)
    return table_id
