import pybullet as p
import pybullet_data
import time

def main():
    physicsClient = p.connect(p.GUI)

    # Add pybullet_data search path
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load URDF
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF("plane.urdf")
    tableId = p.loadURDF("table/table.urdf", basePosition = [0.5, 0, -0.65])
    blockId = p.loadURDF("block.urdf", basePosition = [0.5, 0, -0.2])
    robotId = p.loadURDF("kuka_iiwa/model.urdf", basePosition = [0,0,0])

    # Calculate inverse kinematics for robot arm
    targetPosition = [0.5, 0, 0]
    targetOrientation = p.getQuaternionFromEuler([0,0,0])
    jointPosition = p.calculateInverseKinematics(robotId, 6, targetPosition, targetOrientation)

    # Apply joint positions to reach for the block
    for i, jointPosition in enumerate(jointPosition):
        p.setJointMotorControl2(robotId, i, p.POSITION_CONTROL, targetPosition = jointPosition)

    # Simulation Loop
    for i in range(100):
        p.stepSimulation()
        time.sleep(1./240.)
    
    p.disconnect()


if __name__ == '__main__':
    print("Running Simulation")
    main()
