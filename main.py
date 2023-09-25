import numpy as np
import pybullet as p
import pybullet_data

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)

    #set start position

    startPos = [0,0,0.8]
    # startOrientation = p.getQuaternionFromEuler([np.pi/2,0,0]) # everything in radians
    startOrientation = p.getQuaternionFromEuler([0,0,0])

    planeId = p.loadURDF("plane.urdf")
    robotId = p.loadURDF("urdf/digit_model.urdf", startPos, startOrientation)
    p.setRealTimeSimulation(1)

    print('Num Joints:', p.getNumJoints(robotId))
    num_joints = p.getNumJoints(robotId)
    for i in range(num_joints):
        print(p.getJointInfo(robotId, i), end='\n\n')
    input()

    while True:
        print(p.getBasePositionAndOrientation(robotId))
        p.stepSimulation()
    p.disconnect()