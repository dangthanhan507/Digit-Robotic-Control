import numpy as np
import pybullet as p
import pybullet_data

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)

    #set start position

    startPos = [0,0,1]
    startOrientation = p.getQuaternionFromEuler([np.pi/2,0,0])

    planeId = p.loadURDF("plane.urdf")
    robotId = p.loadURDF("urdf/digit_model.urdf", startPos, startOrientation)

    while True:
        p.stepSimulation()
    p.disconnect()
