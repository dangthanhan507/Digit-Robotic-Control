import pybullet as p
import pybullet_data


if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("urdf/digit_model.urdf")
    p.stepSimulation()
    input()
    p.disconnect()    