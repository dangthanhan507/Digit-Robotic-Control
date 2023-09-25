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
        joint_info = p.getJointInfo(robotId, i)
        joint_idx = joint_info[0]
        joint_name = joint_info[1]
        joint_type = joint_info[2]
        qIndex = joint_info[3]
        uIndex = joint_info[4]
        jointDamping = joint_info[6]
        jointFriction = joint_info[7]
        joint_lower_limit = joint_info[8]
        joint_upper_limit = joint_info[9]
        joint_max_force = joint_info[10]
        joint_max_velocity = joint_info[11]
        linkName = joint_info[12]

        print('Start Joint Print', i)
        print('==================')
        print('joint_idx: ', joint_idx)
        print('joint_name: ', joint_name)
        print('joint_type: ', joint_type)
        print('qIndex: ', qIndex)
        print('uIndex: ', uIndex)
        print('jointDamping: ', jointDamping)
        print('jointFriction: ', jointFriction)
        print('joint_lower_limit: ', joint_lower_limit)
        print('joint_upper_limit: ', joint_upper_limit)
        print('joint_max_force: ', joint_max_force)
        print('joint_max_velocity: ', joint_max_velocity)
        print('linkName: ', linkName)

        print('\n\n')

        

    p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, targetPosition=0, force=500)

    while True:
        print(p.getBasePositionAndOrientation(robotId))
        p.stepSimulation()
    p.disconnect()