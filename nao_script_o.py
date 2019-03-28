# This is edited for Nao robot to walk normally

import vrep
# import sys
import time
import copy
import numpy as np







print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,-500000,5)
if clientID != -1:
    print('Connected to remote API server')
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    emptyBuff = bytearray()

    # vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

    # Save all the joints in a list named Joints
    headJoints = ['HeadYaw', 'HeadPitch']
    # allJoints = ['HeadYaw', 'HeadPitch']
    rubbish = ['imported_part_20_sub0']
    leftLegJoints = ['LHipYawPitch3', 'LHipRoll3', 'LHipPitch3', 'LKneePitch3', 'LAnklePitch3', 'LAnkleRoll3']
    rightLegJoints = ['RHipYawPitch3', 'RHipRoll3', 'RHipPitch3', 'RKneePitch3', 'RAnklePitch3', 'RAnkleRoll3']
    leftArmJoints = ['LShoulderPitch3', 'LShoulderRoll3', 'LElbowYaw3', 'LElbowRoll3', 'LWristYaw3']
    leftHand = ['NAO_LThumbBase', 'Revolute_joint8', 'NAO_LLFingerBase', 'Revolute_joint12', 'Revolute_joint14',
                'NAO_LRFingerBase', 'Revolute_joint11', 'Revolute_joint13']
    rightArmJoints = ['RShoulderPitch3', 'RShoulderRoll3', 'RElbowYaw3', 'RElbowRoll3', 'RWristYaw3']
    rightHand = ['NAO_RThumbBase', 'Revolute_joint0', 'NAO_RLFingerBase', 'Revolute_joint5', 'Revolute_joint6',
                 'NAO_RRFingerBase', 'Revolute_joint2', 'Revolute_joint3']

    """print("Number of leftHand Joints:", len(leftHand))  # print the number of all the joints
    print(leftHand)  # print all the joints' names
    allJoints.extend(leftLegJoints)
    allJoints.extend(rightLegJoints)
    allJoints.extend(leftArmJoints)
    allJoints.extend(leftHand)
    allJoints.extend(rightArmJoints)
    allJoints.extend(rightHand)
    print("Number of all Joints:",len(allJoints))
    print(allJoints)"""
    LegJoints = copy.deepcopy(leftLegJoints)
    LegJoints.extend(rightLegJoints)
    print("LegJoints:", LegJoints)

    # handle the NAO object and plant object
    res, NAOHandle = vrep.simxGetObjectHandle(clientID, "NAO", vrep.simx_opmode_blocking)
    res, indoorPlantHandle = vrep.simxGetObjectHandle(clientID, "indoorPlant", vrep.simx_opmode_blocking)
    res, HeadHandle1 = vrep.simxGetObjectHandle(clientID, "HeadYaw", vrep.simx_opmode_blocking)
    res, HeadHandle2 = vrep.simxGetObjectHandle(clientID, "HeadPitch", vrep.simx_opmode_blocking)

    # handle the LegJoints
    # Handle = [None] * len(rightHand)

    # for i in range(len(rightArmJoints)):
    #     res, Handle[i] = vrep.simxGetObjectHandle(clientID, rightArmJoints[i], vrep.simx_opmode_blocking)

    # for i in range(3):
    #     #vrep.simxSetJointTargetVelocity(clientID, HeadHandle1, 0.1, vrep.simx_opmode_oneshot)
    #     vrep.simxSetJointTargetPosition(clientID, HeadHandle1, 0.8, vrep.simx_opmode_oneshot_wait)
    #     time.sleep(0.8)
    #     vrep.simxSetJointTargetPosition(clientID, HeadHandle1, -0.8, vrep.simx_opmode_oneshot_wait)
    #     time.sleep(0.8)
    # vrep.simxSetJointTargetPosition(clientID, HeadHandle1, 0, vrep.simx_opmode_oneshot_wait)

    # for i in range(3):
    #     #vrep.simxSetJointTargetVelocity(clientID, HeadHandle1, 0.1, vrep.simx_opmode_oneshot)
    #     vrep.simxSetJointTargetPosition(clientID, HeadHandle2, 0.4, vrep.simx_opmode_oneshot)
    #     time.sleep(0.8)
    #     vrep.simxSetJointTargetPosition(clientID, HeadHandle2, 0, vrep.simx_opmode_oneshot)
    #     time.sleep(0.8)
    # #vrep.simxSetJointTargetPosition(clientID, HeadHandle2, 0, vrep.simx_opmode_oneshot)

    # for i in range(3):
    #     vrep.simxSetJointTargetPosition(clientID, Handle[2], 0.8, vrep.simx_opmode_oneshot)
    #     vrep.simxSetJointTargetPosition(clientID, Handle[3], 0.8, vrep.simx_opmode_oneshot)
    #     time.sleep(0.8)
    #     vrep.simxSetJointTargetPosition(clientID, Handle[2], -0.8, vrep.simx_opmode_oneshot)
    #     vrep.simxSetJointTargetPosition(clientID, Handle[3], -0.8, vrep.simx_opmode_oneshot)
    #     time.sleep(0.8)


    # res, HeadYawHandle = vrep.simxGetObjectHandle(clientID, "HeadYaw", vrep.simx_opmode_blocking)
    # res, LShoulderPitchHandle = vrep.simxGetObjectHandle(clientID, "LShoulderPitch3", vrep.simx_opmode_blocking)

    # all joints are handled
    """
    Handle = [0] * len(allJoints)
    res, Handle[0] = vrep.simxGetObjectHandle(clientID, allJoints[0],vrep.simx_opmode_blocking)
    res, joint = vrep.simxGetJointPosition(clientID, Handle[0], vrep.simx_opmode_streaming)
    print("Joint:", allJoints[0])
    print("joint angle:", joint)
    for i in range(1, len(allJoints)):
        res, Handle[i] = vrep.simxGetObjectHandle(clientID, allJoints[i], vrep.simx_opmode_blocking)
        res, joint = vrep.simxGetJointPosition(clientID, Handle[i], vrep.simx_opmode_buffer)
        print("Joint:", allJoints[i])
        print("joint angle:", joint)
    """
    #####################################################################################

    # test for Get the positions of the joints
    """
    for i in range(10):
        res, joint = vrep.simxGetJointPosition(clientID, LShoulderPitchHandle, vrep.simx_opmode_blocking)
        print(res)
        print("origin:", joint)
   # vrep.simxPauseSimulation(clientID,True)
    vrep.simxSetJointTargetPosition(clientID, LShoulderPitchHandle, 2, vrep.simx_opmode_streaming)
    time.sleep(1)
    #vrep.simxSetJointTargetVelocity(clientID, LShoulderPitchHandle, 2, vrep.simx_opmode_streaming)
    #time.sleep(1)
    vrep.simxSetJointTargetPosition(clientID, HeadYawHandle, -2, vrep.simx_opmode_streaming)
    time.sleep(2)
  #  vrep.simxPauseSimulation(clientID,False)
    res, joint = vrep.simxGetJointPosition(clientID, LShoulderPitchHandle, vrep.simx_opmode_buffer)
    print("Joint:LShoulderPitch ", joint)
    # time.sleep(0.3)
    # res, joint = vrep.simxGetJointPosition(clientID, LShoulderPitchHandle, vrep.simx_opmode_buffer)
    # print("Joint:LShoulderPitch ", joint)
 """
    ###############################################################################################
    # create environment instance

    for i in range(1):
        res, handle = vrep.simxGetObjectHandle(clientID, 'LShoulderRoll3', vrep.simx_opmode_blocking)
        res, headhandle = vrep.simxGetObjectHandle(clientID, 'HeadPitch_link_respondable', vrep.simx_opmode_blocking)
        
        print(vrep.simxGetObjectOrientation(clientID,handle,-1,vrep.simx_opmode_blocking))
        # print(vrep.simxGetObjectPosition(clientID,handle,-1,vrep.simx_opmode_blocking))
        # print(vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking))
        vrep.simxSetObjectOrientation(clientID, handle, -1 ,(360,360,360) , vrep.simx_opmode_oneshot)
        # print("Joint:", allJoints[i])
        # print("joint angle:", joint)

    vrep.simxGetPingTime(clientID)
    # Now close the connection to V-REP:
    print(clientID)
    vrep.simxFinish(clientID)






else:
    print('Failed connecting to remote API server')
print('Program ended')
