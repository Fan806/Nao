# This is edited for Nao robot to walk normally

import vrep
# import sys
import time
import copy
import numpy as np

# from ddpg import playGame

def getState():
    #height
    res, handle = vrep.simxGetObjectHandle(clientID, 'LHipYawPitch3', vrep.simx_opmode_blocking)
    print(handle)
    res, pos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    height = pos[2]

    #torso
    res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_20_sub0", vrep.simx_opmode_blocking)
    print(handle)
    res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    res, Torsolv, Torsoav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    TorsoAngle = Angle

    #Rthigh
    res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_13_sub", vrep.simx_opmode_blocking)
    res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    res, Rthighlv, Rthighav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    RthighAngle = Angle

    #Lthigh
    res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_10_sub", vrep.simx_opmode_blocking)
    res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    res, Lthighlv, Lthighav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    LthighAngle = Angle

    #Rcalf
    res, handle = vrep.simxGetObjectHandle(clientID, "r_knee_pitch_link_pure2", vrep.simx_opmode_blocking)
    res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    res, Rcalflv, Rcalfav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    RcalfAngle = Angle

    #Lcalf
    res, handle = vrep.simxGetObjectHandle(clientID, "l_knee_pitch_link_pure2", vrep.simx_opmode_blocking)
    res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    res, Lcalflv, Lcalfav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    LcalfAngle = Angle

    state = [height]

    state.extend(TorsoAngle)
    state.extend(Torsolv)
    state.extend(Torsoav)

    state.extend(RthighAngle)
    state.extend(Rthighlv)
    state.extend(Rthighav)

    state.extend(LthighAngle)
    state.extend(Lthighlv)
    state.extend(Lthighav)

    state.extend(RcalfAngle)
    state.extend(Rcalflv)
    state.extend(Rcalfav)

    state.extend(LcalfAngle)
    state.extend(Lcalflv)
    state.extend(Lcalfav)

    state = np.array(state)
    return state


def step(action):
    #Hip
    res, handle = vrep.simxGetObjectHandle(clientID, "RHipPitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, action[0], vrep.simx_opmode_oneshot)

    res, handle = vrep.simxGetObjectHandle(clientID, "LHipPitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, action[1], vrep.simx_opmode_oneshot)

    #Knee
    res, handle = vrep.simxGetObjectHandle(clientID, "LKneePitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, action[2], vrep.simx_opmode_oneshot)

    res, handle = vrep.simxGetObjectHandle(clientID, "RKneePitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, action[3], vrep.simx_opmode_oneshot)

    #Ankle
    res, handle = vrep.simxGetObjectHandle(clientID, "RAnkePitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, action[4], vrep.simx_opmode_oneshot)

    res, handle = vrep.simxGetObjectHandle(clientID, "LAnkePitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, action[5], vrep.simx_opmode_oneshot)


    # State

    # res, handle = vrep.simxGetObjectHandle(clientID, 'LHipYawPitch3', vrep.simx_opmode_blocking)
    # res, pos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    # height = pos[2]

    # #torso
    # res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_20_sub0", vrep.simx_opmode_blocking)
    # res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    # res, Torsolv, Torsoav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    # res, Torsopos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    # TorsoAngle = Angle

    # #Rthigh
    # res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_13_sub", vrep.simx_opmode_blocking)
    # res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    # res, Rthighlv, Rthighav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    # res, Rthighpos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    # RthighAngle = Angle

    # #Lthigh
    # res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_10_sub", vrep.simx_opmode_blocking)
    # res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    # res, Lthighlv, Lthighav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    # res, Lthighpos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    # LthighAngle = Angle

    # #Rcalf
    # res, handle = vrep.simxGetObjectHandle(clientID, "r_knee_pitch_link_pure2", vrep.simx_opmode_blocking)
    # res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    # res, Rcalflv, Rcalfav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    # res, Rcalfpos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    # RcalfAngle = Angle

    # #Lcalf
    # res, handle = vrep.simxGetObjectHandle(clientID, "l_knee_pitch_link_pure2", vrep.simx_opmode_blocking)
    # res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    # res, Lcalflv, Lcalfav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    # res, Lcalfpos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    # LcalfAngle = Angle

    # state = [height]

    # state.extend(TorsoAngle)
    # state.extend(Torsolv)
    # state.extend(Torsoav)

    # state.extend(RthighAngle)
    # state.extend(Rthighlv)
    # state.extend(Rthighav)

    # state.extend(LthighAngle)
    # state.extend(Lthighlv)
    # state.extend(Lthighav)

    # state.extend(RcalfAngle)
    # state.extend(Rcalflv)
    # state.extend(Rcalfav)

    # state.extend(LcalfAngle)
    # state.extend(Lcalflv)
    # state.extend(Lcalfav)

    # state = np.array(state)

    state = getState()

    # epsilon = 1e-5
    # alpha = -np.log(epsilon)/(np.pi**2/4)
    # ex = np.sin(np.pi/4)*height
    # ez = (1-np.cos(np.pi/4))*height
    # alphx = -np.log(epsilon)/(ex**2)
    # alphaz = -np.log(epsilon)/(ez**2)


    # eangley = -np.sin(np.pi/4)*height*np.sqrt(9.8/(np.cos(np.pi/4)*height))-np.cos(np.pi/4)*np.sqrt(2*9.8*(1-np.cos(np.pi/4))*height)
    # eanglez = -np.sin(np.pi/4)*np.sqrt(2*9.8*(1-np.cos(np.pi/4))*height)

    # alpha_angley = -np.log(epsilon)/(eangley**2)
    # alpha_anglez = -np.log(epsilon)/(eanglez**2)

    # V_Torso = np.sqrt(9.8/Torsopos[2])*(-height*np.sin(np.pi/4))
    # V_Rthigh = np.sqrt(9.8/Rthighpos[2])*(-height*np.sin(np.pi/4))
    # V_Lthigh = np.sqrt(9.8/Lthighpos[2])*(-height*np.sin(np.pi/4))
    # V_Rcalf = np.sqrt(9.8/Rcalfpos[2])*(-height*np.sin(np.pi/4))
    # V_Lcalf = np.sqrt(9.8/Rthighpos[2])*(-height*np.sin(np.pi/4))

    
    # r_torso = np.exp(-alpha*TorsoAngle**2)

    # # alpha_Rthigh = -np.ln(np.log(epsilon))/(np.pi**2/4)
    # r_Rthigh = np.exp(-alpha*RthighAngle**2)
    # r_Lthigh = np.exp(-alpha*LthighAngle**2)
    # r_Rcalf = np.exp(-alpha*RcalfAngle**2)
    # r_Lcalf = np.exp(-alpha*LcalfAngle**2)

    # r_torso_angley = np.exp(-alpha_angley*((V_Torso-Torsoav[2])**2))
    # r_Rthigh_angley = np.exp(-alpha_angley*((V_Rthigh-Rthighav[2])**2))
    # r_Lthigh_angley = np.exp(-alpha_angley*((V_Lthigh-Lthighav[2])**2))
    # r_Rcalf_angley = np.exp(-alpha_angley*((V_Rcalf-Rcalfav[2])**2))
    # r_Lcalf_angley = np.exp(-alpha_angley*((V_Lcalf-Lcalfav[2])**2))

    # r_torso_anglez = np.exp(-alpha_anglez*((-Torsoav[3])**2))
    # r_Rthigh_anglez = np.exp(-alpha_anglez*((-Rthighav[3])**2))
    # r_Lthigh_anglez = np.exp(-alpha_anglez*((-Lthighav[3])**2))
    # r_Rcalf_anglez = np.exp(-alpha_anglez*((-Rcalfav[3])**2))
    # r_Lcalf_anglez = np.exp(-alpha_anglez*((-Lcalfav[3])**2))

    # reward = r_torso + r_Rthigh + r_Lthigh + r_Rcalf + r_Lcalf + r_torso_angley + r_Rthigh_angley + r_Lthigh_angley + r_Rcalf_angley + r_Lcalf_angley + r_torso_angley + r_Rthigh_anglez + r_Lthigh_anglez + r_Rcalf_anglez + r_Lcalf_anglez

    reward = 0
    return state, reward

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
    leftLegJoints = ['LHipYawPitch3', 'LHipRoll3', 'LHipPitch3', 'LKneePitch3', 'LAnklePitch3', 'LAnkleRoll3']
    rightLegJoints = ['RHipYawPitch3', 'RHipRoll3', 'RHipPitch3', 'RKneePitch3', 'RAnklePitch3', 'RAnkleRoll3']
    leftArmJoints = ['LShoulderPitch3', 'LShoulderRoll3', 'LElbowYaw3', 'LElbowRoll3', 'LWristYaw3']
    leftHand = ['NAO_LThumbBase', 'Revolute_joint8', 'NAO_LLFingerBase', 'Revolute_joint12', 'Revolute_joint14',
                'NAO_LRFingerBase', 'Revolute_joint11', 'Revolute_joint13']
    rightArmJoints = ['RShoulderPitch3', 'RShoulderRoll3', 'RElbowYaw3', 'RElbowRoll3', 'RWristYaw3']
    rightHand = ['NAO_RThumbBase', 'Revolute_joint0', 'NAO_RLFingerBase', 'Revolute_joint5', 'Revolute_joint6',
                 'NAO_RRFingerBase', 'Revolute_joint2', 'Revolute_joint3']

    LegJoints = copy.deepcopy(leftLegJoints)
    LegJoints.extend(rightLegJoints)
    print("LegJoints:", LegJoints)

    # playGame(1)
    # playGame(0)

    # #-----------------------------state--------------------------------

    # #height
    # res, handle = vrep.simxGetObjectHandle(clientID, 'LHipYawPitch3', vrep.simx_opmode_blocking)
    # res, pos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    # height = pos[2]

    # #torso
    # res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_20_sub0", vrep.simx_opmode_blocking)
    # res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    # res, Torsolv, Torsoav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    # TorsoAngle = Angle

    # #Rthigh
    # res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_13_sub", vrep.simx_opmode_blocking)
    # res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    # res, Rthighlv, Rthighav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    # RthighAngle = Angle

    # #Lthigh
    # res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_10_sub", vrep.simx_opmode_blocking)
    # res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    # res, Lthighlv, Lthighav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    # LthighAngle = Angle

    # #Rcalf
    # res, handle = vrep.simxGetObjectHandle(clientID, "r_knee_pitch_link_pure2", vrep.simx_opmode_blocking)
    # res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    # res, Rcalflv, Rcalfav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    # RcalfAngle = Angle

    # #Lcalf
    # res, handle = vrep.simxGetObjectHandle(clientID, "l_knee_pitch_link_pure2", vrep.simx_opmode_blocking)
    # res, Angle = vrep.simxGetObjectOrientation(clientID, handle, -1, vrep.simx_opmode_blocking)
    # res, Lcalflv, Lcalfav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    # LcalfAngle = Angle

    # state = [height]

    # state.extend(TorsoAngle)
    # state.extend(Torsolv)
    # state.extend(Torsoav)

    # state.extend(RthighAngle)
    # state.extend(Rthighlv)
    # state.extend(Rthighav)

    # state.extend(LthighAngle)
    # state.extend(Lthighlv)
    # state.extend(Lthighav)

    # state.extend(RcalfAngle)
    # state.extend(Rcalflv)
    # state.extend(Rcalfav)

    # state.extend(LcalfAngle)
    # state.extend(Lcalflv)
    # state.extend(Lcalfav)

    # state = np.array(state)

    # #-----------------------------action--------------------------------

    # #Hip
    # res, handle = vrep.simxGetObjectHandle(clientID, "RHipRoll3", vrep.simx_opmode_blocking)
    # res, RTorso = vrep.simxGetJointPosition(clientID, handle, vrep.simx_opmode_blocking)
    # res, RHipF = vrep.simxGetJointForce(clientID, handle, vrep.simx_opmode_blocking)

    # res, handle = vrep.simxGetObjectHandle(clientID, "LHipRoll3", vrep.simx_opmode_blocking)
    # res, LTorso = vrep.simxGetJointPosition(clientID, handle, vrep.simx_opmode_blocking)
    # res, LHipF = vrep.simxGetJointForce(clientID, handle, vrep.simx_opmode_blocking)
    # # vrep.simxSetJointTargetPosition(clientID, handle, -100, vrep.simx_opmode_oneshot)

    # #Knee
    # res, handle = vrep.simxGetObjectHandle(clientID, "LKneePitch3", vrep.simx_opmode_blocking)
    # res, RKnee = vrep.simxGetJointPosition(clientID, handle, vrep.simx_opmode_blocking)
    # res, RKneeF = vrep.simxGetJointForce(clientID, handle, vrep.simx_opmode_blocking)

    # res, handle = vrep.simxGetObjectHandle(clientID, "RKneePitch3", vrep.simx_opmode_blocking)
    # res, LKnee = vrep.simxGetJointPosition(clientID, handle, vrep.simx_opmode_blocking)
    # res, LKneeF = vrep.simxGetJointForce(clientID, handle, vrep.simx_opmode_blocking)

    # #Ankle
    # res, handle = vrep.simxGetObjectHandle(clientID, "RAnkeRoll3", vrep.simx_opmode_blocking)
    # res, RAnkle = vrep.simxGetJointPosition(clientID, handle, vrep.simx_opmode_blocking)
    # res, RAnkleF = vrep.simxGetJointForce(clientID, handle, vrep.simx_opmode_blocking)

    # res, handle = vrep.simxGetObjectHandle(clientID, "LAnkeRoll3", vrep.simx_opmode_blocking)
    # res, LAnkle = vrep.simxGetJointPosition(clientID, handle, vrep.simx_opmode_blocking)
    # res, LAnkleF = vrep.simxGetJointForce(clientID, handle, vrep.simx_opmode_blocking)

    # action = np.array([Rtorso, LTorso, RKnee, LKnee, RAnkle, LAnkle])

    getState()
    # step([ 1.0724496 , 1.05645561, -0.73066556, -0.12962897 ,1.6850575,  -0.6771174 ])
    # step([ 0.7724496 , 0.05645561, -0.73066556, -0.12962897 ,1.6850575,  -0.6771174 ])
    # step([ 0.7724496 , 0.05645561, -0.73066556, -0.12962897 ,1.6850575,  -0.6771174 ])
    step([1]*6)
    res, handle = vrep.simxGetObjectHandle(clientID, "RHipPitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, -200, vrep.simx_opmode_oneshot)
    # (clientID, "RHipPitch3", vrep.simx_opmode_blocking)
    # vrep.simxSetJointTargetPosition(clientID, 
    vrep.simxGetPingTime(clientID)
    # Now close the connection to V-REP:
    print(clientID)
    vrep.simxFinish(clientID)


else:
    print('Failed connecting to remote API server')
print('Program ended')

