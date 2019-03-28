import vrep
import numpy as np
import random
import time
from environment import vrep_env
from reinforcement import reinforcement

ip = '127.0.0.1'
port = 19997
time_step = 0.01
motor_names = ['LeftMotor', 'RightMotor']
object_names = ['Bola']
robot_names = ['DifferentialDriveRobot']
goal_pos = [0.73, 0, 0.0725]
goal_limits = [0.174, -0.174]
epsilon = 0.8

NUM_ACT = 10 #discretization of actions per motor

print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,-500000,5)
if clientID != -1:
    print('Connected to remote API server')
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    emptyBuff = bytearray()


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

    # handle the NAO object and plant object
    res, NAOHandle = vrep.simxGetObjectHandle(clientID, "NAO", vrep.simx_opmode_blocking)
    res, indoorPlantHandle = vrep.simxGetObjectHandle(clientID, "indoorPlant", vrep.simx_opmode_blocking)
    res, HeadHandle1 = vrep.simxGetObjectHandle(clientID, "HeadYaw", vrep.simx_opmode_blocking)
    res, HeadHandle2 = vrep.simxGetObjectHandle(clientID, "HeadPitch", vrep.simx_opmode_blocking)

    # handle the LegJoints
    Handle = [None] * len(rightHand)

    for i in range(len(rightArmJoints)):
        res, Handle[i] = vrep.simxGetObjectHandle(clientID, rightArmJoints[i], vrep.simx_opmode_blocking)

    for i in range(3):
        #vrep.simxSetJointTargetVelocity(clientID, HeadHandle1, 0.1, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, HeadHandle1, 0.8, vrep.simx_opmode_oneshot_wait)
        time.sleep(0.8)
        vrep.simxSetJointTargetPosition(clientID, HeadHandle1, -0.8, vrep.simx_opmode_oneshot_wait)
        time.sleep(0.8)
    vrep.simxSetJointTargetPosition(clientID, HeadHandle1, 0, vrep.simx_opmode_oneshot_wait)

    for i in range(3):
        #vrep.simxSetJointTargetVelocity(clientID, HeadHandle1, 0.1, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, HeadHandle2, 0.4, vrep.simx_opmode_oneshot)
        time.sleep(0.8)
        vrep.simxSetJointTargetPosition(clientID, HeadHandle2, 0, vrep.simx_opmode_oneshot)
        time.sleep(0.8)
    #vrep.simxSetJointTargetPosition(clientID, HeadHandle2, 0, vrep.simx_opmode_oneshot)

    for i in range(3):
        vrep.simxSetJointTargetPosition(clientID, Handle[2], 0.8, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, Handle[3], 0.8, vrep.simx_opmode_oneshot)
        time.sleep(0.8)
        vrep.simxSetJointTargetPosition(clientID, Handle[2], -0.8, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, Handle[3], -0.8, vrep.simx_opmode_oneshot)
        time.sleep(0.8)

    vrep.simxGetPingTime(clientID)
    # Now close the connection to V-REP:
    print(clientID)
    vrep.simxFinish(clientID)






else:
    print('Failed connecting to remote API server')
print('Program ended')


def get_reward(state_info):
    reward = 0
    robot_pos = np.array(state_info[robot_names[0]][0])
    target_pos = np.array(state_info[object_names[0]][0])
    distance = np.linalg.norm(robot_pos - target_pos)
    reward = 1/ distance if distance != 0 else 1000
    return reward

def train(env, model):
    actions = select_action()


def main():
    env = vrep_env(ip, port, time_step, motor_names, robot_names, object_names)
    reinforce = reinforcement(robot_names, object_names, goal_pos, goal_limits)
    model = None
    # --------- Stop Sim ----
    env.startSimulation()
    env.stop_robot(motor_names)
    env.stopSimulation()
    time.sleep(.05)

    # --------- Dummy Sim ---
    num_simulations = 0
    while(num_simulations < 2):
        env.startSimulation()
        dt = 0
        while (dt < 1):
            env.setJointVelocity(motor_names, [0,0])
            state_info = env.getSimulationState()
            reinforce.updateInfo(state_info)
            print(reinforce.getReward())
            dt += time_step
        env.stop_robot(motor_names)
        env.stopSimulation()
        time.sleep(.05)
        num_simulations += 1

    # --------- Train -------
    #print("train")
    #train(env, model)

    # --------- Test --------
    #print("test")
    #test(clientID, motor_handles, target_handle, ddr_handle, dt, model)

    # close any open connections
    env.finishSimulation()


if __name__ == "__main__":
    main()

