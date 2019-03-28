import numpy as np
import random
import argparse
from keras.models import model_from_json, Model
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.optimizers import Adam
import tensorflow as tf
from keras.engine.training import collect_trainable_weights
import json

import vrep
# import sys
import time
import copy

from ReplayBuffer import ReplayBuffer
from ActorNetwork import ActorNetwork
from CriticNetwork import CriticNetwork
# from OU import OU
import timeit
import time

# from sklearn.preprocessing import StandardScaler
# from sklearn.preprocessing import MinMaxScaler

# OU = OU()       #Ornstein-Uhlenbeck Process

clientID=vrep.simxStart('127.0.0.1',19997,True,True,-500000,5)


def getState():

    #height
    res, handle = vrep.simxGetObjectHandle(clientID, 'LHipYawPitch3', vrep.simx_opmode_blocking)
    res, pos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    
    height = pos[2]
    print("initial height:", height)

    res, handlhead = vrep.simxGetObjectHandle(clientID, "HeadPitch_link_respondable", vrep.simx_opmode_blocking)

    #torso
    res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_20_sub0", vrep.simx_opmode_blocking)
    res, Angle = vrep.simxGetObjectOrientation(clientID, handle, handlhead, vrep.simx_opmode_blocking)
    res, Torsolv, Torsoav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    res, Torsopos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    TorsoAngle = Angle[2]

    #Rthigh
    res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_13_sub", vrep.simx_opmode_blocking)
    res, Angle = vrep.simxGetObjectOrientation(clientID, handle, handlhead, vrep.simx_opmode_blocking)
    res, Rthighlv, Rthighav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    res, Rthighpos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    RthighAngle = Angle[2]

    #Lthigh
    res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_10_sub", vrep.simx_opmode_blocking)
    res, Angle = vrep.simxGetObjectOrientation(clientID, handle, handlhead, vrep.simx_opmode_blocking)
    res, Lthighlv, Lthighav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    res, Lthighpos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    LthighAngle = Angle[2]

    #Rcalf
    res, handle = vrep.simxGetObjectHandle(clientID, "r_knee_pitch_link_pure2", vrep.simx_opmode_blocking)
    res, Angle = vrep.simxGetObjectOrientation(clientID, handle, handlhead, vrep.simx_opmode_blocking)
    res, Rcalflv, Rcalfav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    res, Rcalfpos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    RcalfAngle = Angle[2]

    #Lcalf
    res, handle = vrep.simxGetObjectHandle(clientID, "l_knee_pitch_link_pure2", vrep.simx_opmode_blocking)
    res, Angle = vrep.simxGetObjectOrientation(clientID, handle, handlhead, vrep.simx_opmode_blocking)
    res, Lcalflv, Lcalfav = vrep.simxGetObjectVelocity(clientID, handle, vrep.simx_opmode_blocking)
    res, Lcalfpos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    LcalfAngle = Angle[2]

    state = [height]

    state.append(TorsoAngle)
    state.extend(Torsolv)
    state.extend(Torsoav)
    state.extend(Torsopos)

    state.append(RthighAngle)
    state.extend(Rthighlv)
    state.extend(Rthighav)
    state.extend(Rthighpos)

    state.append(LthighAngle)
    state.extend(Lthighlv)
    state.extend(Lthighav)
    state.extend(Lthighpos)

    state.append(RcalfAngle)
    state.extend(Rcalflv)
    state.extend(Rcalfav)
    state.extend(Rcalfpos)

    state.append(LcalfAngle)
    state.extend(Lcalflv)
    state.extend(Lcalfav)
    state.extend(Lcalfpos)

    state = np.array(state)

    return state


def getStep(inistate, action):
    res, handlhead = vrep.simxGetObjectHandle(clientID, "HeadPitch_link_respondable", vrep.simx_opmode_blocking)

    # res, handle = vrep.simxGetObjectHandle(clientID, 'LHipYawPitch3', vrep.simx_opmode_blocking)
    # res, pos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
    iniheight = inistate[0]
    iniTorsoheight = inistate[10]
    iniRthighheight = inistate[20]
    iniLthighheight = inistate[30]
    iniRcalfheight = inistate[40]
    iniLcalfheight = inistate[50]

    print("-------------------getStep------------------------")
    s = getState()
    height = s[0]

    #torso
    # res, handle = vrep.simxGetObjectHandle(clientID, "imported_part_20_sub0", vrep.simx_opmode_blocking)
    # vrep.simxSetObjectOrientation(clientID, handle, )

    #Hip
    res, handle = vrep.simxGetObjectHandle(clientID, "RHipPitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, action[0], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, handle, action[1], vrep.simx_opmode_oneshot)

    res, handle = vrep.simxGetObjectHandle(clientID, "LHipPitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, action[2], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, handle, action[3], vrep.simx_opmode_oneshot)

    #Knee
    res, handle = vrep.simxGetObjectHandle(clientID, "LKneePitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, action[4], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, handle, action[5], vrep.simx_opmode_oneshot)

    res, handle = vrep.simxGetObjectHandle(clientID, "RKneePitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, action[6], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, handle, action[7], vrep.simx_opmode_oneshot)

    #Ankle
    res, handle = vrep.simxGetObjectHandle(clientID, "RAnklePitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, action[8], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, handle, action[9], vrep.simx_opmode_oneshot)

    res, handle = vrep.simxGetObjectHandle(clientID, "LAnklePitch3", vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, handle, action[10], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, handle, action[11], vrep.simx_opmode_oneshot)
    time.sleep(1)

    state = getState()

    height1 = state[0]
    Torsoheight = state[10]
    Rthighheight = state[20]
    Lthighheight = state[30]
    Rcalfheight = state[40]
    Lcalfheight = state[50]

    epsilon = 1e-5
    alpha = -np.log(epsilon)/(np.pi**2/4)
    ex = np.sin(np.pi/4)*iniheight
    ez = (1-np.cos(np.pi/4))*iniheight
    ez_torso = (1-np.cos(np.pi/4))*iniTorsoheight
    ez_Rthigh = (1-np.cos(np.pi/4))*iniRthighheight
    ez_Lthigh = (1-np.cos(np.pi/4))*iniLthighheight
    ez_Rcalf = (1-np.cos(np.pi/4))*iniRcalfheight
    ez_Lcalf = (1-np.cos(np.pi/4))*iniLcalfheight

    alphax = -np.log(epsilon)/(ex**2)
    alphaz = -np.log(epsilon)/(ez**2)
    alphaz_torso = -np.log(epsilon)/(ez_torso**2)
    alphaz_Rthigh = -np.log(epsilon)/(ez_Rthigh**2)
    alphaz_Lthigh = -np.log(epsilon)/(ez_Lthigh**2)
    alphaz_Rcalf = -np.log(epsilon)/(ez_Rcalf**2)
    alphaz_Lcalf = -np.log(epsilon)/(ez_Lcalf**2)

    eanglex = -np.sin(np.pi/4)*iniheight*np.sqrt(9.8/(np.cos(np.pi/4)*iniheight))-np.cos(np.pi/4)*np.sqrt(2*9.8*(1-np.cos(np.pi/4))*iniheight)
    eanglez = -np.sin(np.pi/4)*np.sqrt(2*9.8*(1-np.cos(np.pi/4))*iniheight)


    alpha_anglex = -np.log(epsilon)/(eanglex**2)
    alpha_anglez = -np.log(epsilon)/(eanglez**2)

    V_Torso = np.sqrt(9.8/Torsoheight)*(-iniheight*np.sin(np.pi/4))
    V_Rthigh = np.sqrt(9.8/Rthighheight)*(-iniheight*np.sin(np.pi/4))
    V_Lthigh = np.sqrt(9.8/Lthighheight)*(-iniheight*np.sin(np.pi/4))
    V_Rcalf = np.sqrt(9.8/Rcalfheight)*(-iniheight*np.sin(np.pi/4))
    V_Lcalf = np.sqrt(9.8/Lcalfheight)*(-iniheight*np.sin(np.pi/4))

    
    r_torso = np.exp(-alpha*state[1]**2)

    # alpha_Rthigh = -np.ln(np.log(epsilon))/(np.pi**2/4)
    r_Rthigh = np.exp(-alpha*state[11]**2)
    r_Lthigh = np.exp(-alpha*state[21]**2)
    r_Rcalf = np.exp(-alpha*state[31]**2)
    r_Lcalf = np.exp(-alpha*state[41]**2)

    r_torso_anglex = np.exp(-alpha_anglex*((V_Torso-state[5])**2))
    r_Rthigh_anglex = np.exp(-alpha_anglex*((V_Rthigh-state[15])**2))
    r_Lthigh_anglex = np.exp(-alpha_anglex*((V_Lthigh-state[25])**2))
    r_Rcalf_anglex = np.exp(-alpha_anglex*((V_Rcalf-state[35])**2))
    r_Lcalf_anglex = np.exp(-alpha_anglex*((V_Lcalf-state[45])**2))

    # r_torso_angley = np.exp(-alpha_anglex*((V_Torso-state[6])**2))
    # r_Rthigh_angley = np.exp(-alpha_anglex*((V_Rthigh-state[16])**2))
    # r_Lthigh_angley = np.exp(-alpha_anglex*((V_Lthigh-state[26])**2))
    # r_Rcalf_angley = np.exp(-alpha_anglex*((V_Rcalf-state[36])**2))
    # r_Lcalf_angley = np.exp(-alpha_anglex*((V_Lcalf-state[46])**2))

    r_torso_anglez = np.exp(-alpha_anglez*((-state[7])**2))
    r_Rthigh_anglez = np.exp(-alpha_anglez*((-state[17])**2))
    r_Lthigh_anglez = np.exp(-alpha_anglez*((-state[27])**2))
    r_Rcalf_anglez = np.exp(-alpha_anglez*((-state[37])**2))
    r_Lcalf_anglez = np.exp(-alpha_anglez*((-state[47])**2))

    #position
    x_cap = state[8] + state[2] * np.sqrt(state[10]/9.8)
    x_cap_Rtigh = state[18] + state[12]*np.sqrt(state[20]/9.8)
    x_cap_Ltigh = state[28] + state[22]*np.sqrt(state[30]/9.8)
    x_cap_Rcalf = state[38] + state[32]*np.sqrt(state[40]/9.8)
    x_cap_Lcalf = state[48] + state[42]*np.sqrt(state[50]/9.8)

    # r_pos_x = np.exp(-alphax*(s[8]-0.1-state[8])**2)
    r_pos_x = np.exp(-alphax*(x_cap - state[8])**2)
    r_pos_Rtigh = np.exp(-alphax*(x_cap_Rtigh - state[18])**2)
    r_pos_Ltigh = np.exp(-alphax*(x_cap_Ltigh - state[28])**2)
    r_pos_Rcalf = np.exp(-alphax*(x_cap_Rtigh - state[38])**2)
    r_pos_Lcalf = np.exp(-alphax*(x_cap_Ltigh - state[48])**2)


    r_pos_z = np.exp(-alphaz*(iniheight-height1)**2)
    r_pos_z_torso = np.exp(-alphaz_torso*(iniTorsoheight-Torsoheight)**2)
    r_pos_z_Rthigh = np.exp(-alphaz_Rthigh*(iniRthighheight-Rthighheight)**2)
    r_pos_z_Lthigh = np.exp(-alphaz_Lthigh*(iniLthighheight-Lthighheight)**2)
    r_pos_z_Rcalf = np.exp(-alphaz_Rcalf*(iniRcalfheight-Rcalfheight)**2)
    r_pos_z_Lcalf = np.exp(-alphaz_Lcalf*(iniLcalfheight-Lcalfheight)**2)


    reward = r_torso + r_Rthigh + r_Lthigh + r_Rcalf + r_Lcalf + r_torso_anglex + r_Rthigh_anglex + r_Lthigh_anglex + r_Rcalf_anglex + r_Lcalf_anglex + r_torso_anglez + r_Rthigh_anglez + r_Lthigh_anglez + r_Rcalf_anglez + r_Lcalf_anglez + 5*(r_pos_z + r_pos_z_torso + r_pos_z_Rthigh + r_pos_z_Lthigh + r_pos_z_Rcalf + r_pos_z_Lcalf) + r_pos_x + 10*np.abs(r_pos_Rcalf-r_pos_Lcalf)+10*np.abs(r_pos_Rtigh-r_pos_Ltigh)
    # reward = 0
    return state, reward


def playGame(train_indicator=0):    #1 means Train, 0 means simply Run
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
    print("sleep............")
    time.sleep(1)
    print("sleep finish..............")

    BUFFER_SIZE = 100000
    BATCH_SIZE = 32
    GAMMA = 0.99
    TAU = 0.001     #Target Network HyperParameters
    LRA = 0.0001    #Learning rate for Actor
    LRC = 0.001     #Lerning rate for Critic

    action_dim = 12  #Steering/Acceleration/Brake

    # the selected state features, it includes pelvis height (hheight ), 
    # joint angle and joint velocity, the angle (torso ,pelvis) and angular 
    # velocity(torso, pelvis) of the pelvis and torso, ground contact information, 
    # displacement of COM of links with reference to (w.r.t) the pelvis (red) and the linear velocities of all body links (green).

    state_dim = 51  #of sensors input

    np.random.seed(1337)

    vision = False

    EXPLORE = 100000.
    episode_count = 2000
    max_steps = 100000
    reward = 0
    done = False
    step = 0
    epsilon = 1
    indicator = 0
    ismodel = False

    #Tensorflow GPU optimization
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    sess = tf.Session(config=config)
    from keras import backend as K
    K.set_session(sess)

    actor = ActorNetwork(sess, state_dim, action_dim, BATCH_SIZE, TAU, LRA)
    critic = CriticNetwork(sess, state_dim, action_dim, BATCH_SIZE, TAU, LRC)
    buff = ReplayBuffer(BUFFER_SIZE)    #Create replay buffer

    #Now load the weight
    print("Now we load the weight")
    try:
        actor.model.load_weights("model2/actormodel.h5")
        critic.model.load_weights("model2/criticmodel.h5")
        actor.target_model.load_weights("model2/actormodel.h5")
        critic.target_model.load_weights("model2/criticmodel.h5")
        ismodel = True
        print("Weight load successfully")
    except:
        print("Cannot find the weight")

    print("Learning to Walk Experiment Start.")
    # vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    # print("Come!!!!!!!!!!")

    # action = [-0.6, 0, -0.6, 0, 1, 0, 1, 0, -0.4, 0, -0.4, 0]
    # ini_state, r_t = getStep(getState() ,action)
    # # vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
    # time.sleep(1)
    # print("END!!!!!!!")
    # exit(0)

    # ini_state = getState()
    for i in range(episode_count):
        # vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
        # vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
        vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
        print("QUXI!!!!!!!")
        action = [-0.6, 0, -0.6, 0, 1, 0, 1, 0, -0.4, 0, -0.4, 0]
        ini_state, r_t = getStep(getState() ,action)
        # vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
        time.sleep(1)


        print("Episode : " + str(i) + " Replay Buffer " + str(buff.count()))

        # if np.mod(i, 3) == 0:
        #     ob = env.reset(relaunch=True)   #relaunch TORCS every 3 episode because of the memory leak error
        # else:
        #     ob = env.reset()

        # s_t = np.hstack((ob.angle, ob.track, ob.trackPos, ob.speedX, ob.speedY,  ob.speedZ, ob.wheelSpinVel/100.0, ob.rpm))
        
        s_t = ini_state

        total_reward = 0.
        grads = 1
        # vrep.simxFinish(-1)
        # vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

        for j in range(max_steps):
            # vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
            loss = 0 
            epsilon -= 1.0 / EXPLORE
            # a_t = np.zeros([1,action_dim])
            # noise_t = np.zeros([1,action_dim])
            if (i==0) and (ismodel == False):
                a_t = np.array([0.05, -0.05, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0])
                a_t+=action
            else:    
                a_t = actor.model.predict(s_t.reshape(1, s_t.shape[0])).flatten()
                # if action[4]>0:
                #     action[4] = min(action[4],0.3)
                # else:
                #     action[4] = max(action[4], -0.3)
                # if action[6]>0:
                #     action[6] = min(action[6],0.3)
                # else:
                #     action[6] = max(action[6],-0.3)
            
                # if action[8]>0:
                #     action[8] = min(action[8],0.04)
                # else:
                #     action[8] = max(action[8], -0.04)
                # if action[10]>0:
                #     action[10] = min(action[10],0.04)
                # else:
                #     action[10] = max(action[10],-0.04)
                a_t+=action

            # noise_t[0][0] = train_indicator * max(epsilon, 0) * OU.function(a_t_original[0][0],  0.0 , 0.60, 0.30)
            # noise_t[0][1] = train_indicator * max(epsilon, 0) * OU.function(a_t_original[0][1],  0.5 , 1.00, 0.10)
            # noise_t[0][2] = train_indicator * max(epsilon, 0) * OU.function(a_t_original[0][2], -0.1 , 1.00, 0.05)

            #The following code do the stochastic brake
            #if random.random() <= 0.1:
            #    print("********Now we apply the brake***********")
            #    noise_t[0][2] = train_indicator * max(epsilon, 0) * OU.function(a_t_original[0][2],  0.2 , 1.00, 0.10)



            # a_t[0][0] = a_t_original[0][0] + noise_t[0][0]
            # a_t[0][1] = a_t_original[0][1] + noise_t[0][1]
            # a_t[0][2] = a_t_original[0][2] + noise_t[0][2]

            s_t1, r_t = getStep(ini_state, a_t)
    
            # ob, r_t, done, info = env.step(a_t[0])

            # s_t1 = np.hstack((ob.angle, ob.track, ob.trackPos, ob.speedX, ob.speedY, ob.speedZ, ob.wheelSpinVel/100.0, ob.rpm))
        
            buff.add(s_t, a_t, r_t, s_t1)      #Add replay buffer
            

            #Do the batch update
            batch = buff.getBatch(BATCH_SIZE)
            states = np.asarray([e[0] for e in batch])
            actions = np.asarray([e[1] for e in batch])
            rewards = np.asarray([e[2] for e in batch])
            new_states = np.asarray([e[3] for e in batch])
            # dones = np.asarray([e[4] for e in batch])
            y_t = np.asarray([e[1] for e in batch])

            target_q_values = critic.target_model.predict([new_states, actor.target_model.predict(new_states)]) 

            for k in range(len(batch)):
                y_t[k] = rewards[k] + GAMMA*target_q_values[k]
       
            if (train_indicator):
                loss += critic.model.train_on_batch([states,actions], y_t) 
                a_for_grad = actor.model.predict(states)
                # print("a_for_grad:",a_for_grad)
                compute_a = a_for_grad.flatten()
                grads = critic.gradients(states, a_for_grad)
                p_max = np.max(grads[grads.shape[0]-1])
                p_min =np.min(grads[grads.shape[0]-1])
                for k in range(action_dim):
                    if grads[grads.shape[0]-1][k] >= 0:
                        grads[grads.shape[0]-1][k] *= ((0.5-compute_a[k])/0.5)
                    else:
                        grads[grads.shape[0]-1][k] *= ((compute_a[k]-0)/0.5)

                actor.train(states, grads)
                actor.target_train()
                critic.target_train()

            print("Episode", i, "Step", step, "Action", a_t, "Reward", r_t, "Loss", loss)

            total_reward += r_t
            s_t = s_t1
        
        
            step += 1

            if (s_t1[0]<0.15):
                break
            # if done:
            #     break

        if np.mod(i, 3) == 0:
            if (train_indicator):
                print("Now we save model")
                actor.model.save_weights("model2/actormodel.h5", overwrite=True)
                with open("actormodel.json", "w") as outfile:
                    json.dump(actor.model.to_json(), outfile)

                critic.model.save_weights("model2/criticmodel.h5", overwrite=True)
                with open("criticmodel.json", "w") as outfile:
                    json.dump(critic.model.to_json(), outfile)

        print("Finish-----------------------------------------------------")
        vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
        time.sleep(1)
        # vrep.simxFinish(clientID)


        print("TOTAL REWARD @ " + str(i) +"-th Episode  : Reward " + str(total_reward))
        print("Total Step: " + str(step))
        print("")

    # env.end()  # This is for shutting down TORCS
    vrep.simxFinish(clientID)

    print("Finish.")

def main():
    print('Program started')
    # clientID=vrep.simxStart('127.0.0.1',19997,True,True,-500000,5)
    if clientID != -1:
        # print('Connected to remote API server')
        # vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
        print("training Model...........")
        playGame(1)
        print("predict Model...........")
        playGame(0)
        # vrep.simxGetPingTime(clientID)
        # Now close the connection to V-REP:
        print(clientID)
        # vrep.simxFinish(clientID)


    else:
        print('Failed connecting to remote API server')
    print('Program ended')

main()
# vrep.simxFinish(-1)  # just in case, close all opened connections
# clientID=vrep.simxStart('127.0.0.1',19997,True,True,-500000,5)
# if clientID != -1:
#     print('Connected to remote API server')
#     vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
#     playGame(1, clientID)
#     playGame(0)
#     vrep.simxGetPingTime(clientID)
#     # Now close the connection to V-REP:
#     print(clientID)
#     vrep.simxFinish(clientID)


# else:
#     print('Failed connecting to remote API server')
# print('Program ended')
