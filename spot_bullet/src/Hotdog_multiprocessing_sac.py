#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Hotdog.py
#
#  Created on: 2020. 11. 25.
#  Update(21.00.00) : Change file name, class
#  Update(21.05.04) : Multiprocessing
#  Author    : Choi Chanyeok, Ham Seoyeon
#

from multiprocessing import Process, Queue
import threading
import time
import os, sys
import time
import serial
import math
import dynamixel_sdk as dynamixel
from mpl_toolkits import mplot3d
import numpy as np
from math import *
import matplotlib.pyplot as plt

import argparse

from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.Gyroscope import *
from Phidget22.Devices.Magnetometer import *
from Phidget22.Devices.Spatial import *


from gtts import gTTS
import playsound

current = []
presentposition = []
ADDR_XM = []
emergency_situation = 0

# global ADDR_XM
global Leg_thetaes


# Control table address for Dynamixel XM-W350-R             # Control table address is different in Dynamixel model
ADDR_XM_TORQUE_ENABLE      = 64
ADDR_XM_Pgain      = 84
ADDR_XM_Igain      = 82
ADDR_XM_Dgain      = 80
ADDR_XM_FF2ND      = 88
ADDR_XM_FF1ST      = 90
ADDR_XM_GOAL_POSITION      = 116
ADDR_XM_GOAL_CURRENT       = 102
ADDR_XM_PRESENT_POSITION   = 132
ADDR_XM_ProfileACC         = 108
ADDR_XM_ProfileVel         = 112
ADDR_XM_PRESENT_CURRENT    = 126

# Protocol version
PROTOCOL_VERSION1           = 1                             # See which protocol version is used in the Dynamixel
PROTOCOL_VERSION2           = 2

# Default setting
XM_ID = list(range(1,13))
FR1_ID                      = 1
FR2_ID                      = 2
FR3_ID                      = 3
FL4_ID                      = 4
FL5_ID                      = 5
FL6_ID                      = 6
RR7_ID                      = 7
RR8_ID                      = 8
RR9_ID                      = 9
RL10_ID                     = 10
RL11_ID                     = 11
RL12_ID                     = 12
IMU                         = 13


BAUDRATE                    = 3000000
DEVICENAME1                  = "COM17" #ID1,2,3,10,11,12
DEVICENAME2                  = "COM18" #ID4,5,6,7,8,9

#DEVICENAME1                  = "/dev/ttyUSB0".encode('utf-8')        # Check which port is being used on your controller
                                                                     # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque

MINIMUM_POSITION_VALUE       = 2000                         # Dynamixel will rotate between this value
MAXIMUM_POSITION_VALUE       = 2100                         # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
MOVING_STATUS_THRESHOLD      = 20                           # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

"""
#############################################################################################################
#arduino nano ble <-> jetson nano
#############################################################################################################
from bluepy.btle import Peripheral, UUID, BTLEDisconnectError, DefaultDelegate
import time

led_service_uuid = "19B10001-E8F2-537E-4F6C-D104768A1214"
led_char_uuid = "19B10001-E8F2-537E-4F6C-D104768A1214"

class NotificationDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data):
        xyz = data.decode().split('|')
        print("X = ", xyz[0], " Y = ", xyz[1], " Z = ", xyz[2])

# print("Arduino Peripheral")
arduino = Peripheral(deviceAddr="c8:f6:1d:b5:f7:14") #arduino = Peripheral(deviceAddr="d2:f1:79:40:d9:8d")
arduino.setDelegate(NotificationDelegate())

characteristics = arduino.getCharacteristics()
#############################################################################################################
#############################################################################################################
"""
def dynamixel_data(position):
    param_goalposition = [0,0,0,0]
    param_goalposition[0] = dynamixel.DXL_LOBYTE(dynamixel.DXL_LOWORD(position))
    param_goalposition[1] = dynamixel.DXL_HIBYTE(dynamixel.DXL_LOWORD(position))
    param_goalposition[2] = dynamixel.DXL_LOBYTE(dynamixel.DXL_HIWORD(position))
    param_goalposition[3] = dynamixel.DXL_HIBYTE(dynamixel.DXL_HIWORD(position))

    return param_goalposition

def SAC(id, data, result):
    import sys
    sys.path.append('../../')

    import numpy as np

    from sac_lib import SoftActorCritic, NormalizedActions, ReplayBuffer, PolicyNetwork
    import copy
    from gym import spaces

    from spotmicro.GymEnvs.spot_bezier_env import spotBezierEnv
    from spotmicro.Kinematics.SpotKinematics import SpotModel
    from spotmicro.GaitGenerator.Bezier import BezierGait

    # TESTING
    from spotmicro.OpenLoopSM.SpotOL import BezierStepper

    import time

    import torch
    import os

    print("STARTING SPOT SAC")

    # TRAINING PARAMETERS
    seed = 0
    max_timesteps = 4e6
    batch_size = 256
    eval_freq = 1e4
    save_model = True
    file_name = "spot_sac_"

    # Find abs path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    results_path = os.path.join(my_path, "../results")
    models_path = os.path.join(my_path, "../models")

    if not os.path.exists(results_path):
        os.makedirs(results_path)

    if not os.path.exists(models_path):
        os.makedirs(models_path)

    env = spotBezierEnv(render=False,
                        on_rack=False,
                        height_field=False,
                        draw_foot_path=False)
    env = NormalizedActions(env)

    # Set seeds
    env.seed(seed)
    torch.manual_seed(seed)
    np.random.seed(seed)

    state_dim = env.observation_space.shape[0]
    print("STATE DIM: {}".format(state_dim))
    action_dim = env.action_space.shape[0]
    print("ACTION DIM: {}".format(action_dim))
    max_action = float(env.action_space.high[0])

    print("RECORDED MAX ACTION: {}".format(max_action))

    hidden_dim = 256
    policy = PolicyNetwork(state_dim, action_dim, hidden_dim)

    replay_buffer_size = 1000000
    replay_buffer = ReplayBuffer(replay_buffer_size)

    sac = SoftActorCritic(policy=policy,
                          state_dim=state_dim,
                          action_dim=action_dim,
                          replay_buffer=replay_buffer)

    policy_num = 0
    if os.path.exists(models_path + "/" + file_name + str(policy_num) +
                      "_critic"):
        print("Loading Existing Policy")
        sac.load(models_path + "/" + file_name + str(policy_num))
        policy = sac.policy_net

    # Evaluate untrained policy and init list for storage
    evaluations = []

    state = env.reset()
    done = False
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0
    max_t_per_ep = 5000

    # State Machine for Random Controller Commands
    bz_step = BezierStepper(dt=0.01)

    # Bezier Gait Generator
    bzg = BezierGait(dt=0.01)

    # Spot Model
    spot = SpotModel()
    T_bf0 = spot.WorldToFoot
    T_bf = copy.deepcopy(T_bf0)

    BaseClearanceHeight = bz_step.ClearanceHeight
    BasePenetrationDepth = bz_step.PenetrationDepth

    print("STARTED SPOT SAC")

    for t in range(int(max_timesteps)):

        pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth = bz_step.StateMachine(
        )

        env.spot.GetExternalObservations(bzg, bz_step)

        # Read UPDATED state based on controls and phase
        state = env.return_state()

        action = sac.policy_net.get_action(state)

        # Bezier params specced by action_dim
        CD_SCALE = 0.002
        SLV_SCALE = 0.01
        StepLength += action[0] * CD_SCALE
        StepVelocity += action[1] * SLV_SCALE
        LateralFraction += action[2] * SLV_SCALE
        YawRate = action[3]
        ClearanceHeight += action[4] * CD_SCALE
        PenetrationDepth += action[5] * CD_SCALE

        # CLIP EVERYTHING
        StepLength = np.clip(StepLength, bz_step.StepLength_LIMITS[0],
                             bz_step.StepLength_LIMITS[1])
        StepVelocity = np.clip(StepVelocity, bz_step.StepVelocity_LIMITS[0],
                               bz_step.StepVelocity_LIMITS[1])
        LateralFraction = np.clip(LateralFraction,
                                  bz_step.LateralFraction_LIMITS[0],
                                  bz_step.LateralFraction_LIMITS[1])
        YawRate = np.clip(YawRate, bz_step.YawRate_LIMITS[0],
                          bz_step.YawRate_LIMITS[1])
        ClearanceHeight = np.clip(ClearanceHeight,
                                  bz_step.ClearanceHeight_LIMITS[0],
                                  bz_step.ClearanceHeight_LIMITS[1])
        PenetrationDepth = np.clip(PenetrationDepth,
                                   bz_step.PenetrationDepth_LIMITS[0],
                                   bz_step.PenetrationDepth_LIMITS[1])

        contacts = state[-4:]

        # Get Desired Foot Poses
        T_bf = bzg.GenerateTrajectory(StepLength, LateralFraction, YawRate,
                                      StepVelocity, T_bf0, T_bf,
                                      ClearanceHeight, PenetrationDepth,
                                      contacts)
        # Add DELTA to XYZ Foot Poses
        RESIDUALS_SCALE = 0.05
        # T_bf["FL"][3, :3] += action[6:9] * RESIDUALS_SCALE
        # T_bf["FR"][3, :3] += action[9:12] * RESIDUALS_SCALE
        # T_bf["BL"][3, :3] += action[12:15] * RESIDUALS_SCALE
        # T_bf["BR"][3, :3] += action[15:18] * RESIDUALS_SCALE
        T_bf["FL"][3, 2] += action[6] * RESIDUALS_SCALE
        T_bf["FR"][3, 2] += action[7] * RESIDUALS_SCALE
        T_bf["BL"][3, 2] += action[8] * RESIDUALS_SCALE
        T_bf["BR"][3, 2] += action[9] * RESIDUALS_SCALE

        joint_angles = spot.IK(orn, pos, T_bf)
        # Pass Joint Angles
        env.pass_joint_angles(joint_angles.reshape(-1))

        # Perform action
        next_state, reward, done, _ = env.step(action)
        done_bool = float(done)

        episode_timesteps += 1

        # Store data in replay buffer
        replay_buffer.push(state, action, reward, next_state, done_bool)

        state = next_state
        episode_reward += reward

        # Train agent after collecting sufficient data for buffer
        if len(replay_buffer) > batch_size:
            sac.soft_q_update(batch_size)

        if episode_timesteps > max_t_per_ep:
            done = True

        if done:
            # Reshuffle State Machine
            bzg.reset()
            bz_step.reshuffle()
            bz_step.ClearanceHeight = BaseClearanceHeight
            bz_step.PenetrationDepth = BasePenetrationDepth
            # +1 to account for 0 indexing.
            # +0 on ep_timesteps since it will increment +1 even if done=True
            print(
                "Total T: {} Episode Num: {} Episode T: {} Reward: {:.2f} REWARD PER STEP: {:.2f}"
                .format(t + 1, episode_num, episode_timesteps, episode_reward,
                        episode_reward / float(episode_timesteps)))
            # Reset environment
            state, done = env.reset(), False
            evaluations.append(episode_reward)
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1

        # Evaluate episode
        if (t + 1) % eval_freq == 0:
            # evaluate_policy(policy, env_name, seed,
            np.save(results_path + "/" + str(file_name), evaluations)
            if save_model:
                sac.save(models_path + "/" + str(file_name) + str(t))
                # replay_buffer.save(t)

    env.close()
    
def social_distance_detection(id, data, result):
    import numpy as np
    import argparse
    import sys
    import cv2
    from math import pow, sqrt


    # Parse the arguments from command line
    arg = argparse.ArgumentParser(description='Social distance detection')

    arg.add_argument('-v', '--video', type = str, default = '', help = 'Video file path. If no path is given, video is captured using device.')

    arg.add_argument('-m', '--model', required = True, help = "Path to the pretrained model.")

    arg.add_argument('-p', '--prototxt', required = True, help = 'Prototxt of the model.')

    arg.add_argument('-l', '--labels', required = True, help = 'Labels of the dataset.')

    arg.add_argument('-c', '--confidence', type = float, default = 0.2, help='Set confidence for detecting objects')

    args = vars(arg.parse_args())


    labels = [line.strip() for line in open(args['labels'])]

    # Generate random bounding box bounding_box_color for each label
    bounding_box_color = np.random.uniform(0, 255, size=(len(labels), 3))


    # Load model
    print("\nLoading model...\n")
    network = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])

    print("\nStreaming video using device...\n")


    # Capture video from file or through device
    if args['video']:
        cap = cv2.VideoCapture(args['video'])
    else:
        cap = cv2.VideoCapture(2)


    frame_no = 0

    while cap.isOpened():

        frame_no = frame_no+1

        # Capture one frame after another
        ret, frame = cap.read()

        if not ret:
            break

        (h, w) = frame.shape[:2]

        # Resize the frame to suite the model requirements. Resize the frame to 300X300 pixels
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)

        network.setInput(blob)
        detections = network.forward()

        pos_dict = dict()
        coordinates = dict()

        # Focal length
        F = 615

        for i in range(detections.shape[2]):

            confidence = detections[0, 0, i, 2]

            if confidence > args["confidence"]:

                class_id = int(detections[0, 0, i, 1])

                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype('int')

                # Filtering only persons detected in the frame. Class Id of 'person' is 15
                if class_id == 15.00:

                    # Draw bounding box for the object
                    cv2.rectangle(frame, (startX, startY), (endX, endY), bounding_box_color[class_id], 2)

                    label = "{}: {:.2f}%".format(labels[class_id], confidence * 100)
                    print("{}".format(label))


                    coordinates[i] = (startX, startY, endX, endY)

                    # Mid point of bounding box
                    x_mid = round((startX+endX)/2,4)
                    y_mid = round((startY+endY)/2,4)

                    height = round(endY-startY,4)

                    # Distance from camera based on triangle similarity
                    distance = (165 * F)/height
                    print("Distance(cm):{dist}\n".format(dist=distance))

                    # Mid-point of bounding boxes (in cm) based on triangle similarity technique
                    x_mid_cm = (x_mid * distance) / F
                    y_mid_cm = (y_mid * distance) / F
                    pos_dict[i] = (x_mid_cm,y_mid_cm,distance)

        # Distance between every object detected in a frame
        close_objects = set()
        for i in pos_dict.keys():
            for j in pos_dict.keys():
                if i < j:
                    dist = sqrt(pow(pos_dict[i][0]-pos_dict[j][0],2) + pow(pos_dict[i][1]-pos_dict[j][1],2) + pow(pos_dict[i][2]-pos_dict[j][2],2))

                    # Check if distance less than 2 metres or 200 centimetres
                    if dist < 200:
                        close_objects.add(i)
                        close_objects.add(j)

        for i in pos_dict.keys():
            if i in close_objects:
                COLOR = np.array([0,0,255])
            else:
                COLOR = np.array([0,255,0])
            (startX, startY, endX, endY) = coordinates[i]

            cv2.rectangle(frame, (startX, startY), (endX, endY), COLOR, 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            # Convert cms to feet
            cv2.putText(frame, 'Depth: {i} ft'.format(i=round(pos_dict[i][2]/30.48,4)), (startX, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR, 2)

        cv2.namedWindow('Frame',cv2.WINDOW_NORMAL)

        # Show frame
        #cv2.imshow('Frame', frame)
        #cv2.resizeWindow('Frame',800,600)

        key = cv2.waitKey(1) & 0xFF

        # Press `q` to exit
        if key == ord("q"):
            break

    # Clean
    cap.release()
    cv2.destroyAllWindows()

def hotdoghandel(id, data, result):
    from bluepy.btle import Peripheral, UUID, BTLEDisconnectError, DefaultDelegate
    import time

    led_service_uuid = "19B10001-E8F2-537E-4F6C-D104768A1214"
    led_char_uuid = "19B10001-E8F2-537E-4F6C-D104768A1214"

    class NotificationDelegate(DefaultDelegate):
        def __init__(self):
            DefaultDelegate.__init__(self)

        def handleNotification(self, cHandle, data):
            xyz = data.decode().split('|')
            print("X = ", xyz[0], " Y = ", xyz[1], " Z = ", xyz[2])

    print("Arduino Peripheral")
    #arduino = Peripheral(deviceAddr="d2:f1:79:40:d9:8d")
    arduino = Peripheral(deviceAddr="c8:f6:1d:b5:f7:14")
    arduino.setDelegate(NotificationDelegate())
        
    #services = arduino.getServices()
    characteristics = arduino.getCharacteristics()
    # for characteristic in characteristics:
    #     try:
    #         print("Value: ", characteristic.read().decode(), " UUID: ", characteristic.uuid,
    #              " Handle: ", characteristic.handle," ValueHandle: ", characteristic.valHandle)
    #     except:
    #         pass

    #start_notification_data = b"\x01\x00"
    # arduino.writeCharacteristic(13, start_notification_data) will also work, 13 comes from "characteristic handle + 1"
    while True:
        a = [10,20]
        for i in a:
            print(i)
            #arduino.writeCharacteristic(11, bytes([i]))
            arduino.writeCharacteristic(11, str(i))
            time.sleep(5)
        
        

    #i = 0
    # while True:
    #     try:
    #         if i < 256:
    #             arduino.writeCharacteristic(11, bytes([0]))
    #             print(arduino.readCharacteristic(11))
    #             # arduino.writeCharacteristic(11, "A") 
    #             # if arduino.waitForNotifications(0.1):
    #             #     continue
    #             print("Send : ", i)
    #             i = i + 1
    #             time.sleep(1)
    #         else:
    #             i = 0 
    #     except KeyboardInterrupt:
    #         break

       # arduino.disconnect()

def u2d2dynamixel(id, data, result, legposition):
    # import os, sys
    # import time
    # import serial
    # import math
    # import dynamixel_sdk as dynamixel
    # from mpl_toolkits import mplot3d
    # import numpy as np
    # from math import *
    # import matplotlib.pyplot as plt
    # global Leg_thetaes

    # Initialize PortHandler1 Structs
    # Set the port path
    # Get methods and members of PortHandler1Linux or PortHandler1Windows
    portHandler1 = dynamixel.PortHandler(DEVICENAME1)
    portHandler2 = dynamixel.PortHandler(DEVICENAME2)

    # Initialize PacketHandler1 Structs
    packetHandler1 = dynamixel.PacketHandler(PROTOCOL_VERSION2)
    packetHandler2 = dynamixel.PacketHandler(PROTOCOL_VERSION2)

    index = 0
    dxl_comm_result = COMM_TX_FAIL                                   # Communication result
    goal_position = [MINIMUM_POSITION_VALUE, MAXIMUM_POSITION_VALUE] # Goal position of Dynamixel XM-W350-R
    dxl_error = 0                                                    # Dynamixel error
    present_position = 0                                             # Present position of Dynamixel XM-W350-R

    # Open port 1
    if portHandler1.openPort():
        print("Succeeded to open the port!")
    else:
        print("Failed to open the port!")
        print("Press any key to terminate...")
        getch()
        quit()

    # Open port 2
    if portHandler2.openPort():
        print("Succeeded to open the port!")
    else:
        print("Failed to open the port!")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate 1
    if portHandler1.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate!")
    else:
        print("Failed to change the baudrate!")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate 2
    if portHandler2.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate!")
    else:
        print("Failed to change the baudrate!")
        print("Press any key to terminate...")
        getch()
        quit()

    vel = 0
    acc = 0
    # Enable Dynamixel torque 1
    for ID in [1,2,3,10,11,12]:
        dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(portHandler1, ID, ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler1.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % (ID))

        packetHandler1.write4ByteTxRx(portHandler1, ID, ADDR_XM_ProfileVel, vel)
        packetHandler1.write4ByteTxRx(portHandler1, ID, ADDR_XM_ProfileACC, acc)

    # Enable Dynamixel torque 2
    for ID in [4,5,6,7,8,9]:
        dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler2, ID, ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler2.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % (ID))

        packetHandler2.write4ByteTxRx(portHandler2, ID, ADDR_XM_ProfileVel, vel)
        packetHandler2.write4ByteTxRx(portHandler2, ID, ADDR_XM_ProfileACC, acc)

    groupwrite_num1 = 0
    groupread_num1 = 0

    groupwrite_num2 = 0
    groupread_num2 = 0
    addr_xm_present_current = 0

    groupwrite_num1 = dynamixel.GroupSyncWrite(portHandler1, packetHandler1, ADDR_XM_GOAL_POSITION, 4)
    groupwrite_num2 = dynamixel.GroupSyncWrite(portHandler2, packetHandler1, ADDR_XM_GOAL_POSITION, 4)

    groupread_num1 = dynamixel.GroupSyncRead(portHandler1, packetHandler1, ADDR_XM_PRESENT_CURRENT, 2)
    groupread_num2 = dynamixel.GroupSyncRead(portHandler2, packetHandler2, ADDR_XM_PRESENT_CURRENT, 2)
    groupread_position_num1 = dynamixel.GroupSyncRead(portHandler1, packetHandler1, ADDR_XM_PRESENT_POSITION, 2)
    groupread_position_num2 = dynamixel.GroupSyncRead(portHandler2, packetHandler2, ADDR_XM_PRESENT_POSITION, 2)
    
    for device1 in [1,2,3,10,11,12]:
        groupread_num1.addParam(device1)
    for device2 in [4,5,6,7,8,9]:
        groupread_num2.addParam(device2)

    while 1:
        # global groupwrite_num1
        # global groupread_num1
        # global groupwrite_num2
        # global groupread_num2
        # global addr_xm_present_current

        #WRITE
        leg_position = legposition.get()
        if len(leg_position) != 0:    
            # for i in range(0, len(leg_position)):
                # if ADDR_XM[i] == ADDR_XM_GOAL_POSITION:
                # groupwrite_num1 = dynamixel.GroupSyncWrite(portHandler1, packetHandler1, ADDR_XM[i], 4)
                # groupwrite_num2 = dynamixel.GroupSyncWrite(portHandler2, packetHandler1, ADDR_XM[i], 4)

                # if ADDR_XM[i] == ADDR_XM_GOAL_CURRENT:
                #     groupwrite_num1 = dynamixel.GroupSyncWrite(portHandler1, packetHandler1, ADDR_XM[i], 2)
                #     groupwrite_num2 = dynamixel.GroupSyncWrite(portHandler2, packetHandler1, ADDR_XM[i], 2)

                # else:
                #     groupwrite_num1 = dynamixel.GroupSyncWrite(portHandler1, packetHandler1, ADDR_XM[i], 1)
                #     groupwrite_num2 = dynamixel.GroupSyncWrite(portHandler2, packetHandler1, ADDR_XM[i], 1)
                #     data[i] = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

            groupwrite_num1.addParam(1, dynamixel_data(leg_position[0]))
            groupwrite_num1.addParam(2, dynamixel_data(leg_position[1]))
            groupwrite_num1.addParam(3, dynamixel_data(leg_position[2]))

            groupwrite_num2.addParam(4, dynamixel_data(leg_position[3]))
            groupwrite_num2.addParam(5, dynamixel_data(leg_position[4]))
            groupwrite_num2.addParam(6, dynamixel_data(leg_position[5]))

            groupwrite_num2.addParam(7, dynamixel_data(leg_position[6]))
            groupwrite_num2.addParam(8, dynamixel_data(leg_position[7]))
            groupwrite_num2.addParam(9, dynamixel_data(leg_position[8]))

            groupwrite_num1.addParam(10, dynamixel_data(leg_position[9]))
            groupwrite_num1.addParam(11, dynamixel_data(leg_position[10]))
            groupwrite_num1.addParam(12, dynamixel_data(leg_position[11]))

            dxl_comm_result1 = groupwrite_num1.txPacket()
            dxl_comm_result2 = groupwrite_num2.txPacket()

            if (dxl_comm_result1 != COMM_SUCCESS):
                print("%s" % packetHandler1.getTxRxResult(dxl_comm_result1))

            if (dxl_comm_result2 != COMM_SUCCESS):
                print("%s" % packetHandler2.getTxRxResult(dxl_comm_result2))

            groupwrite_num1.clearParam()
            groupwrite_num2.clearParam()

        #READ CURRENT
        global current
        global presentposition
        current = []
        presentposition = []

        groupread_num1.txRxPacket()
        groupread_num2.txRxPacket()

        for ID in XM_ID:
            if not 4 <= ID <= 9:  
                #current
                addr_xm_present_current = groupread_num1.getData(ID, ADDR_XM_PRESENT_CURRENT,2)
                if addr_xm_present_current > 0x7fff:
                    addr_xm_present_current = addr_xm_present_current - 65536
                current.append(addr_xm_present_current)

                #present position
                # addr_xm_present_position = groupread_position_num1.getData(ID, ADDR_XM_PRESENT_POSITION,2)
                # if addr_xm_present_current > 0x7fff:
                #     addr_xm_present_current = addr_xm_present_current - 65536
                # presentposition.append(addr_xm_present_position)

            else:
                addr_xm_present_current = groupread_num2.getData(ID, ADDR_XM_PRESENT_CURRENT,2)
                if addr_xm_present_current > 0x7fff:
                    addr_xm_present_current = addr_xm_present_current - 65536
                current.append(addr_xm_present_current)

                #present position
                # addr_xm_present_position = groupread_position_num2.getData(ID, ADDR_XM_PRESENT_POSITION,2)
                # if addr_xm_present_current > 0x7fff:
                #     addr_xm_present_current = addr_xm_present_current - 65536
                # presentposition.append(addr_xm_present_position)
            # print("current : ", current)        

        print("current : ", current)
        print("present position : ", presentposition)

        time.sleep(0.01)

    return 0

def currentplot(id, data, result):
    x = 0
    while 1:
        print("Current "," :" , current)
        print("Present position", " : ", presentposition)
        time.sleep(0.1)
        # for i in [0,1,2,3,4,5,6,7,8,9,10,11]:
        # plt.scatter(x, current[0])
        # plt.pause(0.00001)
        # x += 0.00001
    return 0

def showplot(id, data, result):
    time.sleep(3)
    while 1:
        if len(current) >= 11:
            print("on")
            time.sleep(0.00001)

        # plt.show()
    return 0

def imu(id, data, result):

    def onAccelerationChange(self, acceleration, timestamp):
        print("Acceleration: \t"+ str(acceleration[0])+ "  |  "+ str(acceleration[1])+ "  |  "+ str(acceleration[2]))
        print("Timestamp: " + str(timestamp))
        print("----------")

    def onAngularRateUpdate(self, angularRate, timestamp):
        print("AngularRate: \t"+ str(angularRate[0])+ "  |  "+ str(angularRate[1])+ "  |  "+ str(angularRate[2]))
        print("Timestamp: " + str(timestamp))
        print("----------")

    def onMagneticFieldChange(self, magneticField, timestamp):
        print("MagneticField: \t"+ str(magneticField[0])+ "  |  "+ str(magneticField[1])+ "  |  "+ str(magneticField[2]))
        print("Timestamp: " + str(timestamp))
        print("----------")

    def onSpatialData(self, acceleration, angularRate, magneticField, timestamp):
        print("Acceleration: \t"+ str(acceleration[0])+ "  |  "+ str(acceleration[1])+ "  |  "+ str(acceleration[2]))
        print("AngularRate: \t"+ str(angularRate[0])+ "  |  "+ str(angularRate[1])+ "  |  "+ str(angularRate[2]))
        print("MagneticField: \t"+ str(magneticField[0])+ "  |  "+ str(magneticField[1])+ "  |  "+ str(magneticField[2]))
        print("Timestamp: " + str(timestamp))
        print("----------")

    while 1:
        accelerometer0 = Accelerometer()
        gyroscope0 = Gyroscope()
        magnetometer0 = Magnetometer()
        spatial0 = Spatial()

        accelerometer0.setOnAccelerationChangeHandler(onAccelerationChange)
        gyroscope0.setOnAngularRateUpdateHandler(onAngularRateUpdate)
        magnetometer0.setOnMagneticFieldChangeHandler(onMagneticFieldChange)
        spatial0.setOnSpatialDataHandler(onSpatialData)

        accelerometer0.openWaitForAttachment(5000)
        gyroscope0.openWaitForAttachment(5000)
        magnetometer0.openWaitForAttachment(5000)
        spatial0.openWaitForAttachment(5000)

        try:
            input("Press Enter to Stop\n")
        except (Exception, KeyboardInterrupt):
            pass

        accelerometer0.close()
        gyroscope0.close()
        magnetometer0.close()
        spatial0.close()

def sound_hanece(id, data, result):
    #10:on
    #20:off

    playsound.playsound('Hotdog_voice_1.mp3') #'Hello? I am Hotdog. Call me Hotdog'
    
    arduino.writeCharacteristic(11, bytes([10]))
    time.sleep(1)
    arduino.writeCharacteristic(11, bytes([20]))

    playsound.playsound('Hotdog_voice_2.mp3') #'Please stand with the harness in your right hand.'
    playsound.playsound('Hotdog_voice_3.mp3') #'Environment setup is completed. Lets start!'
    
    while 1:
        print("emergency_situation : ", emergency_situation)
        if emergency_situation == 1:
            arduino.writeCharacteristic(11, bytes([10]))
            playsound.playsound('Hotdog_voice_4.mp3')
        else:
            arduino.writeCharacteristic(11, bytes([20]))


def mic(id, data, result):
    print("Change here")
    # while 1:
    #     time.sleep(10)

def camera(id, data, result):
    while 1:
        emergency_situation = 0
        time.sleep(10)
        emergency_situation = 1

def test(id, data, result, legposition):
    while 1:
        legposition.put([2048,1536,1024,2048,1536,1024,2048,1536,1024,2048,1536,1024])
        time.sleep(5)
        legposition.put([2048,1536,824,2048,1536,824,2048,1536,824,2048,1536,824])
        time.sleep(5)
        

if __name__ == "__main__":

    result = Queue()
    data = Queue()
    legposition = Queue()

    # th1 = threading.Thread(target=SAC, args=(1, data, result))
    th2 = threading.Thread(target=u2d2dynamixel, args=(2, data, result, legposition))
    # th3 = threading.Thread(target=currentplot, args=(3, data, result))
    # th4 = threading.Thread(target=showplot, args=(4, data, result))
    # imu_th    = threading.Thread(target=imu   , args=(5, data, result))
    # sh_th  = threading.Thread(target=sound_hanece, args=(6, data, result))
    # mic_th    = threading.Thread(target=mic   , args=(7, data, result))
    # camera_th = threading.Thread(target=camera, args=(8, data, result))
    test = threading.Thread(target=test, args=(3, data, result, legposition))
    
    # th1.start()
    th2.start()
    # th3.start()
    # th4.start()
    # imu_th.start()
    # sh_th.start()
    # mic_th.start()
    # camera_th.start()
    test.start()

    # th1.join()
    th2.join()
    # th3.join()
    # th4.join()
    # imu_th.join()
    # sh_th.join()
    # mic_th.join()
    # camera_th.join()
    test.join()
