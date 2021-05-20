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

current = []
presentposition = []
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

def RL(id, data, result):
    import numpy as np
    import matplotlib.pyplot as plt
    import copy

    import os
    import sys
    sys.path.append('../../')

    from spotmicro.GymEnvs.spot_bezier_env import spotBezierEnv
    from spotmicro.util.gui import GUI
    from spotmicro.Kinematics.SpotKinematics import SpotModel
    from spotmicro.Kinematics.LieAlgebra import RPY
    from spotmicro.GaitGenerator.Bezier import BezierGait

    # TESTING
    from spotmicro.OpenLoopSM.SpotOL import BezierStepper

    import time

    import torch

    print("STARTING SPOT TEST ENV")
    seed = 0
    max_timesteps = 4e6
    file_name = "spot_ars_"

    # Find abs path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    # print(my_path)
    results_path = os.path.join(my_path, "../results")
    models_path = os.path.join(my_path, "../models")

    if not os.path.exists(results_path):
        os.makedirs(results_path)

    if not os.path.exists(models_path):
        os.makedirs(models_path)

    env = spotBezierEnv(render=True,
                        on_rack=False,
                        height_field=True,
                        draw_foot_path=False)

    # Set seeds
    env.seed(seed)
    torch.manual_seed(seed)
    np.random.seed(seed)

    state_dim = env.observation_space.shape[0]
    print("STATE DIM: {}".format(state_dim))
    action_dim = env.action_space.shape[0]
    print("ACTION DIM: {}".format(action_dim))
    max_action = float(env.action_space.high[0])

    state = env.reset()

    g_u_i = GUI(env.spot.quadruped)

    spot = SpotModel()
    T_bf0 = spot.WorldToFoot
    T_bf = copy.deepcopy(T_bf0)

    bzg = BezierGait(dt=env._time_step)

    bz_step = BezierStepper(dt=env._time_step, mode=0)

    action = env.action_space.sample()

    yaw = 0.0

    print("STARTED SPOT TEST ENV")
    t = 0
    while t < (int(max_timesteps)):

        bz_step.ramp_up()

        pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth = bz_step.StateMachine(
        )

        pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth = g_u_i.UserInput(
        )

        yaw = env.return_yaw()

        P_yaw = 5.0

        YawRate += - yaw * P_yaw

        # print("YAW RATE: {}".format(YawRate))

        # TEMP
        bz_step.StepLength = StepLength
        bz_step.LateralFraction = LateralFraction
        bz_step.YawRate = YawRate
        bz_step.StepVelocity = StepVelocity

        contacts = state[-4:]

        # Get Desired Foot Poses
        T_bf = bzg.GenerateTrajectory(StepLength, LateralFraction, YawRate,
                                      StepVelocity, T_bf0, T_bf,
                                      ClearanceHeight, PenetrationDepth,
                                      contacts)
        joint_angles = spot.IK(orn, pos, T_bf)
        data.put(joint_angles)################################
        env.pass_joint_angles(joint_angles.reshape(-1))
        # Get External Observations
        env.spot.GetExternalObservations(bzg, bz_step)
        # Step
        state, reward, done, _ = env.step(action)
        if done:
            print("DONE")

        # time.sleep(1.0)

        t += 1
    env.close()

def u2d2dynamixel(id, data, result):
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

    # Enable Dynamixel torque 1
    for ID in [1,2,3,10,11,12]:
        dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(portHandler1, ID, ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler1.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % (ID))

    # Enable Dynamixel torque 2
    for ID in [4,5,6,7,8,9]:
        dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler2, ID, ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler2.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % (ID))

    vel = 200
    acc = 200
    packetHandler1.write4ByteTxRx(portHandler1, FR1_ID, ADDR_XM_ProfileVel, vel)
    packetHandler1.write4ByteTxRx(portHandler1, FR2_ID, ADDR_XM_ProfileVel, vel)
    packetHandler1.write4ByteTxRx(portHandler1, FR3_ID, ADDR_XM_ProfileVel, vel)
    packetHandler1.write4ByteTxRx(portHandler1, FL4_ID, ADDR_XM_ProfileVel, vel)
    packetHandler1.write4ByteTxRx(portHandler1, FL5_ID, ADDR_XM_ProfileVel, vel)
    packetHandler1.write4ByteTxRx(portHandler1, FL6_ID, ADDR_XM_ProfileVel, vel)

    packetHandler2.write4ByteTxRx(portHandler2, RR7_ID, ADDR_XM_ProfileVel, vel)
    packetHandler2.write4ByteTxRx(portHandler2, RR8_ID, ADDR_XM_ProfileVel, vel)
    packetHandler2.write4ByteTxRx(portHandler2, RR9_ID, ADDR_XM_ProfileVel, vel)
    packetHandler2.write4ByteTxRx(portHandler2, RL10_ID, ADDR_XM_ProfileVel, vel)
    packetHandler2.write4ByteTxRx(portHandler2, RL11_ID, ADDR_XM_ProfileVel, vel)
    packetHandler2.write4ByteTxRx(portHandler2, RL12_ID, ADDR_XM_ProfileVel, vel)

    packetHandler1.write4ByteTxRx(portHandler1, FR1_ID, ADDR_XM_ProfileACC, acc)
    packetHandler1.write4ByteTxRx(portHandler1, FR2_ID, ADDR_XM_ProfileACC, acc)
    packetHandler1.write4ByteTxRx(portHandler1, FR3_ID, ADDR_XM_ProfileACC, acc)
    packetHandler1.write4ByteTxRx(portHandler1, FL4_ID, ADDR_XM_ProfileACC, acc)
    packetHandler1.write4ByteTxRx(portHandler1, FL5_ID, ADDR_XM_ProfileACC, acc)
    packetHandler1.write4ByteTxRx(portHandler1, FL6_ID, ADDR_XM_ProfileACC, acc)

    packetHandler2.write4ByteTxRx(portHandler2, RR7_ID, ADDR_XM_ProfileACC, acc)
    packetHandler2.write4ByteTxRx(portHandler2, RR8_ID, ADDR_XM_ProfileACC, acc)
    packetHandler2.write4ByteTxRx(portHandler2, RR9_ID, ADDR_XM_ProfileACC, acc)
    packetHandler2.write4ByteTxRx(portHandler2, RL10_ID, ADDR_XM_ProfileACC, acc)
    packetHandler2.write4ByteTxRx(portHandler2, RL11_ID, ADDR_XM_ProfileACC, acc)
    packetHandler2.write4ByteTxRx(portHandler2, RL12_ID, ADDR_XM_ProfileACC, acc)

    groupwrite_num1 = 0
    groupread_num1 = 0
    groupwrite_num2 = 0
    groupread_num2 = 0
    addr_xm_present_current = 0

    groupread_num1 = dynamixel.GroupSyncRead(portHandler1, packetHandler1, ADDR_XM_PRESENT_CURRENT, 4)
    groupread_num2 = dynamixel.GroupSyncRead(portHandler2, packetHandler2, ADDR_XM_PRESENT_CURRENT, 4)

    groupread_num1.clearParam()
    groupread_num2.clearParam()
    
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
        # if len(ADDR_XM) != 0:    
        #     for i in range(0, len(ADDR_XM)):
        #         if ADDR_XM[i] == ADDR_XM_GOAL_POSITION:
        #             groupwrite_num1 = dynamixel.GroupSyncWrite(portHandler1, packetHandler1, ADDR_XM[i], 4)
        #             groupwrite_num2 = dynamixel.GroupSyncWrite(portHandler2, packetHandler1, ADDR_XM[i], 4)

        #         if ADDR_XM[i] == ADDR_XM_GOAL_CURRENT:
        #             groupwrite_num1 = dynamixel.GroupSyncWrite(portHandler1, packetHandler1, ADDR_XM[i], 2)
        #             groupwrite_num2 = dynamixel.GroupSyncWrite(portHandler2, packetHandler1, ADDR_XM[i], 2)

        #         else:
        #             groupwrite_num1 = dynamixel.GroupSyncWrite(portHandler1, packetHandler1, ADDR_XM[i], 1)
        #             groupwrite_num2 = dynamixel.GroupSyncWrite(portHandler2, packetHandler1, ADDR_XM[i], 1)
        #             data[i] = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

        #         groupwrite_num1.addParam(ID1, dynamixel_data(data[i][0]))
        #         groupwrite_num1.addParam(ID2, dynamixel_data(data[i][1]))
        #         groupwrite_num1.addParam(ID3, dynamixel_data(data[i][2]))

        #         groupwrite_num2.addParam(ID4, dynamixel_data(data[i][3]))
        #         groupwrite_num2.addParam(ID5, dynamixel_data(data[i][4]))
        #         groupwrite_num2.addParam(ID6, dynamixel_data(data[i][5]))

        #         groupwrite_num2.addParam(ID7, dynamixel_data(data[i][6]))
        #         groupwrite_num2.addParam(ID8, dynamixel_data(data[i][7]))
        #         groupwrite_num2.addParam(ID9, dynamixel_data(data[i][8]))

        #         groupwrite_num1.addParam(ID10, dynamixel_data(data[i][9]))
        #         groupwrite_num1.addParam(ID11, dynamixel_data(data[i][10]))
        #         groupwrite_num1.addParam(ID12, dynamixel_data(data[i][11]))

        #     dxl_comm_result1 = groupwrite_num1.txPacket()
        #     dxl_comm_result2 = groupwrite_num2.txPacket()

        #     if (dxl_comm_result1 != COMM_SUCCESS):
        #         print("%s" % packetHandler1.getTxRxResult(dxl_comm_result1))

        #     if (dxl_comm_result2 != COMM_SUCCESS):
        #         print("%s" % packetHandler2.getTxRxResult(dxl_comm_result2))

        #     groupwrite_num1.clearParam()
        #     groupwrite_num2.clearParam()

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
                addr_xm_present_position = groupread_num1.getData(ID, ADDR_XM_PRESENT_POSITION,2)
                # if addr_xm_present_current > 0x7fff:
                #     addr_xm_present_current = addr_xm_present_current - 65536
                presentposition.append(addr_xm_present_position)

            else:
                addr_xm_present_current = groupread_num2.getData(ID, ADDR_XM_PRESENT_CURRENT,2)
                if addr_xm_present_current > 0x7fff:
                    addr_xm_present_current = addr_xm_present_current - 65536
                current.append(addr_xm_present_current)

                #present position
                addr_xm_present_position = groupread_num2.getData(ID, ADDR_XM_PRESENT_POSITION,2)
                # if addr_xm_present_current > 0x7fff:
                #     addr_xm_present_current = addr_xm_present_current - 65536
                presentposition.append(addr_xm_present_position)

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


if __name__ == "__main__":

    result = Queue()
    data = Queue()
    ADDR_XM = Queue()

    # th1 = threading.Thread(target=RL, args=(1, data, result))
    th2 = threading.Thread(target=u2d2dynamixel, args=(2, data, result))
    th3 = threading.Thread(target=currentplot, args=(3, data, result))
    # th4 = threading.Thread(target=showplot, args=(4, data, result))
    
    # th1.start()
    th2.start()
    th3.start()
    # th4.start()

    # th1.join()
    th2.join()
    th3.join()
    # th4.join()