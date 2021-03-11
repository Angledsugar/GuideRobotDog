#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# protocol_combined.py
#
#  Created on: 2016. 6. 16.
#      Author: Ryu Woon Jung (Leon)
#

#
# *********     Protocol Combined Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 1.0 and 2.0
# This example is tested with a Dynamixel MX-28, a Dynamixel PRO 54-200 and an USB2DYNAMIXEL
# Be sure that properties of Dynamixel MX and PRO are already set as %% MX - ID : 1 / Baudnum : 1 (Baudrate : 1000000) , PRO - ID : 1 / Baudnum : 3 (Baudrate : 1000000)
#

# Be aware that:
# This example configures two different control tables (especially, if it uses Dynamixel and Dynamixel PRO). It may modify critical Dynamixel parameter on the control table, if DYNAMIXEL have wrong ID.
#

import os, sys
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(sys.stdin.fileno())
    def getch():
        return sys.stdin.read(1)

os.sys.path.append('../dynamixel_sdk')             # Path setting

import dynamixel_sdk as dynamixel   
#from dynamixel_sdk import *                                 # Uses DYNAMIXEL SDK library
#import dynamixel_sdk as dynamixel

# Control table address for Dynamixel XM-W350-R             # Control table address is different in Dynamixel model
ADDR_XM_TORQUE_ENABLE      = 64
ADDR_XM_GOAL_POSITION      = 116
ADDR_XM_PRESENT_POSITION   = 132
ADDR_XM_ProfileACC   = 108
ADDR_XM_ProfileVel   = 112

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

BAUDRATE                    = 3000000
DEVICENAME                  = "COM6"
#DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')        # Check which port is being used on your controller
                                                                     # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque

MINIMUM_POSITION_VALUE       = 2000                         # Dynamixel will rotate between this value
MAXIMUM_POSITION_VALUE       = 2100                         # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
MOVING_STATUS_THRESHOLD      = 20                           # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = dynamixel.PortHandler(DEVICENAME)

# Initialize PacketHandler Structs
packetHandler = dynamixel.PacketHandler(PROTOCOL_VERSION2)

index = 0
dxl_comm_result = COMM_TX_FAIL                                   # Communication result
goal_position = [MINIMUM_POSITION_VALUE, MAXIMUM_POSITION_VALUE] # Goal position of Dynamixel XM-W350-R
dxl_error = 0                                                    # Dynamixel error
present_position = 0                                             # Present position of Dynamixel XM-W350-R

# Open port
if portHandler.openPort():
    print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel torque
for ID in XM_ID:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % (ID))

'''
# 90degree Hotdog position
packetHandler.write4ByteTxRx(portHandler, FR1_ID, ADDR_XM_GOAL_POSITION, 2048)
packetHandler.write4ByteTxRx(portHandler, FR2_ID, ADDR_XM_GOAL_POSITION, 2048)
packetHandler.write4ByteTxRx(portHandler, FR3_ID, ADDR_XM_GOAL_POSITION, 1024)

packetHandler.write4ByteTxRx(portHandler, FL4_ID, ADDR_XM_GOAL_POSITION, 2048)
packetHandler.write4ByteTxRx(portHandler, FL5_ID, ADDR_XM_GOAL_POSITION, 2048)
packetHandler.write4ByteTxRx(portHandler, FL6_ID, ADDR_XM_GOAL_POSITION, 1024)

packetHandler.write4ByteTxRx(portHandler, RR7_ID, ADDR_XM_GOAL_POSITION, 2048)
packetHandler.write4ByteTxRx(portHandler, RR8_ID, ADDR_XM_GOAL_POSITION, 2048)
packetHandler.write4ByteTxRx(portHandler, RR9_ID, ADDR_XM_GOAL_POSITION, 1024)

packetHandler.write4ByteTxRx(portHandler, RL10_ID, ADDR_XM_GOAL_POSITION, 2048)
packetHandler.write4ByteTxRx(portHandler, RL11_ID, ADDR_XM_GOAL_POSITION, 2048)
packetHandler.write4ByteTxRx(portHandler, RL12_ID, ADDR_XM_GOAL_POSITION, 1024)
time.sleep(1000)
'''

packetHandler.write4ByteTxRx(portHandler, FR1_ID, ADDR_XM_ProfileVel, 50)
packetHandler.write4ByteTxRx(portHandler, FL4_ID, ADDR_XM_ProfileVel, 50)
packetHandler.write4ByteTxRx(portHandler, RR7_ID, ADDR_XM_ProfileVel, 50)
packetHandler.write4ByteTxRx(portHandler, RL10_ID, ADDR_XM_ProfileVel, 50)

packetHandler.write4ByteTxRx(portHandler, FR2_ID, ADDR_XM_ProfileVel, 50)
packetHandler.write4ByteTxRx(portHandler, FL5_ID, ADDR_XM_ProfileVel, 50)
packetHandler.write4ByteTxRx(portHandler, RR8_ID, ADDR_XM_ProfileVel, 50)
packetHandler.write4ByteTxRx(portHandler, RL11_ID, ADDR_XM_ProfileVel, 50)

packetHandler.write4ByteTxRx(portHandler, FR3_ID, ADDR_XM_ProfileVel, 50)
packetHandler.write4ByteTxRx(portHandler, FL6_ID, ADDR_XM_ProfileVel, 50)
packetHandler.write4ByteTxRx(portHandler, RR9_ID, ADDR_XM_ProfileVel, 50)
packetHandler.write4ByteTxRx(portHandler, RL12_ID, ADDR_XM_ProfileVel, 50)

packetHandler.write4ByteTxRx(portHandler, FR2_ID, ADDR_XM_ProfileACC, 10)
packetHandler.write4ByteTxRx(portHandler, FL5_ID, ADDR_XM_ProfileACC, 10)
packetHandler.write4ByteTxRx(portHandler, RR8_ID, ADDR_XM_ProfileACC, 10)
packetHandler.write4ByteTxRx(portHandler, RL11_ID, ADDR_XM_ProfileACC, 10)

packetHandler.write4ByteTxRx(portHandler, FR3_ID, ADDR_XM_ProfileACC, 50)
packetHandler.write4ByteTxRx(portHandler, FL6_ID, ADDR_XM_ProfileACC, 50)
packetHandler.write4ByteTxRx(portHandler, RR9_ID, ADDR_XM_ProfileACC, 50) 
packetHandler.write4ByteTxRx(portHandler, RL12_ID, ADDR_XM_ProfileACC, 50)


# Initial Hotdog position
packetHandler.write4ByteTxRx(portHandler, FR1_ID, ADDR_XM_GOAL_POSITION, 2048)
packetHandler.write4ByteTxRx(portHandler, FR2_ID, ADDR_XM_GOAL_POSITION, 1536)
packetHandler.write4ByteTxRx(portHandler, FR3_ID, ADDR_XM_GOAL_POSITION, 1124)

packetHandler.write4ByteTxRx(portHandler, FL4_ID, ADDR_XM_GOAL_POSITION, 2048)
packetHandler.write4ByteTxRx(portHandler, FL5_ID, ADDR_XM_GOAL_POSITION, 1536)
packetHandler.write4ByteTxRx(portHandler, FL6_ID, ADDR_XM_GOAL_POSITION, 1124)

packetHandler.write4ByteTxRx(portHandler, RR7_ID, ADDR_XM_GOAL_POSITION, 2048)
packetHandler.write4ByteTxRx(portHandler, RR8_ID, ADDR_XM_GOAL_POSITION, int((180 - 23.1125)*(4096//360)))
packetHandler.write4ByteTxRx(portHandler, RR9_ID, ADDR_XM_GOAL_POSITION, int(96.3977*(4096//360)))

packetHandler.write4ByteTxRx(portHandler, RL10_ID, ADDR_XM_GOAL_POSITION, 2048)
packetHandler.write4ByteTxRx(portHandler, RL11_ID, ADDR_XM_GOAL_POSITION, int((180 - 23.1125)*(4096//360)))
packetHandler.write4ByteTxRx(portHandler, RL12_ID, ADDR_XM_GOAL_POSITION, int(96.3977*(4096//360)))
time.sleep(5)

sec = 0.2

while 1:
    print("step1_FR,RL")
    degree = [90, 180-34.6286, 99.0964] #134.4153, 88.8306
    degree_DX = [2048, int(degree[1]*(4096//360)), int(degree[2]*(4096//360))]
    degree2 = [90, 180-23.1125, 96.3977] #134.4153, 88.8306
    degree_DX2 = [2048, int(degree2[1]*(4096//360)), int(degree2[2]*(4096//360))]
    packetHandler.write4ByteTxRx(portHandler, FR2_ID, ADDR_XM_GOAL_POSITION, degree_DX[1])
    packetHandler.write4ByteTxRx(portHandler, FR3_ID, ADDR_XM_GOAL_POSITION, degree_DX[2])
    packetHandler.write4ByteTxRx(portHandler, RL11_ID, ADDR_XM_GOAL_POSITION, degree_DX2[1])
    packetHandler.write4ByteTxRx(portHandler, RL12_ID, ADDR_XM_GOAL_POSITION, degree_DX2[2])
    time.sleep(sec)
    

    print('step2_FR,RL')
    degree = [90, 180-34.9028, 85.3528]
    degree_DX = [2048, int(degree[1]*(4096//360)), int(degree[2]*(4096//360))]
    degree2 = [90, 180-21.4216, 180-104.1678] #134.4153, 88.8306
    degree_DX2 = [2048, int(degree2[1]*(4096//360)), int(degree2[2]*(4096//360))]
    packetHandler.write4ByteTxRx(portHandler, FR2_ID, ADDR_XM_GOAL_POSITION, degree_DX[1])
    packetHandler.write4ByteTxRx(portHandler, FR3_ID, ADDR_XM_GOAL_POSITION, degree_DX[2])
    packetHandler.write4ByteTxRx(portHandler, RL11_ID, ADDR_XM_GOAL_POSITION, degree_DX2[1])
    packetHandler.write4ByteTxRx(portHandler, RL12_ID, ADDR_XM_GOAL_POSITION, degree_DX2[2])
    time.sleep(sec)

    print("step3_FR,RL")
    degree = [90, 180-44.9194, 97.7173]
    degree_DX = [2048, int(degree[1]*(4096//360)), int(degree[2]*(4096//360))]
    degree2 = [90, 180-30.9475, 98.8754] #134.4153, 88.8306
    degree_DX2 = [2048, int(degree2[1]*(4096//360)), int(degree2[2]*(4096//360))]
    packetHandler.write4ByteTxRx(portHandler, FR2_ID, ADDR_XM_GOAL_POSITION, degree_DX[1])
    packetHandler.write4ByteTxRx(portHandler, FR3_ID, ADDR_XM_GOAL_POSITION, degree_DX[2])
    packetHandler.write4ByteTxRx(portHandler, RL11_ID, ADDR_XM_GOAL_POSITION, degree_DX2[1])
    packetHandler.write4ByteTxRx(portHandler, RL12_ID, ADDR_XM_GOAL_POSITION, degree_DX2[2])
    time.sleep(sec)

    print("step1_FL,RR")
    degree = [90, 180-34.6286, 99.0964] #134.4153, 88.8306
    degree_DX = [2048, int(degree[1]*(4096//360)), int(degree[2]*(4096//360))]
    degree2 = [90, 180-23.1125, 96.3977] #134.4153, 88.8306
    degree_DX2 = [2048, int(degree2[1]*(4096//360)), int(degree2[2]*(4096//360))]
    packetHandler.write4ByteTxRx(portHandler, FL5_ID, ADDR_XM_GOAL_POSITION, degree_DX[1])
    packetHandler.write4ByteTxRx(portHandler, FL6_ID, ADDR_XM_GOAL_POSITION, degree_DX[2])
    packetHandler.write4ByteTxRx(portHandler, RR8_ID, ADDR_XM_GOAL_POSITION, degree_DX2[1])
    packetHandler.write4ByteTxRx(portHandler, RR9_ID, ADDR_XM_GOAL_POSITION, degree_DX2[2])
    time.sleep(sec)
    

    print('step2_FL,RR')
    degree = [90, 180-34.9028, 85.3528]
    degree_DX = [2048, int(degree[1]*(4096//360)), int(degree[2]*(4096//360))]
    degree2 = [90, 180-21.4216, 104.1678] #134.4153, 88.8306
    degree_DX2 = [2048, int(degree2[1]*(4096//360)), int(degree[2]*(4096//360))]
    packetHandler.write4ByteTxRx(portHandler, FL5_ID, ADDR_XM_GOAL_POSITION, degree_DX[1])
    packetHandler.write4ByteTxRx(portHandler, FL6_ID, ADDR_XM_GOAL_POSITION, degree_DX[2])
    packetHandler.write4ByteTxRx(portHandler, RR8_ID, ADDR_XM_GOAL_POSITION, degree_DX2[1])
    packetHandler.write4ByteTxRx(portHandler, RR9_ID, ADDR_XM_GOAL_POSITION, degree_DX2[2])
    time.sleep(sec)

    print("step3_FL,RR")
    degree = [90, 180-44.9194, 97.7173]
    degree_DX = [2048, int(degree[1]*(4096//360)), int(degree[2]*(4096//360))]
    degree2 = [90, 180-30.9475, 98.8754] #134.4153, 88.8306
    degree_DX2 = [2048, int(degree2[1]*(4096//360)), int(degree2[2]*(4096//360))]
    packetHandler.write4ByteTxRx(portHandler, FL5_ID, ADDR_XM_GOAL_POSITION, degree_DX[1])
    packetHandler.write4ByteTxRx(portHandler, FL6_ID, ADDR_XM_GOAL_POSITION, degree_DX[2])
    packetHandler.write4ByteTxRx(portHandler, RR8_ID, ADDR_XM_GOAL_POSITION, degree_DX2[1])
    packetHandler.write4ByteTxRx(portHandler, RR9_ID, ADDR_XM_GOAL_POSITION, degree_DX2[2])
    time.sleep(sec)


# Disable Dynamixel Torque
for ID in XM_ID:
    packetHandler.write1ByteTxRx(portHandler, ID, ADDR_XM_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
#portHandler.closePort(portHandler)
