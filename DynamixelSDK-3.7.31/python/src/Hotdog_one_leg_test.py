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
import serial
import math


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

# os.sys.path.append('../../src/dynamixel_sdk')             # Path setting
#os.sys.path.append('../group_sync_write') 

import dynamixel_sdk as dynamixel
import Hotdog_IK_def as IK

print(dynamixel.__path__)

'''
from dynamixel_sdk import *  
from dynamixel_sdk import group_sync_write
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.robotis_def import * 
'''

#from dynamixel_sdk import *     
#import dynamixel_functions as dynamixel                                   # Uses DYNAMIXEL SDK library
#import dynamixel_sdk from dynamixel

# Control table address for Dynamixel XM-W350-R             # Control table address is different in Dynamixel model
ADDR_XM_TORQUE_ENABLE      = 64
ADDR_XM_GOAL_POSITION      = 116
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
DEVICENAME                  = "COM5"
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

# for ID in XM_ID:
#     dxl_comm_result_1, dxl_error_1 = packetHandler.read1ByteTxRx(portHandler, ID, ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE)
#     if dxl_comm_result_1 != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result_1))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error_1))
#     else:
#         print("Dynamixel#%d has been successfully connected" % (ID))

# Initialize Groupsyncwrite instance
groupwrite_num = dynamixel.GroupSyncWrite(portHandler, packetHandler, ADDR_XM_GOAL_POSITION , 4)
groupread_num = dynamixel.GroupSyncRead(portHandler, packetHandler, ADDR_XM_PRESENT_CURRENT, 4)

def position_data(position):
    param_goalposition = [0,0,0,0]
    param_goalposition[0] = dynamixel.DXL_LOBYTE(dynamixel.DXL_LOWORD(position))
    param_goalposition[1] = dynamixel.DXL_HIBYTE(dynamixel.DXL_LOWORD(position))
    param_goalposition[2] = dynamixel.DXL_LOBYTE(dynamixel.DXL_HIWORD(position))
    param_goalposition[3] = dynamixel.DXL_HIBYTE(dynamixel.DXL_HIWORD(position))

    return param_goalposition

def leg_position_write(ID1, ID2, ID3, position1, position2, position3, times):
    groupwrite_num.clearParam()
    groupwrite_num.addParam(ID1, position_data(position1))
    groupwrite_num.addParam(ID2, position_data(position2))
    groupwrite_num.addParam(ID3, position_data(position3))
    dxl_comm_result = groupwrite_num.txPacket()
    if (dxl_comm_result != COMM_SUCCESS):
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    time.sleep(times)

    return 0

def leg_current_read():
    current = []
    groupread_num.clearParam()
    for ID in XM_ID:
        groupread_num.addParam(ID)
    dxl_comm_result = groupread_num.txRxPacket()
    if (dxl_comm_result != COMM_SUCCESS):
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    for ID in XM_ID:
        addr_xm_present_current = groupread_num.getData(ID, ADDR_XM_PRESENT_CURRENT,2)
        if  addr_xm_present_current > 0x7fff:
            addr_xm_present_current = addr_xm_present_current- 65536
        current.append(addr_xm_present_current)

    print("Current "," :" , current)
    return current

def IMU_value(times):
    read_data = []
    IMU_x = []
    IMU_y = []
    IMU_z = []
    comma = 0 
    if portHandler.getBytesAvailable() > 0 :
        read_data = portHandler.readPort(1024)
        read_data = read_data.decode('ascii')
        for i in range(len(read_data)):
            if read_data[i] == ',' and comma == 0:
                comma = comma + 1
                IMU_x = read_data[0:i]
                comma_first = i
            elif read_data[i] == ',' and comma == 1:
                comma = comma + 1
                IMU_y = read_data[comma_first+1:i]
                comma_first = i
            elif read_data[i] == ',' and comma == 2:
                IMU_z = read_data[comma_first+1:i]
                comma = 0
                break
    
    print("IMU_x:", IMU_x, "IMU_y:", IMU_y, "IMU_z:", IMU_z)
    #time.sleep(times)
    
    return read_data

grd_leg_theta = []
leg_current_read_now = []
position = [2048,2560]

#radian
grd_leg_theta = IK.leg_theta((-0.4,0,0), (0,200,0))

#degree
for i in range(len(grd_leg_theta)):
    grd_leg_theta[i] = math.degrees(grd_leg_theta[i])
    if i == 2 or i == 5 or i == 8 or i == 11:
        grd_leg_theta[i] = grd_leg_theta[i] - 180 
print(grd_leg_theta)

#DXL Unit
for i in range(len(grd_leg_theta)):
    grd_leg_theta[i] = int(grd_leg_theta[i]*(4096//360) + 2048)

print(grd_leg_theta)

four_grd_leg_theta((-0.4, 0, 0), (0, 200, 0))

leg_position_write(1,2,3, grd_leg_theta[3], grd_leg_theta[4], grd_leg_theta[5], 0.1)
leg_position_write(4,5,6, grd_leg_theta[0], grd_leg_theta[1], grd_leg_theta[2], 0.1)
leg_position_write(7,8,9, grd_leg_theta[9], grd_leg_theta[10], grd_leg_theta[11], 0.1)
leg_position_write(10,11,12, grd_leg_theta[6], grd_leg_theta[7], grd_leg_theta[8], 0.1)

'''
while 1:
    

    #IMU = IMU_value(0.01)
    #position 2048
    leg_position_write(1,2,3, position[0], position[0], position[0], 0.1)
    time.sleep(0.1)
    
    #position 2560 
    leg_position_write(1,2,3, 2048, 2048, 1848, 0.1)
    time.sleep(0.1)
    
    leg_current_read_now = leg_current_read()
    print(leg_current_read_now[2])
    if abs(leg_current_read_now[2]) >= 50:
        leg_position_write(4,5,6, 2048, 2560, 2560, 0.1)
        leg_position_write(7,8,9, 2048, 2560, 2560, 0.1)
        leg_position_write(10,11,12, 2048, 2560, 2560, 0.1)
    else:
        leg_position_write(4,5,6, 2048, 2048, 2048, 0.1)
        leg_position_write(7,8,9, 2048, 2048, 2048, 0.1)
        leg_position_write(10,11,12, 2048, 2048, 2048, 0.1)
'''




time.sleep(10000)

# Disable Dynamixel Torque
for ID in XM_ID:
    packetHandler.write1ByteTxRx(portHandler, ID, ADDR_XM_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
#portHandler.closePort(portHandler)
