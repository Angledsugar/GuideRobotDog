#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
#  Hotdog_def.py
#
#  Created on: 2020. 11. 25.
#      Author: Choi chan yeok
#

import os, sys
import time
import serial
import math
import dynamixel_sdk as dynamixel
from mpl_toolkits import mplot3d
import numpy as np
from math import *
import matplotlib.pyplot as plt
global Leg_thetaes

Leg_thetaes = []
grd_leg_theta = []

def setupView(limit):
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Z")
    ax.set_zlabel("Y")
    return ax

setupView(200).view_init(elev=12., azim=28)

omega =  pi/4
phi =0
psi = 0

xm = 0
ym = 0
zm = 0

l1 = 70
l2 = 0
l3 = 220
l4 = 200

L = 350
W = 180

#FL,FR, RL, RR Leg Point
Lp=np.array([[100,-100,170,1],[100,-100,-170,1],[-100,-100,170,1],[-100,-100,-170,1]])

sHp=np.sin(pi/2)
cHp=np.cos(pi/2)

Lo=np.array([0,0,0,1])


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

# Control table address for Dynamixel XM-W350-R             # Control table address is different in Dynamixel model
ADDR_XM_TORQUE_ENABLE      = 64
ADDR_XM_Pgain      = 84
ADDR_XM_Igain      = 82
ADDR_XM_Dgain      = 80
ADDR_XM_FF2ND      = 88
ADDR_XM_FF1ST      = 90
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

# Initialize Groupsyncwrite instance
groupwrite_num1 = dynamixel.GroupSyncWrite(portHandler1, packetHandler1, ADDR_XM_GOAL_POSITION , 4)
groupread_num1 = dynamixel.GroupSyncRead(portHandler1, packetHandler1, ADDR_XM_PRESENT_CURRENT, 4)
Pwrite_num1 = dynamixel.GroupSyncWrite(portHandler1, packetHandler1, ADDR_XM_Pgain , 2)
Dwrite_num1 = dynamixel.GroupSyncWrite(portHandler1, packetHandler1, ADDR_XM_Dgain , 2)


groupwrite_num2 = dynamixel.GroupSyncWrite(portHandler2, packetHandler2, ADDR_XM_GOAL_POSITION , 4)
groupread_num2 = dynamixel.GroupSyncRead(portHandler2, packetHandler2, ADDR_XM_PRESENT_CURRENT, 4)
Pwrite_num2 = dynamixel.GroupSyncWrite(portHandler2, packetHandler2, ADDR_XM_Pgain , 2)
Dwrite_num2 = dynamixel.GroupSyncWrite(portHandler2, packetHandler2, ADDR_XM_Dgain , 2)

def p_data(Pgain):
    p_goalposition = [0,0]
    p_goalposition[0] = dynamixel.DXL_LOBYTE(dynamixel.DXL_LOWORD(Pgain))
    p_goalposition[1] = dynamixel.DXL_HIBYTE(dynamixel.DXL_LOWORD(Pgain))

    return p_goalposition

def P_write_1(ID1, ID2, ID3, Pgain1, Pgain2, Pgain3, times):
    Pwrite_num1.clearParam()
    Pwrite_num1.addParam(ID1, p_data(Pgain1))
    Pwrite_num1.addParam(ID2, p_data(Pgain2))
    Pwrite_num1.addParam(ID3, p_data(Pgain3))
    dxl_comm_result = Pwrite_num1.txPacket()

    if (dxl_comm_result != COMM_SUCCESS):
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
    #time.sleep(times)

    return 0

def P_write_2(ID1, ID2, ID3, Pgain1, Pgain2, Pgain3, times):
    Pwrite_num2.clearParam()
    Pwrite_num2.addParam(ID1, p_data(Pgain1))
    Pwrite_num2.addParam(ID2, p_data(Pgain2))
    Pwrite_num2.addParam(ID3, p_data(Pgain3))
    dxl_comm_result = Pwrite_num2.txPacket()

    if (dxl_comm_result != COMM_SUCCESS):
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
    #time.sleep(times)

    return 0

def position_data(position):
    param_goalposition = [0,0,0,0]
    param_goalposition[0] = dynamixel.DXL_LOBYTE(dynamixel.DXL_LOWORD(position))
    param_goalposition[1] = dynamixel.DXL_HIBYTE(dynamixel.DXL_LOWORD(position))
    param_goalposition[2] = dynamixel.DXL_LOBYTE(dynamixel.DXL_HIWORD(position))
    param_goalposition[3] = dynamixel.DXL_HIBYTE(dynamixel.DXL_HIWORD(position))

    return param_goalposition


def leg_position_write_1(ID1, ID2, ID3, position1, position2, position3, times):
    groupwrite_num1.clearParam()
    groupwrite_num1.addParam(ID1, position_data(position1))
    groupwrite_num1.addParam(ID2, position_data(position2))
    groupwrite_num1.addParam(ID3, position_data(position3))
    dxl_comm_result = groupwrite_num1.txPacket()

    if (dxl_comm_result != COMM_SUCCESS):
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
    #time.sleep(times)

    return 0

def leg_position_write_2(ID1, ID2, ID3, position1, position2, position3, times):
    groupwrite_num2.clearParam()
    groupwrite_num2.addParam(ID1, position_data(position1))
    groupwrite_num2.addParam(ID2, position_data(position2))
    groupwrite_num2.addParam(ID3, position_data(position3))
    dxl_comm_result = groupwrite_num2.txPacket()

    if (dxl_comm_result != COMM_SUCCESS):
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    #time.sleep(times)

    return 0

def leg_position_bulk_write(ID1, ID2, ID3, 
                            ID4, ID5, ID6,
                            ID7, ID8, ID9,
                            ID10, ID11, ID12, 
                            position1, position2, position3,
                            position4, position5, position6,
                            position7, position8, position9,
                            position10, position11, position12):

    groupwrite_num1.clearParam()
    groupwrite_num2.clearParam()
    groupwrite_num1.addParam(ID1, position_data(position1))
    groupwrite_num1.addParam(ID2, position_data(position2))
    groupwrite_num1.addParam(ID3, position_data(position3))
    groupwrite_num2.addParam(ID4, position_data(position4))
    groupwrite_num2.addParam(ID5, position_data(position5))
    groupwrite_num2.addParam(ID6, position_data(position6))
    groupwrite_num2.addParam(ID7, position_data(position7))
    groupwrite_num2.addParam(ID8, position_data(position8))
    groupwrite_num2.addParam(ID9, position_data(position9))
    groupwrite_num1.addParam(ID10, position_data(position10))
    groupwrite_num1.addParam(ID11, position_data(position11))
    groupwrite_num1.addParam(ID12, position_data(position12))
    dxl_comm_result1 = groupwrite_num1.txPacket()
    dxl_comm_result2 = groupwrite_num2.txPacket()

    if (dxl_comm_result1 != COMM_SUCCESS):
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result1))

    if (dxl_comm_result2 != COMM_SUCCESS):
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result2))

    return 0

def leg_current_read():
    current = []
    groupread_num.clearParam()
    for ID in XM_ID:
        groupread_num.addParam(ID)
    dxl_comm_result = groupread_num.txRxPacket()
    if (dxl_comm_result != COMM_SUCCESS):
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
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
    if portHandler1.getBytesAvailable() > 0 :
        read_data = portHandler1.readPort(1024)
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

def bodyIK(omega,phi,psi,xm,ym,zm):
    Rx = np.array([[1,0,0,0],
                   [0,np.cos(omega),-np.sin(omega),0],
                   [0,np.sin(omega),np.cos(omega),0],[0,0,0,1]])
    Ry = np.array([[np.cos(phi),0,np.sin(phi),0],
                   [0,1,0,0],
                   [-np.sin(phi),0,np.cos(phi),0],[0,0,0,1]])
    Rz = np.array([[np.cos(psi),-np.sin(psi),0,0],
                   [np.sin(psi),np.cos(psi),0,0],[0,0,1,0],[0,0,0,1]])
    Rxyz=Rx@Ry@Rz

    T = np.array([[0,0,0,xm],[0,0,0,ym],[0,0,0,zm],[0,0,0,0]])
    Tm = T+Rxyz

    return([Tm @ np.array([[cHp,0,sHp,L/2],[0,1,0,0],[-sHp,0,cHp,W/2],[0,0,0,1]]),
           Tm @ np.array([[cHp,0,sHp,L/2],[0,1,0,0],[-sHp,0,cHp,-W/2],[0,0,0,1]]),
           Tm @ np.array([[cHp,0,sHp,-L/2],[0,1,0,0],[-sHp,0,cHp,W/2],[0,0,0,1]]),
           Tm @ np.array([[cHp,0,sHp,-L/2],[0,1,0,0],[-sHp,0,cHp,-W/2],[0,0,0,1]])
           ])

def legIK(point):
    (x,y,z)=(point[0],point[1],point[2])
    F=sqrt(x**2+y**2-l1**2)
    G=F-l2  
    H=sqrt(G**2+z**2)
    theta1=-atan2(y,x)-atan2(F,-l1)

    D=(H**2-l3**2-l4**2)/(2*l3*l4)
    theta3=acos(D) 

    theta2=atan2(z,G)-atan2(l4*sin(theta3),l3+l4*cos(theta3))

    if len(Leg_thetaes) >= 12:
        Leg_thetaes.clear()
    
    Leg_thetaes.extend([theta1, theta2, theta3])
    return (theta1,theta2,theta3)

def calcLegPoints(angles):
    (theta1,theta2,theta3)=angles
    theta23=theta2+theta3

    T0=Lo
    T1=T0+np.array([-l1*cos(theta1),l1*sin(theta1),0,0])
    T2=T1+np.array([-l2*sin(theta1),-l2*cos(theta1),0,0])
    T3=T2+np.array([-l3*sin(theta1)*cos(theta2),-l3*cos(theta1)*cos(theta2),l3*sin(theta2),0])
    T4=T3+np.array([-l4*sin(theta1)*cos(theta23),-l4*cos(theta1)*cos(theta23),l4*sin(theta23),0])

    return np.array([T0,T1,T2,T3,T4])

def drawLegPoints(p):
    plt.plot([x[0] for x in p],[x[2] for x in p],[x[1] for x in p], 'k-', lw=3)
    plt.plot([p[0][0]],[p[0][2]],[p[0][1]],'bo',lw=2)
    plt.plot([p[4][0]],[p[4][2]],[p[4][1]],'ro',lw=2)    

def drawLegPair(Tl,Tr,Ll,Lr):
    Ix=np.array([[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    drawLegPoints([Tl@x for x in calcLegPoints(legIK(np.linalg.inv(Tl)@Ll))])
    drawLegPoints([Tr@Ix@x for x in calcLegPoints(legIK(Ix@np.linalg.inv(Tr)@Lr))])

def drawRobot(Lp,angles,center):
    (omega,phi,psi)=angles
    (xm,ym,zm)=center

    FP=[0,0,0,1]
    (Tlf,Trf,Tlb,Trb)= bodyIK(omega,phi,psi,xm,ym,zm)
    CP=[x@FP for x in [Tlf,Trf,Tlb,Trb]]

    CPs=[CP[x] for x in [0,1,3,2,0]]
    plt.plot([x[0] for x in CPs],[x[2] for x in CPs],[x[1] for x in CPs], 'bo-', lw=2)

    drawLegPair(Tlf,Trf,Lp[0],Lp[1])
    drawLegPair(Tlb,Trb,Lp[2],Lp[3])

def leg_theta(angles, center):
    drawRobot(Lp, angles, center)
    #plt.show()
    return Leg_thetaes

def four_leg_theta(angles, center):
    #radian
    grd_leg_theta = leg_theta(angles, center)

    #degree
    for i in range(len(grd_leg_theta)):
        grd_leg_theta[i] = math.degrees(grd_leg_theta[i])
        if i == 2 or i == 5 or i == 8 or i == 11:
            grd_leg_theta[i] = grd_leg_theta[i] - 180 

    #DXL Unit
    for i in range(len(grd_leg_theta)):
        grd_leg_theta[i] = int(grd_leg_theta[i]*(4096//360) + 2048)

    return grd_leg_theta


P_write_1(1, 2, 3, 1200, 1200 ,1200,0.1)
P_write_1(4, 5, 6, 1200, 1200 ,1200,0.1)
P_write_2(7, 8, 9, 1200, 1200 ,1200,0.1)
P_write_2(10, 11, 12, 1200, 1200 ,1200,0.1)