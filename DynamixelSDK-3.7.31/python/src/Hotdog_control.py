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
import Hotdog_def as hd
import matplotlib.pyplot as plt

hd.leg_position_write(1,2,3, 2048, 2048, 1024, 0.1)
hd.leg_position_write(4,5,6, 2048, 2048, 1024, 0.1)
hd.leg_position_write(7,8,9, 2048, 2048, 1024, 0.1)
hd.leg_position_write(10,11,12, 2048, 2048, 1024, 0.1)
time.sleep(1)

motion_a = []
motion_b = []
motion_c = []
la = [(-0.4, 0, 0), (0,200,0), (0, 0, 0), (0,200,0),(0.4, 0, 0), (0, 200, 0)]    
i = 0

motion_a.extend(hd.four_leg_theta(la[0], la[1]))
print(motion_a)
motion_b.extend(hd.four_leg_theta(la[2], la[3]))
print(motion_b)
motion_c.extend(hd.four_leg_theta(la[4], la[5]))
print(motion_c)

#plt.show()

while 1:
    # print("motion_a: ", motion_a)
    # print("motion_b: ", motion_b)
    # print("motion_c: ", motion_c)

    hd.leg_position_write(1,2,3, motion_a[3], motion_a[4], motion_a[5], 0.1)
    hd.leg_position_write(4,5,6, motion_a[0], motion_a[1], motion_a[2], 0.1)
    hd.leg_position_write(7,8,9, motion_a[9], motion_a[10], motion_a[11], 0.1)
    hd.leg_position_write(10,11,12, motion_a[6], motion_a[7], motion_a[8], 1)
    
    hd.leg_position_write(1,2,3, motion_b[3], motion_b[4], motion_b[5], 0.1)
    hd.leg_position_write(4,5,6, motion_b[0], motion_b[1], motion_b[2], 0.1)
    hd.leg_position_write(7,8,9, motion_b[9], motion_b[10], motion_b[11], 0.1)
    hd.leg_position_write(10,11,12, motion_b[6], motion_b[7], motion_b[8], 1)
    
    hd.leg_position_write(1,2,3, motion_c[3], motion_c[4], motion_c[5], 0.1)
    hd.leg_position_write(4,5,6, motion_c[0], motion_c[1], motion_c[2], 0.1)
    hd.leg_position_write(7,8,9, motion_c[9], motion_c[10], motion_c[11], 0.1)
    hd.leg_position_write(10,11,12, motion_c[6], motion_c[7], motion_c[8], 1)

    hd.leg_position_write(1,2,3, motion_b[3], motion_b[4], motion_b[5], 0.1)
    hd.leg_position_write(4,5,6, motion_b[0], motion_b[1], motion_b[2], 0.1)
    hd.leg_position_write(7,8,9, motion_b[9], motion_b[10], motion_b[11], 0.1)
    hd.leg_position_write(10,11,12, motion_b[6], motion_b[7], motion_b[8], 1)
    
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
