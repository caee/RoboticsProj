# -*- coding: utf-8 -*-
"""
Created on Tue Nov  7 13:17:22 2023

@author: carle
"""

import dynamixel_sdk as dxl
import time
import numpy as np

import matplotlib.pyplot as plt


ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_CW_COMPLIANCE_MARGIN = 26
ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
ADDR_MX_CW_COMPLIANCE_SLOPE = 28
ADDR_MX_CCW_COMPLIANCE_SLOPE = 29
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_POSITION = 36
ADDR_MX_PUNCH = 48
ADDR_MOVING=46
PROTOCOL_VERSION = 1.0

#For changing modes
ADDR_CW_ANGLE_LIMIT=6
ADDR_CCW_ANGLE_LIMIT=8



# DXL_IDS = [1,2,3,4]
DXL_IDS = [1,2,3,4]
DEVICENAME = 'COM4'
BAUDRATE = 1000000
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
    
# # Get Dynamixel model number and ping it
# DXL_ID_to_ping = 1
# dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID_to_ping)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))
# else:
#     print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (DXL_ID_to_ping, dxl_model_number))


#Change motor
idx=1
#Change mode
wheel = 0
joint = 1
mode = joint
#Angle limits
# Joint 1: 60-220deg
# Joint 2: 20-280
#Joint 3: 150-190
#Base : 0-300
DPU=300/1023 #Degrees per unit of position. Setting 300deg to max, maps to 1023
J1min=60/DPU
J1max=220/DPU
J2min=20/DPU
J2max=280/DPU
J3min=150/DPU
J3max=190/DPU
Bmin=1/DPU
Bmax=300/DPU
mins=[Bmin,J1min, J2min,J3min]
mins=np.array(mins).astype(int)
maxes=[Bmax,J1max,J2max,J3max]
maxes=np.array(maxes).astype(int)

if mode==joint:
    for i in range(0,len(DXL_IDS)):
        packetHandler.write2ByteTxRx(portHandler, DXL_IDS[i], ADDR_CW_ANGLE_LIMIT, mins[i])
        packetHandler.write2ByteTxRx(portHandler, DXL_IDS[i], ADDR_CCW_ANGLE_LIMIT, maxes[i])
    #J2
else:
    for i in range(0,len(DXL_IDS)):
        packetHandler.write2ByteTxRx(portHandler, DXL_IDS[i], ADDR_CW_ANGLE_LIMIT, 0)
        packetHandler.write2ByteTxRx(portHandler, DXL_IDS[i], ADDR_CCW_ANGLE_LIMIT, 0)
    #J2

speed = 70
direction = "CCW"

if (direction=="CW"):
    speed+=1023

#In degrees



#pos=512


slope = 32
margin = 10

#For Zeroing
# zeroPosDeg=150
# pos=int(zeroPosDeg/DPU)
# motor_pos=[pos,pos,pos,pos]
#Set position
motor_pos=np.array([180,170,170,170])
motor_pos=motor_pos/DPU
motor_pos=motor_pos.astype(int)


for i in range(0,len(DXL_IDS)):
    print(i)
    print(DXL_IDS[i])
    print(motor_pos[i])
    packetHandler.write1ByteTxRx(portHandler, DXL_IDS[i], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write2ByteTxRx(portHandler, DXL_IDS[i], ADDR_MX_CW_COMPLIANCE_MARGIN, margin)
    packetHandler.write2ByteTxRx(portHandler, DXL_IDS[i], ADDR_MX_CCW_COMPLIANCE_MARGIN, margin)
    packetHandler.write1ByteTxRx(portHandler, DXL_IDS[i], ADDR_MX_CW_COMPLIANCE_SLOPE, slope)
    packetHandler.write1ByteTxRx(portHandler, DXL_IDS[i], ADDR_MX_CCW_COMPLIANCE_SLOPE, slope)
    packetHandler.write2ByteTxRx(portHandler, DXL_IDS[i], ADDR_MX_MOVING_SPEED, speed)
    packetHandler.write2ByteTxRx(portHandler, DXL_IDS[i], ADDR_MX_GOAL_POSITION, motor_pos[i])
#time.sleep(1)
print('Write command set. waiting for movement completion...')
movingArr=np.ones(4)
longlist=np.ones((1,4))
while np.sum(movingArr):
    for i in range(0,len(DXL_IDS)):
        movingArr[i]=packetHandler.read1ByteTxRx(portHandler, DXL_IDS[i], ADDR_MOVING)[0]
    
    longlist=np.append(longlist,[movingArr],axis=0)
    print('____')

#x=np.linspace()
plt.plot(longlist)

# dxl_prev_position = packetHandler.read4ByteTxRx(portHandler, DXL_IDS[idx], ADDR_MX_PRESENT_POSITION)

# packetHandler.write1ByteTxRx(portHandler, 1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
# packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_CW_COMPLIANCE_MARGIN, margin)
# packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_CCW_COMPLIANCE_MARGIN, margin)
# packetHandler.write1ByteTxRx(portHandler, 1, ADDR_MX_CW_COMPLIANCE_SLOPE, slope)
# packetHandler.write1ByteTxRx(portHandler, 1, ADDR_MX_CCW_COMPLIANCE_SLOPE, slope)
# packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_MOVING_SPEED, speed)
# packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_GOAL_POSITION, int(190/DPU))

#dxl_present_position = packetHandler.read4ByteTxRx(portHandler, DXL_IDS[idx], ADDR_MX_PRESENT_POSITION)
#print("prev pos: ",dxl_prev_position)
#print("present pos: ",dxl_present_position)


#print('press any key to stop motors')
disTorque=0
if disTorque:
    print('Disabling torque')
    for DXL_ID in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

# Close port
portHandler.closePort()
