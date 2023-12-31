# -*- coding: utf-8 -*-
"""
Connects to robot and primes motors
"""
import dynamixel_sdk as dxl
import time
import numpy as np
import cv2

#Addresses for the dynamixel AX-12A motors
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_CW_COMPLIANCE_MARGIN = 26
ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
ADDR_MX_CW_COMPLIANCE_SLOPE = 28
ADDR_MX_CCW_COMPLIANCE_SLOPE = 29
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_POSITION = 36
ADDR_MX_PUNCH = 48
ADDR_MOVING = 46
PROTOCOL_VERSION = 1.0
# For changing modes
ADDR_CW_ANGLE_LIMIT = 6
ADDR_CCW_ANGLE_LIMIT = 8

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

#Standard values for setting the motors
stdMargin = 10
stdSlope = 32
stdSpeed = 80
stdIDS = [1, 2, 3, 4]
stdPunch=32

# Set angle limits, pre-defined values
# Joint 1: 60-220deg
# Joint 2: 20-280
# Joint 3: 150-190
# Base : 0-300
DPU = 300/1023  # Degrees per unit of position. Setting 300deg to max, maps to 1023
J1min = 60/DPU
J1max = 220/DPU
J2min = 20/DPU
J2max = 280/DPU
J3min = 145/DPU
J3max = 200/DPU
Bmin = 1/DPU
Bmax = 300/DPU


def robotConnect(port, mode="joint", speed=stdSpeed, slope=stdSlope, margin=stdMargin, punch=stdPunch,DXL_IDS=stdIDS):
    '''
    Connects to a series of dynamixel AX-12A robots and initializes them
    inputs: port: connection port to robot 
            mode: "joint" or "wheel" mode
            speed: motor speeds. units/sec where 1 unit is 0.29 degrees
            slope: slope of error correction
            margin: error margin for motor feedback correction
            
    returns:portHandler: dynamixel port handler object
            packetHandler: dynamixel packet handler object
    '''

    DEVICENAME = port
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

    # initializing Angle limits

    mins = [Bmin, J1min, J2min, J3min]
    mins = np.array(mins).astype(int)
    maxes = [Bmax, J1max, J2max, J3max]
    maxes = np.array(maxes).astype(int)

    #Loops through motors and sets the angle limits, 
    #based on whether joint or wheel mode has been defined
    if mode == "joint":
        for i in range(0, len(DXL_IDS)):
            packetHandler.write2ByteTxRx(
                portHandler, DXL_IDS[i], ADDR_CW_ANGLE_LIMIT, mins[i])
            packetHandler.write2ByteTxRx(
                portHandler, DXL_IDS[i], ADDR_CCW_ANGLE_LIMIT, maxes[i])
        # J2
    else:
        for i in range(0, len(DXL_IDS)):
            packetHandler.write2ByteTxRx(
                portHandler, DXL_IDS[i], ADDR_CW_ANGLE_LIMIT, 0)
            packetHandler.write2ByteTxRx(
                portHandler, DXL_IDS[i], ADDR_CCW_ANGLE_LIMIT, 0)
        # J2

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # setting parameters
    print("Setting robot parameters...")
    for i in range(0, len(DXL_IDS)):
        packetHandler.write1ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write2ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_CW_COMPLIANCE_MARGIN, margin)
        packetHandler.write2ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_CCW_COMPLIANCE_MARGIN, margin)
        packetHandler.write1ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_CW_COMPLIANCE_SLOPE, slope)
        packetHandler.write1ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_CCW_COMPLIANCE_SLOPE, slope)
        packetHandler.write2ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_MOVING_SPEED, speed)
        packetHandler.write2ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_PUNCH, punch)
    return portHandler, packetHandler


def robotUpdateParams(portHandler, packetHandler, DXL_IDS=stdIDS, speed=stdSpeed, slope=stdSlope, margin=stdMargin):
    '''
    Updates the given parameters for specific id's of the motors.
    inputs: portHandler: dynamixel portHandler object 
            packetHanler: dynamixel packetHandler object
            DXL_IDS: the id's of the motors wished updated
            speed: motor speeds. units/sec where 1 unit is 0.29 degrees
            slope: slope of error correction
            margin: error margin for motor feedback correction
    '''
    print("updating robot parameters")
    for i in range(0, len(DXL_IDS)):
        packetHandler.write1ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write2ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_CW_COMPLIANCE_MARGIN, margin)
        packetHandler.write2ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_CCW_COMPLIANCE_MARGIN, margin)
        packetHandler.write1ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_CW_COMPLIANCE_SLOPE, slope)
        packetHandler.write1ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_CCW_COMPLIANCE_SLOPE, slope)
        packetHandler.write2ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_MOVING_SPEED, speed)


def robotMove(portHandler, packetHandler, pos, DXL_IDS=stdIDS):
    '''
    Moves the robot to a given set of position angles, defined by the 'pos' param
    inputs: portHandler: dynamixel portHandler object 
            packetHanler: dynamixel packetHandler object
            pos: a len(pos)=len(DXL_IDS) np array of angles to set the motors
            DXL_IDS: the id's of the motors wished updated
    '''
    for i in range(0, len(DXL_IDS)):
        #Loops through every motor and sets the angles
        motor_pos = np.array(pos)/DPU
        motor_pos = motor_pos.astype(int)
        packetHandler.write2ByteTxRx(
            portHandler, DXL_IDS[i], ADDR_MX_GOAL_POSITION, motor_pos[i])
    movingArr = np.ones(4)
    
    #While moving, check when it is done and only return from function when 
    #movement is excecuted
    
    print("executing movement...")
    while np.sum(movingArr):
        for i in range(0, len(DXL_IDS)):
            movingArr[i] = packetHandler.read1ByteTxRx(
                portHandler, DXL_IDS[i], ADDR_MOVING)[0]
    print("Finished movement!")


def robotTerminate(portHandler, packetHandler, DXL_IDS=stdIDS, disTorque=0):
    '''
    Disconnects from the robot. Closes port and disables torque.
    inputs: portHandler: dynamixel portHandler object 
            packetHanler: dynamixel packetHandler object
            pos: a len(pos)=len(DXL_IDS) np array of angles to set the motors
            DXL_IDS: the id's of the motors wished updated
            disTorque: int, decides whether torque should be disabled when 
                       disconnecting
    '''
    if disTorque:
        print('Disabling torque')
        for DXL_ID in DXL_IDS:
            packetHandler.write1ByteTxRx(
                portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

    # Close port
    portHandler.closePort()
