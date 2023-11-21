# -*- coding: utf-8 -*-
"""

"""

import numpy as np
import InverseKinematics as INV
import TrajectoryPlanning as TP
import matplotlib.pyplot as plt
import robotConnect as RC
from msvcrt import getch

# Establishes the beginning and end point of the trajectory, as well as the center of the circle the two points are located on
circle_center = np.array([150, 0, 120])
pos_in = np.array([150, 32, 120])
pos_out = np.array([150, 0, 152])

# Sets a beginning and end velocity of the end effector in global coordinates
v_in = [0, 0, 0]
v_out = [0, -27, 0]

# Estimates the beginning and end angles (Uses the inverse kinematics function)
q_in = INV.inverse_kin(pos_in,0)
q_out = INV.inverse_kin(pos_out,0)

# Estimates the Jacobian for the beginning and end point
J_in, J_in_inv = TP.Jacobian_stylus(q_in)
J_out, J_out_inv = TP.Jacobian_stylus(q_out)

# Estimates the global angular velocity of the stylus. This is only valid due to the circle path
r_in = pos_in - circle_center
w_in = np.cross(r_in,v_in)/np.linalg.norm(r_in)**2
r_out = pos_out - circle_center
w_out = np.cross(r_out,v_out)/np.linalg.norm(r_out)**2

# Assembles the velocity and angular velocity in a vector
xi_in = np.zeros((6))
xi_in[0:3] = v_in
xi_in[3:6] = w_in
xi_out = np.zeros(6)
xi_out[0:3] = v_out
xi_out[3:6] = w_out

# Estimates the joint angle velocities from the xi vector and inverse Jacobian
q_dot_in = J_in_inv@xi_in
q_dot_out = J_out_inv@xi_out

# Sets the beginning and end acceleration to 0
a_in = [0, 0, 0, 0]
a_out = [0, 0, 0, 0]

# Specify beginning and end time
T = [0, 2]

# Estimates the polynomial parameters for the joint trajectories
qt1, qt2, qt3, qt4 = TP.trajectory(q_in, q_out, q_dot_in, q_dot_out, a_in, a_out, T)

# Creates 100 points over the two seconds
ts = np.linspace(0,2,10)

# Estimates 100 joint angles for the path
q1s = qt1[5]*ts**5 + qt1[4]*ts**4 + qt1[3]*ts**3 + qt1[2]*ts**2 + qt1[1]*ts + qt1[0]
q2s = qt2[5]*ts**5 + qt2[4]*ts**4 + qt2[3]*ts**3 + qt2[2]*ts**2 + qt2[1]*ts + qt2[0]
q3s = qt3[5]*ts**5 + qt3[4]*ts**4 + qt3[3]*ts**3 + qt3[2]*ts**2 + qt3[1]*ts + qt3[0]
q4s = qt4[5]*ts**5 + qt4[4]*ts**4 + qt4[3]*ts**3 + qt4[2]*ts**2 + qt4[1]*ts + qt4[0]


portHandler, packetHandler = RC.robotConnect("COM4",mode="joint",speed=70)
zeroPos = [150, 150, 150, 150]

pos = zeroPos
RC.robotMove(portHandler, packetHandler, pos)
i=0
q_deg=np.array([q1s,q2s,q3s,q4s])*180/np.pi
while True:
    i+=1
    if i>(np.size(q1s)-1):
        break
    pos=zeroPos+q_deg[:,i]
    RC.robotMove(portHandler,packetHandler,pos)
    # Break the loop if 'q' key is pressed
    # pressedKey = cv2.waitKey(1) & 0xFF
    # if pressedKey == ord('q'):
    #     break
    # elif pressedKey == ord('z'):
    #     robotMove(portHandler, packetHandler, zeroPos)
    # elif pressedKey == ord('x'):
    #     robotMove(portHandler, packetHandler, tablePos)
    
# Release the camera and close all OpenCV windows

RC.robotTerminate(portHandler, packetHandler)
# # def followTrajectory(q):
    



#amp=30
#freq=0.5
#t = np.arange(0, 10, 0.01)
#follow this trajectory
#speeds = amp * np.cos(2 * np.pi * freq * t)
#desired_positions = amp*np.sin(2*np.pi * freq * t) / (2*np.pi*freq)