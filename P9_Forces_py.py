# -*- coding: utf-8 -*-
"""
Created on Tue Nov 21 10:33:39 2023

@author: carle
"""
import numpy as np
import InverseKinematics as INV
import TrajectoryPlanning as TP
import matplotlib.pyplot as plt


angles=37 #number of points
joints=4
qs=np.zeros((angles,joints))
# First, create positions for the circle
p_0c = [150,0,120]
#Then, holding array for positions
p0 = np.zeros((37,3))
p0 = p0 + p_0c;

R=32 #Radius for circle
phi=np.linspace(0,2*np.pi,angles) #trajectory range
J=np.zeros((angles,6,joints))
for i in range(0,angles): 
    p0[i,1] = p0[i,1] + R*np.cos(phi[i]);
    p0[i,2] = p0[i,2] + R*np.sin(phi[i]);

    #Define x,y,z
    x_s = p0[i,0];
    y_s = p0[i,1];
    z_s = p0[i,2];
    
    qs[i,:]=INV.inverse_kin(np.array([x_s,y_s,z_s]).astype(int),0)
    J[i,:,:],_=TP.Jacobian_stylus(qs[i,:])



#tau=J.F
#F=[Fx Fy Fz tau_xy tau_xz tau_yz]
F=np.array([0,0,-1,0,0,0])
F=F.reshape(np.size(F),1)
tau=np.zeros_like(qs)
for i in range(0,angles):
    tau[i,:]=np.transpose(np.dot(np.transpose(J[i,:,:]),F))
    
plt.plot(phi,tau)