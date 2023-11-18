import numpy as np
import sys

def inverse_kin(X,oritentation,theta_3=1):
    """
    :param X: 3x1 array with coordinates [x,y,z]
    :type X: array
    :param orientation: Orientation of end effector (stylus) in radians
    :type orientation: float
    :param theta_3: [optional] 1 (default) for first option for solving 3rd joint angle. 2 for second option
    :type theta_3: int
    :returns: 4x1 array of joint angles
    :rtype: array
    """


    # Stylus orientation angle 
    varphi = oritentation

    # DH parameters
    a_1 = 50 # [mm]
    a_2 = 93 # [mm]
    a_3 = 93 # [mm]
    a_4 = 50 # [mm]
    
    # Disassembles coordinate vector and removes the height of the 'foot' from the z-value
    x_s = X[0]
    y_s = X[1]
    z_s = X[2]-a_1
    # Creates an 'x_hat' value which is the combined x-y value on the new plane dictated by the q_1 angle
    x_hat = np.sqrt(x_s**2 + y_s**2)

    # finds coordinates of the third joint angle
    p_03x_hat = x_hat - a_4*np.cos(varphi)
    p_03z = z_s - a_4*np.sin(varphi)
    
    # find the distance between joint angle 1 and 3
    d = np.sqrt((p_03z)**2 + p_03x_hat**2)   

    # Determines first joint angle
    q_1= np.arctan2(y_s,x_s)

    # Determines third joint angle (two options)
    if theta_3 == 1:
        #This is option 1 for theta_3
        q_3 = np.pi - np.arccos((a_2**2+a_3**2-d**2)/(2*a_2*a_3))
    elif theta_3 == 2:
        #This is option 2 for theta_3
        q_3 = np.arccos((-a_2**2-a_3**2+d**2)/(2*a_2*a_3))
    else:
        sys.exit('Invalid option for third angle. Must be either 1 or 2')

    # Determines second joint angle
    q_2 = - np.arctan2(p_03x_hat,p_03z) - np.arctan2(a_3*np.sin(q_3),a_2+a_3*np.cos(q_3)) + np.pi/2

    # Determines fourth joint angle
    q_4 = varphi - q_3 - q_2

    # Combines all four joint angles into an array
    q = np.array([q_1, q_2, q_3, q_4])

    return q