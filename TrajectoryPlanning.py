import numpy as np

def trajectory(q_in, q_out,v_in,v_out,a_in,a_out,T):
    """
    :param q_in: 4x1 with initial angles
    :type q_in: array
    :param v_in: 4x1 with initial angle velocities
    :type v_in: array
    :param a_in: 4x1 with initial angle accelerations
    :type a_in: array
    :param q_out: 4x1 with final angles
    :type q_out: array
    :param v_out: 4x1 with final angle velocities
    :type v_out: array
    :param a_out: 4x1 with final angle accelerations
    :type a_out: array
    :param T: 2x1 array of initial and final time
    :type T: array
    :returns: 4 6x1 arrays containing the polynomial parameters for the angle trajectories
    """    

    # Determine intial and final time
    t_in = T[0]
    t_out = T[1]

    # DH parameters
    a_1 = 50 # [mm]
    a_2 = 93 # [mm]
    a_3 = 93 # [mm]
    a_4 = 50 # [mm]

    # Sets up the matrix to solve for the trajectory polynomial parameters
    Big_matrix = [[1, t_in, t_in**2, t_in**3, t_in**4, t_in**5],
                  [0, 1, 2*t_in, 3*t_in**2, 4*t_in**3, 5*t_in**4],
                  [0, 0, 2, 6*t_in, 12*t_in**2, 20*t_in**3],
                  [1, t_out, t_out**2, t_out**3, t_out**4, t_out**5],
                  [0, 1, 2*t_out, 3*t_out**2, 4*t_out**3, 5*t_out**4],
                  [0, 0, 2, 6*t_out, 12*t_out**2, 20*t_out**3]]
    
    # solution containing known initial and final angle, velocity and acceleration
    sol_1 = [[q_in[0]], [v_in[0]], [a_in[0]], [q_out[0]], [v_out[0]], [a_out[0]]]
    sol_2 = [[q_in[1]], [v_in[1]], [a_in[1]], [q_out[1]], [v_out[1]], [a_out[1]]]
    sol_3 = [[q_in[2]], [v_in[2]], [a_in[2]], [q_out[2]], [v_out[2]], [a_out[2]]]
    sol_4 = [[q_in[3]], [v_in[3]], [a_in[3]], [q_out[3]], [v_out[3]], [a_out[3]]]

    # solving for the polynomial parameters
    A1 = np.linalg.solve(Big_matrix,sol_1)
    A2 = np.linalg.solve(Big_matrix,sol_2)
    A3 = np.linalg.solve(Big_matrix,sol_3)
    A4 = np.linalg.solve(Big_matrix,sol_4)

    return A1, A2, A3, A4

def Transformation(q):
    """
    :param q: 4x1 array with joint angles
    :type X: array
    :returns: 5 4x4 local transformation matrices
    """

    # DH parameters
    a_1 = 50 # [mm]
    a_2 = 93 # [mm]
    a_3 = 93 # [mm]
    a_4 = 50 # [mm]

    # joint angles
    t_1 = q[0]
    t_2 = q[1]
    t_3 = q[2]
    t_4 = q[3]

    # sets up the five transformation matrices
    T01 = np.array([[np.cos(t_1), 0, np.sin(t_1), 0],
           [np.sin(t_1), 0, -np.cos(t_1), 0],
           [0, 1, 0, 50],
           [0, 0, 0, 1]])
    T12 = np.array([[np.cos(t_2), -np.sin(t_2), 0, a_2*np.cos(t_2)],
           [np.sin(t_2), np.cos(t_2), 0, a_2*np.sin(t_2)],
           [0, 0, 1, 0],
           [0, 0, 0, 1]])
    T23 = np.array([[np.cos(t_3), -np.sin(t_3), 0, a_3*np.cos(t_3)],
           [np.sin(t_3), np.cos(t_3), 0, a_3*np.sin(t_3)],
           [0, 0, 1, 0],
           [0, 0, 0, 1]])
    T34 = np.array([[np.cos(t_4), -np.sin(t_4), 0, a_4*np.cos(t_4)],
           [np.sin(t_4), np.cos(t_4), 0, a_4*np.sin(t_4)],
           [0, 0, 1, 0],
           [0, 0, 0, 1]])
    T45 = np.array([[-0.3162272401, -0.9486834734, 0, -14.99997505],
           [0.9486834734, -0.3162272401, 0,  45.00000831],
           [0, 0, 1, 0],
           [0, 0, 0, 1]])
    
    return T01, T12, T23, T34, T45

def Jacobian_stylus(q):
    """
    :param q: 4x1 array with joint angles
    :type X: array
    :returns: 2 6x4 matrices. The first is the Jacobian manipulator and the second is the pseudo inverse of that jacobian
    """
    # Calculates transformation matrices
    T01_in, T12_in, T23_in, T34_in, T45_in = Transformation(q)
    
    # Estimates global transformation matrices
    T02_in = T01_in@T12_in
    T03_in = T02_in@T23_in
    T04_in = T03_in@T34_in
 
    # Calculates z's for the jacobian
    z_0_in = [0, 0, 1]
    z_1_in = T01_in[0:3,2]
    z_2_in = T02_in[0:3,2]
    z_3_in = T03_in[0:3,2]
    
    # Calculates o's for the Jacobian
    o_0_in = [0, 0, 0]
    o_1_in = T01_in[0:3,3]
    o_2_in = T02_in[0:3,3]
    o_3_in = T03_in[0:3,3]
    o_4_in = T04_in[0:3,3]
    
    # Assembles the Jacobian
    J_stylus_in = np.zeros((6,4))
    J_stylus_in[0:3,0] = np.cross(z_0_in,(o_4_in-o_0_in))
    J_stylus_in[3:6,0] = z_0_in
    J_stylus_in[0:3,1] = np.cross(z_1_in,(o_4_in-o_1_in))
    J_stylus_in[3:6,1] = z_1_in
    J_stylus_in[0:3,2] = np.cross(z_2_in,(o_4_in-o_2_in))
    J_stylus_in[3:6,2] = z_2_in
    J_stylus_in[0:3,3] = np.cross(z_3_in,(o_4_in-o_3_in))
    J_stylus_in[3:6,3] = z_3_in

    # Calculates the pseudo inverse of the Jacobian
    J_inv_in = np.linalg.pinv(J_stylus_in)

    return J_stylus_in, J_inv_in
