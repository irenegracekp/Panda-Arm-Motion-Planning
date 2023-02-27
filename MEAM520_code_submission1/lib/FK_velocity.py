import numpy as np 
#from lib.calcJacobian import calcJacobian

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    J = np.zeros((6, 7))

    ## STUDENT CODE GOES HERE
    
    #joint angles
    q1 = q_in[0]
    q2 = q_in[1]
    q3 = q_in[2]
    q4 = q_in[3]
    q5 = q_in[4]
    q6 = q_in[5]
    q7 = q_in[6]

    #Vel_jacob
    jv = np.array([[(0.088*((-1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1))*np.cos(q4) - 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4))*np.cos(q5) + 0.088*(1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) - 1.0*np.cos(q1)*np.cos(q3))*np.sin(q5))*np.cos(q6) + (0.21*((-1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1))*np.cos(q4) - 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4))*np.cos(q5) + 0.21*(1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) - 1.0*np.cos(q1)*np.cos(q3))*np.sin(q5))*np.sin(q6) + (-0.21*(1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.sin(q4) + 0.21*np.sin(q1)*np.sin(q2)*np.cos(q4))*np.cos(q6) + (0.088*(1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.sin(q4) - 0.088*np.sin(q1)*np.sin(q2)*np.cos(q4))*np.sin(q6) + (0.0825*np.sin(q1)*np.cos(q2)*np.cos(q3) + 0.0825*np.sin(q3)*np.cos(q1))*np.cos(q4) + (0.384*np.sin(q1)*np.cos(q2)*np.cos(q3) + 0.384*np.sin(q3)*np.cos(q1))*np.sin(q4) + 0.0825*np.sin(q1)*np.sin(q2)*np.sin(q4) - 0.384*np.sin(q1)*np.sin(q2)*np.cos(q4) - 0.316*np.sin(q1)*np.sin(q2) - 0.0825*np.sin(q1)*np.cos(q2)*np.cos(q3) - 0.0825*np.sin(q3)*np.cos(q1)]
        ,[(0.088*(-1.0*np.sin(q2)*np.cos(q1)*np.cos(q3)*np.cos(q4) + 1.0*np.sin(q4)*np.cos(q1)*np.cos(q2))*np.cos(q5) + 0.088*np.sin(q2)*np.sin(q3)*np.sin(q5)*np.cos(q1))*np.cos(q6) + (0.21*(-1.0*np.sin(q2)*np.cos(q1)*np.cos(q3)*np.cos(q4) + 1.0*np.sin(q4)*np.cos(q1)*np.cos(q2))*np.cos(q5) + 0.21*np.sin(q2)*np.sin(q3)*np.sin(q5)*np.cos(q1))*np.sin(q6) + (-0.21*np.sin(q2)*np.sin(q4)*np.cos(q1)*np.cos(q3) - 0.21*np.cos(q1)*np.cos(q2)*np.cos(q4))*np.cos(q6) + (0.088*np.sin(q2)*np.sin(q4)*np.cos(q1)*np.cos(q3) + 0.088*np.cos(q1)*np.cos(q2)*np.cos(q4))*np.sin(q6) + 0.384*np.sin(q2)*np.sin(q4)*np.cos(q1)*np.cos(q3) + 0.0825*np.sin(q2)*np.cos(q1)*np.cos(q3)*np.cos(q4) - 0.0825*np.sin(q2)*np.cos(q1)*np.cos(q3) - 0.0825*np.sin(q4)*np.cos(q1)*np.cos(q2) + 0.384*np.cos(q1)*np.cos(q2)*np.cos(q4) + 0.316*np.cos(q1)*np.cos(q2)]
        ,[(0.088*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q5) + 0.088*(-1.0*np.sin(q1)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.cos(q4)*np.cos(q5))*np.cos(q6) + (0.21*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q5) + 0.21*(-1.0*np.sin(q1)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.cos(q4)*np.cos(q5))*np.sin(q6) + (0.0825*np.sin(q1)*np.cos(q3) + 0.0825*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.cos(q4) + (0.384*np.sin(q1)*np.cos(q3) + 0.384*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.sin(q4) + 0.088*(1.0*np.sin(q1)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.sin(q4)*np.sin(q6) - 0.21*(1.0*np.sin(q1)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.sin(q4)*np.cos(q6) - 0.0825*np.sin(q1)*np.cos(q3) - 0.0825*np.sin(q3)*np.cos(q1)*np.cos(q2)]
        ,[0.21*(-(-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q4) + 1.0*np.sin(q2)*np.cos(q1)*np.cos(q4))*np.sin(q6)*np.cos(q5) + 0.088*(-(-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q4) + 1.0*np.sin(q2)*np.cos(q1)*np.cos(q4))*np.cos(q5)*np.cos(q6) + (-0.21*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) + 0.21*np.sin(q2)*np.sin(q4)*np.cos(q1))*np.cos(q6) + (0.088*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) - 0.088*np.sin(q2)*np.sin(q4)*np.cos(q1))*np.sin(q6) - (0.0825*np.sin(q1)*np.sin(q3) - 0.0825*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q4) + (0.384*np.sin(q1)*np.sin(q3) - 0.384*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) - 0.384*np.sin(q2)*np.sin(q4)*np.cos(q1) - 0.0825*np.sin(q2)*np.cos(q1)*np.cos(q4)]
        ,[(-0.21*((-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) + 1.0*np.sin(q2)*np.sin(q4)*np.cos(q1))*np.sin(q5) + 0.21*(-1.0*np.sin(q1)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.cos(q5))*np.sin(q6) + (-0.088*((-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) + 1.0*np.sin(q2)*np.sin(q4)*np.cos(q1))*np.sin(q5) + 0.088*(-1.0*np.sin(q1)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.cos(q5))*np.cos(q6)]
        ,[-(0.088*((-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) + 1.0*np.sin(q2)*np.sin(q4)*np.cos(q1))*np.cos(q5) + 0.088*(-1.0*np.sin(q1)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.sin(q5))*np.sin(q6) + (0.21*((-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) + 1.0*np.sin(q2)*np.sin(q4)*np.cos(q1))*np.cos(q5) + 0.21*(-1.0*np.sin(q1)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.sin(q5))*np.cos(q6) + (-0.088*(-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q4) + 0.088*np.sin(q2)*np.cos(q1)*np.cos(q4))*np.cos(q6) - (0.21*(-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q4) - 0.21*np.sin(q2)*np.cos(q1)*np.cos(q4))*np.sin(q6)]
        ,[0]
        ,[(0.088*((-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) + 1.0*np.sin(q2)*np.sin(q4)*np.cos(q1))*np.cos(q5) + 0.088*(-1.0*np.sin(q1)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.sin(q5))*np.cos(q6) + (0.21*((-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) + 1.0*np.sin(q2)*np.sin(q4)*np.cos(q1))*np.cos(q5) + 0.21*(-1.0*np.sin(q1)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.sin(q5))*np.sin(q6) + (-0.21*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q4) - 0.21*np.sin(q2)*np.cos(q1)*np.cos(q4))*np.cos(q6) + (0.088*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q4) + 0.088*np.sin(q2)*np.cos(q1)*np.cos(q4))*np.sin(q6) + (0.0825*np.sin(q1)*np.sin(q3) - 0.0825*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) + (0.384*np.sin(q1)*np.sin(q3) - 0.384*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q4) - 0.0825*np.sin(q1)*np.sin(q3) - 0.0825*np.sin(q2)*np.sin(q4)*np.cos(q1) + 0.384*np.sin(q2)*np.cos(q1)*np.cos(q4) + 0.316*np.sin(q2)*np.cos(q1) + 0.0825*np.cos(q1)*np.cos(q2)*np.cos(q3)]
        ,[(0.088*(-1.0*np.sin(q1)*np.sin(q2)*np.cos(q3)*np.cos(q4) + 1.0*np.sin(q1)*np.sin(q4)*np.cos(q2))*np.cos(q5) + 0.088*np.sin(q1)*np.sin(q2)*np.sin(q3)*np.sin(q5))*np.cos(q6) + (0.21*(-1.0*np.sin(q1)*np.sin(q2)*np.cos(q3)*np.cos(q4) + 1.0*np.sin(q1)*np.sin(q4)*np.cos(q2))*np.cos(q5) + 0.21*np.sin(q1)*np.sin(q2)*np.sin(q3)*np.sin(q5))*np.sin(q6) + (-0.21*np.sin(q1)*np.sin(q2)*np.sin(q4)*np.cos(q3) - 0.21*np.sin(q1)*np.cos(q2)*np.cos(q4))*np.cos(q6) + (0.088*np.sin(q1)*np.sin(q2)*np.sin(q4)*np.cos(q3) + 0.088*np.sin(q1)*np.cos(q2)*np.cos(q4))*np.sin(q6) + 0.384*np.sin(q1)*np.sin(q2)*np.sin(q4)*np.cos(q3) + 0.0825*np.sin(q1)*np.sin(q2)*np.cos(q3)*np.cos(q4) - 0.0825*np.sin(q1)*np.sin(q2)*np.cos(q3) - 0.0825*np.sin(q1)*np.sin(q4)*np.cos(q2) + 0.384*np.sin(q1)*np.cos(q2)*np.cos(q4) + 0.316*np.sin(q1)*np.cos(q2)]
        ,[(0.088*(-1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) + 1.0*np.cos(q1)*np.cos(q3))*np.cos(q4)*np.cos(q5) + 0.088*(-1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1))*np.sin(q5))*np.cos(q6) + (0.21*(-1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) + 1.0*np.cos(q1)*np.cos(q3))*np.cos(q4)*np.cos(q5) + 0.21*(-1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1))*np.sin(q5))*np.sin(q6) + (0.0825*np.sin(q1)*np.sin(q3)*np.cos(q2) - 0.0825*np.cos(q1)*np.cos(q3))*np.cos(q4) + (0.384*np.sin(q1)*np.sin(q3)*np.cos(q2) - 0.384*np.cos(q1)*np.cos(q3))*np.sin(q4) + 0.088*(1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) - 1.0*np.cos(q1)*np.cos(q3))*np.sin(q4)*np.sin(q6) - 0.21*(1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) - 1.0*np.cos(q1)*np.cos(q3))*np.sin(q4)*np.cos(q6) - 0.0825*np.sin(q1)*np.sin(q3)*np.cos(q2) + 0.0825*np.cos(q1)*np.cos(q3)]
        ,[(-0.21*(-1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1))*np.cos(q4) + 0.21*np.sin(q1)*np.sin(q2)*np.sin(q4))*np.cos(q6) + (0.088*(-1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1))*np.cos(q4) - 0.088*np.sin(q1)*np.sin(q2)*np.sin(q4))*np.sin(q6) + 0.21*(-(1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.sin(q4) + 1.0*np.sin(q1)*np.sin(q2)*np.cos(q4))*np.sin(q6)*np.cos(q5) + 0.088*(-(1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.sin(q4) + 1.0*np.sin(q1)*np.sin(q2)*np.cos(q4))*np.cos(q5)*np.cos(q6) + (-0.384*np.sin(q1)*np.cos(q2)*np.cos(q3) - 0.384*np.sin(q3)*np.cos(q1))*np.cos(q4) - (-0.0825*np.sin(q1)*np.cos(q2)*np.cos(q3) - 0.0825*np.sin(q3)*np.cos(q1))*np.sin(q4) - 0.384*np.sin(q1)*np.sin(q2)*np.sin(q4) - 0.0825*np.sin(q1)*np.sin(q2)*np.cos(q4)]
        ,[(-0.21*((1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.cos(q4) + 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4))*np.sin(q5) + 0.21*(-1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) + 1.0*np.cos(q1)*np.cos(q3))*np.cos(q5))*np.sin(q6) + (-0.088*((1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.cos(q4) + 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4))*np.sin(q5) + 0.088*(-1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) + 1.0*np.cos(q1)*np.cos(q3))*np.cos(q5))*np.cos(q6)]
        ,[-(0.088*((1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.cos(q4) + 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4))*np.cos(q5) + 0.088*(-1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) + 1.0*np.cos(q1)*np.cos(q3))*np.sin(q5))*np.sin(q6) + (0.21*((1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.cos(q4) + 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4))*np.cos(q5) + 0.21*(-1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) + 1.0*np.cos(q1)*np.cos(q3))*np.sin(q5))*np.cos(q6) + (-0.088*(1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.sin(q4) + 0.088*np.sin(q1)*np.sin(q2)*np.cos(q4))*np.cos(q6) - (0.21*(1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.sin(q4) - 0.21*np.sin(q1)*np.sin(q2)*np.cos(q4))*np.sin(q6)]
        ,[0]
        ,[0]
        ,[(0.088*(-1.0*np.sin(q2)*np.sin(q4) - 1.0*np.cos(q2)*np.cos(q3)*np.cos(q4))*np.cos(q5) + 0.088*np.sin(q3)*np.sin(q5)*np.cos(q2))*np.cos(q6) + (0.21*(-1.0*np.sin(q2)*np.sin(q4) - 1.0*np.cos(q2)*np.cos(q3)*np.cos(q4))*np.cos(q5) + 0.21*np.sin(q3)*np.sin(q5)*np.cos(q2))*np.sin(q6) + (-0.088*np.sin(q2)*np.cos(q4) + 0.088*np.sin(q4)*np.cos(q2)*np.cos(q3))*np.sin(q6) + (0.21*np.sin(q2)*np.cos(q4) - 0.21*np.sin(q4)*np.cos(q2)*np.cos(q3))*np.cos(q6) + 0.0825*np.sin(q2)*np.sin(q4) - 0.384*np.sin(q2)*np.cos(q4) - 0.316*np.sin(q2) + 0.384*np.sin(q4)*np.cos(q2)*np.cos(q3) + 0.0825*np.cos(q2)*np.cos(q3)*np.cos(q4) - 0.0825*np.cos(q2)*np.cos(q3)]
        ,[(0.088*np.sin(q2)*np.sin(q3)*np.cos(q4)*np.cos(q5) + 0.088*np.sin(q2)*np.sin(q5)*np.cos(q3))*np.cos(q6) + (0.21*np.sin(q2)*np.sin(q3)*np.cos(q4)*np.cos(q5) + 0.21*np.sin(q2)*np.sin(q5)*np.cos(q3))*np.sin(q6) - 0.088*np.sin(q2)*np.sin(q3)*np.sin(q4)*np.sin(q6) + 0.21*np.sin(q2)*np.sin(q3)*np.sin(q4)*np.cos(q6) - 0.384*np.sin(q2)*np.sin(q3)*np.sin(q4) - 0.0825*np.sin(q2)*np.sin(q3)*np.cos(q4) + 0.0825*np.sin(q2)*np.sin(q3)]
        ,[0.21*(1.0*np.sin(q2)*np.sin(q4)*np.cos(q3) + 1.0*np.cos(q2)*np.cos(q4))*np.sin(q6)*np.cos(q5) + 0.088*(1.0*np.sin(q2)*np.sin(q4)*np.cos(q3) + 1.0*np.cos(q2)*np.cos(q4))*np.cos(q5)*np.cos(q6) + (-0.21*np.sin(q2)*np.cos(q3)*np.cos(q4) + 0.21*np.sin(q4)*np.cos(q2))*np.cos(q6) + (0.088*np.sin(q2)*np.cos(q3)*np.cos(q4) - 0.088*np.sin(q4)*np.cos(q2))*np.sin(q6) - 0.0825*np.sin(q2)*np.sin(q4)*np.cos(q3) + 0.384*np.sin(q2)*np.cos(q3)*np.cos(q4) - 0.384*np.sin(q4)*np.cos(q2) - 0.0825*np.cos(q2)*np.cos(q4)]
        ,[(-0.21*(-1.0*np.sin(q2)*np.cos(q3)*np.cos(q4) + 1.0*np.sin(q4)*np.cos(q2))*np.sin(q5) + 0.21*np.sin(q2)*np.sin(q3)*np.cos(q5))*np.sin(q6) + (-0.088*(-1.0*np.sin(q2)*np.cos(q3)*np.cos(q4) + 1.0*np.sin(q4)*np.cos(q2))*np.sin(q5) + 0.088*np.sin(q2)*np.sin(q3)*np.cos(q5))*np.cos(q6)]
        ,[-(0.088*(-1.0*np.sin(q2)*np.cos(q3)*np.cos(q4) + 1.0*np.sin(q4)*np.cos(q2))*np.cos(q5) + 0.088*np.sin(q2)*np.sin(q3)*np.sin(q5))*np.sin(q6) + (0.21*(-1.0*np.sin(q2)*np.cos(q3)*np.cos(q4) + 1.0*np.sin(q4)*np.cos(q2))*np.cos(q5) + 0.21*np.sin(q2)*np.sin(q3)*np.sin(q5))*np.cos(q6) - (-0.21*np.sin(q2)*np.sin(q4)*np.cos(q3) - 0.21*np.cos(q2)*np.cos(q4))*np.sin(q6) + (0.088*np.sin(q2)*np.sin(q4)*np.cos(q3) + 0.088*np.cos(q2)*np.cos(q4))*np.cos(q6)]
        ,[0]]).reshape(3,7)

        
    #angular vel jacob
    jw = np.array([[-1.0*np.sin(q1)],
        [1.0*np.cos(q1)],
        [0],
        [1.0*np.sin(q2)*np.cos(q1)],
        [1.0*np.sin(q1)*np.sin(q2)],
        [1.0*np.cos(q2)],
        [1.0*np.sin(q1)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2)],
        [1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) - 1.0*np.cos(q1)*np.cos(q3)],
        [-1.0*np.sin(q2)*np.sin(q3)],
        [-(-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q4) + 1.0*np.sin(q2)*np.cos(q1)*np.cos(q4)],
        [-(1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.sin(q4) + 1.0*np.sin(q1)*np.sin(q2)*np.cos(q4)],
        [1.0*np.sin(q2)*np.sin(q4)*np.cos(q3) + 1.0*np.cos(q2)*np.cos(q4)],
        [((-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) + 1.0*np.sin(q2)*np.sin(q4)*np.cos(q1))*np.sin(q5) - (-1.0*np.sin(q1)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.cos(q5)],
        [((1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.cos(q4) + 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4))*np.sin(q5) - (-1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) + 1.0*np.cos(q1)*np.cos(q3))*np.cos(q5)],
        [(-1.0*np.sin(q2)*np.cos(q3)*np.cos(q4) + 1.0*np.sin(q4)*np.cos(q2))*np.sin(q5) - 1.0*np.sin(q2)*np.sin(q3)*np.cos(q5)],
        [(((-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) + 1.0*np.sin(q2)*np.sin(q4)*np.cos(q1))*np.cos(q5) + (-1.0*np.sin(q1)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.sin(q5))*np.sin(q6) - (-(-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q4) + 1.0*np.sin(q2)*np.cos(q1)*np.cos(q4))*np.cos(q6)],
        [(((1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.cos(q4) + 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4))*np.cos(q5) + (-1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) + 1.0*np.cos(q1)*np.cos(q3))*np.sin(q5))*np.sin(q6) - (-(1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.sin(q4) + 1.0*np.sin(q1)*np.sin(q2)*np.cos(q4))*np.cos(q6)],
        [((-1.0*np.sin(q2)*np.cos(q3)*np.cos(q4) + 1.0*np.sin(q4)*np.cos(q2))*np.cos(q5) + 1.0*np.sin(q2)*np.sin(q3)*np.sin(q5))*np.sin(q6) - (1.0*np.sin(q2)*np.sin(q4)*np.cos(q3) + 1.0*np.cos(q2)*np.cos(q4))*np.cos(q6)],
        [(((-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.cos(q4) + 1.0*np.sin(q2)*np.sin(q4)*np.cos(q1))*np.cos(q5) + (-1.0*np.sin(q1)*np.cos(q3) - 1.0*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.sin(q5))*np.sin(q6) - (-(-1.0*np.sin(q1)*np.sin(q3) + 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q4) + 1.0*np.sin(q2)*np.cos(q1)*np.cos(q4))*np.cos(q6)],
        [(((1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.cos(q4) + 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4))*np.cos(q5) + (-1.0*np.sin(q1)*np.sin(q3)*np.cos(q2) + 1.0*np.cos(q1)*np.cos(q3))*np.sin(q5))*np.sin(q6) - (-(1.0*np.sin(q1)*np.cos(q2)*np.cos(q3) + 1.0*np.sin(q3)*np.cos(q1))*np.sin(q4) + 1.0*np.sin(q1)*np.sin(q2)*np.cos(q4))*np.cos(q6)],
        [((-1.0*np.sin(q2)*np.cos(q3)*np.cos(q4) + 1.0*np.sin(q4)*np.cos(q2))*np.cos(q5) + 1.0*np.sin(q2)*np.sin(q3)*np.sin(q5))*np.sin(q6) - (1.0*np.sin(q2)*np.sin(q4)*np.cos(q3) + 1.0*np.cos(q2)*np.cos(q4))*np.cos(q6)]]).reshape(3,7)

 
    J[0:3,:] = jv
    J[3:6,:] = jw

    return J


def FK_velocity(q_in, dq):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param dq: 1 x 7 vector corresponding to the joint velocities.
    :return:
    velocity - 6 x 1 vector corresponding to the end effector velocities.    
    """

    ## STUDENT CODE GOES HERE

    velocity = np.zeros((6, 1))

    j = calcJacobian(q_in)
    dq = np.array(dq).reshape(7,1)
    
    velocity[0:3] = np.dot(j[0:3],dq)
    velocity[3:7] = np.dot(j[3:7],dq)

    return velocity
