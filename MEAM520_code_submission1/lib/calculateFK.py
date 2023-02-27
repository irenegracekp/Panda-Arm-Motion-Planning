import numpy as np
from math import pi

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout
        #self.theta1 = theta1
        #self.theta2 = theta2
        #self.theta3 = theta3
        #self.theta4 = theta4
        #self.theta5 = theta5
        #self.theta6 = theta6
        #self.theta7 = theta7
        #self.theta8 = theta8

        pass

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here
        q = q.reshape((7,))

        jointPositions = np.zeros((8,3))
        T0e = np.identity(4)

        # l1 = np.array([0, 0, 0.141, 0])
        # l2 = np.array([0, np.pi/2, 0.192, theta1])
        # l3 = np.array([0, -np.pi/2, 0, theta2])
        # l4 = np.array([0.0825, np.pi/2, 0.316, theta3])
        # l5 = np.array([0.0825, np.pi/2, 0, theta4 + np.pi/2])
        # l6 = np.array([0, np.pi/2, 0.384, theta5])
        # l7 = np.array([0.088, -np.pi/2, 0, theta6 + ni.pi/2])
        # l8 = np.array([0, 0, 0.21, theta7])

        a = np.array([0,0,0, 0.0825, 0.0825, 0, 0.088,0])
        alpha = np.array([0, -pi/2, pi/2, pi/2, pi/2, -pi/2, pi/2, 0])
        d = np.array([0.141, 0.192, 0, 0.316, 0, 0.384, 0, 0.21])
        theta = np.array([0, q[0], q[1], q[2], q[3]+pi, q[4],-pi/2-pi/2+q[5], q[6]-pi/4])
        #A = np.array([])

        def TransformationMatrix(a, alpha, d, theta, i):
            A = np.array([np.cos(theta[i]), -np.sin(theta[i])*np.cos(alpha[i]), np.sin(theta[i])*np.sin(alpha[i]), a[i]*np.cos(theta[i]), np.sin(theta[i]), np.cos(theta[i])*np.cos(alpha[i]),-np.cos(theta[i])*np.sin(alpha[i]), a[i]*np.sin(theta[i]), 0, np.sin(alpha[i]), np.cos(alpha[i]), d[i],0,0,0,1])
            A = A.reshape(4,4)
            #print(A)
            return A

        A0 = TransformationMatrix(a, alpha, d, theta, 0)
        A1 = TransformationMatrix(a, alpha, d, theta, 1)
        A2 = TransformationMatrix(a, alpha, d, theta, 2)
        A3 = TransformationMatrix(a, alpha, d, theta, 3)
        A4 = TransformationMatrix(a, alpha, d, theta, 4)
        A5 = TransformationMatrix(a, alpha, d, theta, 5)
        A6 = TransformationMatrix(a, alpha, d, theta, 6)
        A7 = TransformationMatrix(a, alpha, d, theta, 7)
        T = [A0,A1,A2,A3,A4,A5,A6,A7]
        

        for j in range(8):
            T0e = np.matmul(T0e, T[j])
            N = T0e
            #print(T0e, j)

            if j == 2:
                L = [[1,0,0,0],[0,1,0,0],[0,0,1,0.195],[0,0,0,1]]
                N = np.matmul(N,L)
                #N[:,3] = N[:,3] + 0.195*N[:,2]
            if j == 4:
                L = [[1,0,0,0],[0,1,0,0],[0,0,1,0.125],[0,0,0,1]]
                N = np.matmul(N,L)
                #N[:,3] = N[:,3] + 0.125*N[:,2]

            if j == 5:
                L = [[1,0,0,0],[0,1,0,0],[0,0,1,-0.015],[0,0,0,1]]
                N = np.matmul(N,L)
                #N[:,3] = N[:,3] + 0.051*N[:,2]
            if j == 6:
                L = [[1,0,0,0],[0,1,0,0],[0,0,1,0.051],[0,0,0,1]]
                N = np.matmul(N,L)
               #N[:,3] = N[:,3] + 0.015*N[:,2]
            '''
            if j == 2:
                N[2][3] = N[2][3] + 0.195
            if j == 4:
                N[2][3] = N[2][3] + 0.125
            if j == 6:
                N[2][3] = N[2][3] + 0.051
            if j == 5:
                N[2][3] = N[2][3] + 0.015
            '''

            for k in range(3):
                jointPositions[j][k] = N[k][3]
            #print(N)

        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1


    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()

    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()

if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([pi/6,pi/6,pi/6,-pi/2,0,pi/6,pi/6])

    joint_positions, T0e = fk.forward(q)
    x = joint_positions[1] - joint_positions[6]
    print("Joint Positions:\n",joint_positions)
    print(x)
    # print("End Effector Pose:\n",T0e)