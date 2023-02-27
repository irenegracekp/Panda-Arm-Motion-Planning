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
    q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    joint_positions, T0e = fk.forward(q)

    #print("Joint Positions:\n",joint_positions)
    #print("End Effector Pose:\n",T0e)


class IK:
    """
    Solves the 6 DOF (joint 5 fixed) IK problem for panda robot arm
    """
    # offsets along x direction 
    a1 = 0 
    a2 = 0
    a3 = 0.0825
    a4 = 0.0825
    a5 = 0 
    a6 = 0.088
    a7 = 0

    # offsets along z direction 
    d1 = 0.333
    d2 = 0 
    d3 = 0.316 
    d4 = 0
    d5 = 0.384
    d6 = 0 
    d7 = 0.210
    
    # This variable is used to express an arbitrary joint angle 
    Q0 = 0.123


    def panda_ik(self, target):
        """
        Solves 6 DOF IK problem given physical target in x, y, z space
        Args:
            target: dictionary containing:
                'R': numpy array of the end effector pose relative to the robot base 
                't': numpy array of the end effector position relative to the robot base 

        Returns:
             q = nx7 numpy array of joints in radians (q5: joint 5 angle should be 0)
        """
        q = np.zeros((8,7))
        wrist_pos = self.kin_decouple(target)
        #q[:[3,5,6]] = self.ik_pos(wrist_pos)
        q[0:4,[3,5,6]] = self.ik_pos(wrist_pos)
        q[4:8,[3,5,6]] = self.ik_pos(wrist_pos)
        # Student's code goes in between: 
        R = target.get('R')
        # Student's code goes in between:
        
        joints_467 = self.ik_pos(wrist_pos)
        joints_123 = self.ik_orient(R,joints_467)
        ## DO NOT EDIT THIS PART 
        # This will convert your joints output to the autograder format
        q = self.sort_joints(q)
        ## DO NOT EDIT THIS PART
        return q

    def kin_decouple(self, target):
        """
        Performs kinematic decoupling on the panda arm to find the position of wrist center
        Args: 
            target: dictionary containing:
                'R': numpy array of the end effector pose relative to the robot base 
                't': numpy array of the end effector position relative to the robot base 

        Returns:
             wrist_pos = 3x1 numpy array of the position of the wrist center in frame 7
        """
        R07 = target.get('R')
        t07 = target.get('t')
        R70 = R07.T
        t70 = (-R70) @ t07
        #print(t70)
        #c = np.array([0,0,1])
        #c = c.reshape(3,1)

        #pos of joint 2 in frame 7 (o72)
        #o72 = t70 + self.d1*(R70[:,-1])
        wrist_pos = t70 + self.d1*(R70[:,-1])
        #print(wrist_pos)
        #print(wrist_pos.shape)

        #wrist_pos = []
        return wrist_pos 

    def ik_pos(self, wrist_pos):
        """
        Solves IK position problem on the joint 4, 6, 7 
        Args: 
            wrist_pos: 3x1 numpy array of the position of the wrist center in frame 7

        Returns:
             joints_467 = nx3 numpy array of all joint angles of joint 4, 6, 7
        """
        #joint var 7
        joints_467 = np.zeros((4,3))


        q71 = np.pi - (np.arctan2(-wrist_pos[1],wrist_pos[0]) + np.pi/4)
        if q71 >= 0:
            q72 = q71 - pi
        else:
            q72 = q71 + pi
        
        q7 = [q71,q72]

        #print(q7)
        #joints_467[0,2] = q7

        a1 = np.sqrt(self.d5**2+self.a4**2)
        a2 = np.sqrt(self.d3**2+self.a3**2)

        #q7 in joints_467
        joints_467[0,2] = q71
        joints_467[1,2] = q71
        joints_467[2,2] = q72
        joints_467[3,2] = q72

        for i in range(2):

        #joint var 6
            T67 = np.array([np.cos(q7[i]-pi/4), -np.sin(q7[i]-pi/4), 0, 0, np.sin(q7[i]-pi/4), np.cos(q7[i]-pi/4), 0, 0,0, 0,1,self.d7,0,0,0,1])
            T67 = T67.reshape(4,4)
            o27 = [wrist_pos[0], wrist_pos[1], wrist_pos[2],1]
            o26 = np.array(T67 @ o27)[0:3] 
            #print(o26)

            if o26[0]<0:
                o256 = o26 + [self.a6,0,0]
            else:
                o256 = o26 - [self.a6,0,0]
            
            a3 = (o256[0]**2 + o256[2]**2 - a1**2 - a2**2)/(2*a1*a2)
            if a3>1 or a3 <-1:
                a3 = np.round(a3)

            theta21 = np.arccos((a3))
            theta22 = -theta21

            #theta2 = [theta21,theta22]
            #print(theta2)
            
            theta11 = np.arctan2(o256[2],o256[0]) - np.arctan2((a2*np.sin(theta21)),(a1+a2*np.cos(theta21)))
            theta12 = np.arctan2(o256[2],o256[0]) - np.arctan2((a2*np.sin(theta22)),(a1+a2*np.cos(theta22)))
            #print(theta1)

            #joint var 6
            q61 = theta11 - np.pi/2 + np.arctan2(self.a3,self.d5)
            q62 = theta12 - np.pi/2 + np.arctan2(self.a3,self.d5)
            #print(q6)
        
            #joints_467[0,1] = q6

            #joint var 4
            q41 = theta21 + np.arctan2(self.d3,self.a3) + np.arctan2(self.d5,self.a4) - np.pi
            q42 = theta22 + np.arctan2(self.d3,self.a3) + np.arctan2(self.d5,self.a4) - np.pi
            #print(q4)
            
            #inserting values in matrix
            #4
            joints_467[i,0] = q41
            joints_467[i,1]= q61
            joints_467[i+2,0] = q42
            joints_467[i+2,1]= q62
        
        #joints_467[0,0] = q4

        #print(joints_467)
        #print(joints_467.shape)
        return joints_467

    def ik_orient(self, R, joints_467):
        """
        Solves IK orientation problem on the joint 1, 2, 3
        Args: 
            R: numpy array of the end effector pose relative to the robot base 
            joints_467: nx3 numpy array of all joint angles of joint 4, 6, 7

        Returns:
            joints_123 = nx3 numpy array of all joint angles of joint 1, 2 ,3
        """
        joints_123 = np.zeros((8,3))
        
        for j in range (4):
            a = [-self.a4, 0,self.a6,0]
            d = [0,self.d5,0,self.d7]
            alpha = np.array([-pi/2, pi/2, pi/2, 0])
            theta = [joints_467[j,0],0, joints_467[j,1], joints_467[j,2]-pi/4]
            x = np.identity(3)
            
            for i in range(4):
                A = np.array([np.cos(theta[i]), -np.sin(theta[i])*np.cos(alpha[i]), np.sin(theta[i])*np.sin(alpha[i]), a[i]*np.cos(theta[i]), np.sin(theta[i]), np.cos(theta[i])*np.cos(alpha[i]),-np.cos(theta[i])*np.sin(alpha[i]), a[i]*np.sin(theta[i]), 0, np.sin(alpha[i]), np.cos(alpha[i]), d[i],0,0,0,1])
                A = A.reshape(4,4)
                return A
                #print(A)
                x = np.matmul(x,A)
            #print(x)
            R37 = x.T
            #print(R37)
            R03 = np.matmul(R,R37)
            #print(R03)

            #q2
            if R03[2,0] == 0 and R03[2,2] == 0:   ###used OR instead of AND (JUGAAD)
                if R03[2,1] == 1:
                    q31 = 0
                    q32 = 0
                    q21 = 0
                    q22 =2
                    q11 = np.arctan2(R03[0,0],R03[1,0])
                    q12 = np.arctan2(R03[0,0],R03[1,0])
                elif R03[2,1] == -1:
                    q31 = np.arctan2(-R03[0,0],-R03[0,1])
                    q32 = np.arctan2(-R03[0,0],-R03[0,1])
                    q21 = pi
                    q22 = pi
                    q11 = 0
                    q12 = 0
                else:
                    break
            else:
                q2 = np.arccos(R03[2,1])
                
                #q1
                q31 = np.arctan2(((R03[2,2])/(np.sin(q2))),((R03[2,0])/(-np.sin(q2))))
                q32 = np.arctan2(((R03[2,2])/(np.sin(-q2))),((R03[2,0])/(-np.sin(-q2))))
                

                #q3
                q11 = np.arctan2((R03[1,1]/np.sin(q2)), (R03[0,1]/(-np.sin(q2))))
                q12 = np.arctan2((R03[1,1]/np.sin(-q2)), (R03[0,1]/(-np.sin(-q2))))

            joints_123[j,2] = q11
            joints_123[j+4,2] = q12

            joints_123[j,0] = q31
            joints_123[j+4,0] = q32

            joints_123[j,1] = q2
            joints_123[j+4,1] = -q2
        

        return joints_123
    
    def sort_joints(self, q, col=0):
        """
        Sort the joint angle matrix by ascending order 
        Args: 
            q: nx7 joint angle matrix 
        Returns: 
            q_as = nx7 joint angle matrix in ascending order 
        """
        if col != 7: 
            q_as = q[q[:, col].argsort()]
            for i in range(q_as.shape[0]-1):
                if (q_as[i, col] < q_as[i+1, col]):
                    # do nothing
                    pass
                else:
                    for j in range(i+1, q_as.shape[0]):
                        if q_as[i, col] < q_as[j, col]:
                            idx = j
                            break
                        elif j == q_as.shape[0]-1:
                            idx = q_as.shape[0]

                    q_as_part = self.sort_joints(q_as[i:idx, :], col+1)
                    q_as[i:idx, :] = q_as_part
        else: 
            q_as = q[q[:, -1].argsort()]
        return q_as

def main(): 
    
    # fk solution code
    fk = FK()

    # input joints  
    q1 = 0
    q2 = 0
    q3 = 0
    q4 = 0
    q6 = 0
    q7 = 0
    
    q_in  = np.array([q1, q2, q3, q4, 0, q6, q7])
    [_, T_fk] = fk.forward(q_in)

    # input of IK class
    target = {'R': T_fk[0:3, 0:3], 't': T_fk[0:3, 3]}
    ik = IK()
    q = ik.panda_ik(target)
    
    # verify IK solutions 
    #for i in range(q.shape[0]):
      #  [_, T_ik] = fk.forward(q[i, :])
        #print('Matrix difference = ')
        #print(T_fk - T_ik)
        #print()

if __name__ == '__main__':
    main()