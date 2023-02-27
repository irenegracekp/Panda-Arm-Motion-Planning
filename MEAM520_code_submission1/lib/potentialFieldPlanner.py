import numpy as np
from math import pi, acos
from scipy.linalg import null_space
from copy import deepcopy


from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
'''

from calcJacobian import calcJacobian
from calculateFK import FK
from detectCollision import detectCollision
from loadmap import loadmap
'''
#Priyanshu Mittal
class PotentialFieldPlanner:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    center = lower + (upper - lower) / 2 # compute middle of range of motion of each joint
    fk = FK()

    def __init__(self, tol=1e-4, max_steps=500, min_step_size=1e-5):
        """
        Constructs a potential field planner with solver parameters.

        PARAMETERS:
        tol - the maximum distance between two joint sets
        max_steps - number of iterations before the algorithm must terminate
        min_step_size - the minimum step size before concluding that the
        optimizer has converged
        """

        # YOU MAY NEED TO CHANGE THESE PARAMETERS

        # solver parameters
        self.tol = tol
        self.max_steps = max_steps
        self.min_step_size = min_step_size


    ######################
    ## Helper Functions ##
    ######################
    # The following functions are provided to you to help you to better structure your code
    # You don't necessarily have to use them. You can also edit them to fit your own situation 

    # @staticmethod
    def attractive_force(self, target, current):
        """
        Helper function for computing the attactive force between the current position and
        the target position for one joint. Computes the attractive force vector between the 
        target joint position and the current joint position 

        INPUTS:
        target - 3x1 numpy array representing the desired joint position in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame

        OUTPUTS:
        att_f - 3x1 numpy array representing the force vector that pulls the joint 
        from the current position to the target position 
        """

        ## STUDENT CODE STARTS HERE
        # print('s:', target.shape)
        # current = current.reshape(3,1)
        # target = target.reshape(3,1)

        att_f = np.zeros((3, 1))
        eta = 3.5
        if (np.linalg.norm((current-target))) > 0.5:
            att_f = -(current - target)/ (np.linalg.norm((current-target)))
        else:
            att_f = -(eta)*(current - target)

        ## END STUDENT CODE

        return att_f

    # @staticmethod
    def repulsive_force(self, obstacle, current, unitvec=np.zeros((3,1))):
        """
        Helper function for computing the repulsive force between the current position
        of one joint and one obstacle. Computes the repulsive force vector between the 
        obstacle and the current joint position 

        INPUTS:
        obstacle - 1x6 numpy array representing the an obstacle box in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame
        unitvec - 3x1 numpy array representing the unit vector from the current joint position 
        to the closest point on the obstacle box 

        OUTPUTS:
        rep_f - 3x1 numpy array representing the force vector that pushes the joint 
        from the obstacle
        """

        ## STUDENT CODE STARTS HERE
        # current = current.reshape(3,1)
        rep_f = np.zeros((1,3))
        n = 100
        rho_0 = 1

        current = np.array(current).reshape(1,3)
        dist, unit = self.dist_point2box(current, obstacle)

        if (dist < rho_0):
            rep_f = -(n * ((1/dist)-(1/rho_0))*(1/(dist**2))*(unit))
        else:
            return rep_f
        ## END STUDENT CODE

        return rep_f

    # @staticmethod
    def dist_point2box(self, p, box):
        """
        Helper function for the computation of repulsive forces. Computes the closest point
        on the box to a given point 
    
        INPUTS:
        p - nx3 numpy array of points [x,y,z]
        box - 1x6 numpy array of minimum and maximum points of box

        OUTPUTS:
        dist - nx1 numpy array of distance between the points and the box
                dist > 0 point outside
                dist = 0 point is on or inside box
        unit - nx3 numpy array where each row is the corresponding unit vector 
        from the point to the closest spot on the box
            norm(unit) = 1 point is outside the box
            norm(unit)= 0 point is on/inside the box

         Method from MultiRRomero
         @ https://stackoverflow.com/questions/5254838/
         calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
        """
        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # Get box info
        boxMin = np.array([box[0], box[1], box[2]])
        boxMax = np.array([box[3], box[4], box[5]])
        boxCenter = boxMin*0.5 + boxMax*0.5
        p = np.array(p)

        # Get distance info from point to box boundary
        dx = np.amax(np.vstack([boxMin[0] - p[:, 0], p[:, 0] - boxMax[0], np.zeros(p[:, 0].shape)]).T, 1)
        dy = np.amax(np.vstack([boxMin[1] - p[:, 1], p[:, 1] - boxMax[1], np.zeros(p[:, 1].shape)]).T, 1)
        dz = np.amax(np.vstack([boxMin[2] - p[:, 2], p[:, 2] - boxMax[2], np.zeros(p[:, 2].shape)]).T, 1)

        # convert to distance
        distances = np.vstack([dx, dy, dz]).T
        dist = np.linalg.norm(distances, axis=1)

        # Figure out the signs
        signs = np.sign(boxCenter-p)

        # Calculate unit vector and replace with
        unit = distances / dist[:, np.newaxis] * signs
        unit[np.isnan(unit)] = 0
        unit[np.isinf(unit)] = 0

        return dist, unit

    # @staticmethod
    def compute_forces(self, target, obstacle, current):
        """
        Helper function for the computation of forces on every joints. Computes the sum 
        of forces (attactive, repulsive) on each joint. 

        INPUTS:
        target - 3x7 numpy array representing the desired joint/end effector positions 
        in the world frame
        obstacle - nx6 numpy array representing the obstacle box min and max positions
        in the world frame
        current- 3x7 numpy array representing the current joint/end effector positions 
        in the world frame

        OUTPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each 
        joint/end effector
        """

        ## STUDENT CODE STARTS HERE
        joint_forces = np.zeros((3, 7))
        att_f = np.zeros((3,7))
        
        for i in range(7):
            att_f[:,i] = (self.attractive_force(target[:,i], current[:,i])).flatten()

            rep_f = np.zeros((3,1))
            for j in range(len(obstacle)):
                rep_f = rep_f + self.repulsive_force(obstacle[j,:], current[:,i]).T

            joint_forces[:,i] = att_f[:,i] + rep_f.flatten()

        ## END STUDENT CODE

        return joint_forces
    
    # @staticmethod
    def compute_torques(self, joint_forces, q):
        """
        Helper function for converting joint forces to joint torques. Computes the sum 
        of torques on each joint.

        INPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each 
        joint/end effector
        q - 1x7 numpy array representing the current joint angles

        OUTPUTS:
        joint_torques - 1x7 numpy array representing the torques on each joint 
        """

        ## STUDENT CODE STARTS HERE

        joint_torques = np.zeros((1, 7))
        

        # print('jv:',jv)

        for i in range(7):
            jv = (calcJacobian(q.T))[0:3,:]
            jx = jv
            jx[:, i:] = 0

            joint_torques_i = ((jx.T) @ (joint_forces[:,i].reshape(3,1))).reshape(1,7)
            joint_torques = joint_torques + joint_torques_i

        ## END STUDENT CODE
        # print(joint_torques.shape)

        return joint_torques

    # @staticmethod
    def q_distance(self, target, current):
        """
        Helper function which computes the distance between any two
        vectors.

        This data can be used to decide whether two joint sets can be
        considered equal within a certain tolerance.

        INPUTS:
        target - 1x7 numpy array representing some joint angles
        current - 1x7 numpy array representing some joint angles

        OUTPUTS:
        distance - the distance between the target and the current joint sets 

        """

        ## STUDENT CODE STARTS HERE

        # distance = 0
        joint_t, x = self.fk.forward(target.reshape(7,))
        joint_c, y = self.fk.forward(current.reshape(7,))

        distance = np.linalg.norm((joint_t[0:7] - joint_c[0:7]))

        ## END STUDENT CODE
        # print(distance)

        return distance
    
    # @staticmethod
    def compute_gradient(self, q, target, map_struct):
        """
        Computes the joint gradient step to move the current joint positions to the
        next set of joint positions which leads to a closer configuration to the goal 
        configuration 

        INPUTS:
        q - 1x7 numpy array. the current joint configuration, a "best guess" so far for the final answer
        target - 1x7 numpy array containing the desired joint angles
        map_struct - a map struct containing the obstacle box min and max positions

        OUTPUTS:
        dq - 1x7 numpy array. a desired joint velocity to perform this task
        """

        ## STUDENT CODE STARTS HERE

        dq = np.zeros((1, 7))
        alpha = 0.05

        jp_t, x = self.fk.forward(target.reshape(7,))
        jp_i, y = self.fk.forward(q.reshape(7,))

        jp_t = jp_t[0:7,:]
        jp_i = jp_i[0:7,:]

        obstacle = map_struct.obstacles

        joint_forces = self.compute_forces(jp_t.T, obstacle, jp_i.T)
        #print(joint_forces.shape)

        torque = self.compute_torques(joint_forces,q)
        # print(torque.shape)

        # print(torque)
        dq = alpha * (torque/np.linalg.norm(torque))
        # print(dq.shape)

        ## END STUDENT CODE


        return dq
 

    ###############################
    ### Potential Feild Solver  ###
    ###############################

    def plan(self, map_struct, start, goal):
        """
        Uses potential field to move the Panda robot arm from the startng configuration to
        the goal configuration.

        INPUTS:
        map_struct - a map struct containing min and max positions of obstacle boxes 
        start - 1x7 numpy array representing the starting joint angles for a configuration 
        goal - 1x7 numpy array representing the desired joint angles for a configuration

        OUTPUTS:
        q - nx7 numpy array of joint angles [q0, q1, q2, q3, q4, q5, q6]. This should contain
        all the joint angles throughout the path of the planner. The first row of q should be
        the starting joint angles and the last row of q should be the goal joint angles. 
        """
        # print('s:', start.shape)
        q_path = []
        q_path.append(start)
        qn = start
        iter = 0

        while True:

            ## STUDENT CODE STARTS HERE
            
            # The following comments are hints to help you to implement the planner
            # You don't necessarily have to follow these steps to complete your code 
            err = np.linalg.norm(qn-goal)

            if err < 0.01 or iter > 300:
                q_path.append(goal)
                q_path = np.array(q_path)
                break         

            else:
                dq = self.compute_gradient(qn, goal, map_struct)
                qn = qn + dq
                q_path.append(qn.flatten())
                iter = iter+1
                

            # Compute gradient 
            # TODO: this is how to change your joint angles
            # Termination Conditions
            # if True:TODO: check termination conditions
            #     break # exit the while loop if conditions are met!

            # YOU NEED TO CHECK FOR COLLISIONS WITH OBSTACLES
            # TODO: Figure out how to use the provided function 

            # YOU MAY NEED TO DEAL WITH LOCAL MINIMA HERE
            # TODO: when detect a local minima, implement a random walk
            
            ## END STUDENT CODE
        # print(len(q_path))
        # q_path = np.array(q_path)

        return q_path

################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    np.set_printoptions(suppress=True,precision=5)

    planner = PotentialFieldPlanner()
    
    # inputs 
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    
    # potential field planning
    q_path = planner.plan(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    # show results
    for i in range(q_path.shape[0]):
<<<<<<< HEAD
        error = planner.q_distance(q_path[i], goal)
        print('iteration:',i,' q =', q_path[i, :], ' error={error:3.4f}'.format(error=error))
=======
        error = PotentialFieldPlanner.q_distance(q_path[i, :], goal)
        print('iteration:',i,' q =', q_path[i, :], ' error={error}'.format(error=error))
>>>>>>> 08769ed27a3ae945cb18dc09167d62c855a27552

    print("q path: ", q_path)
