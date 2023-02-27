import numpy as np 
from lib.calcJacobian import calcJacobian



def IK_velocity(q_in, v_in, omega_in):#hello
     """
     :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
     :param v_in: The desired linear velocity in the world frame. If any element is
     Nan, then that velocity can be anything
     :param omega_in: The desired angular velocity in the world frame. If any
     element is Nan, then that velocity is unconstrained i.e. it can be anything
     :return:
     dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
          are infeasible, then dq should minimize the least squares error. If v_in
          and omega_in have multiple solutions, then you should select the solution
          that minimizes the l2 norm of dq
     """
     

     ## STUDENT CODE GOES HERE
     dq = np.zeros((1, 7))
     J = calcJacobian(q_in)

     v_in = v_in.reshape((3,1))
     omega_in = omega_in.reshape((3,1))
     e = np.vstack((v_in, omega_in))
     #print(e)

     idx = []
     for i in range(len(e)):
          if np.isnan(e[i]):
               idx.append(i)


     J = np.delete(J, idx, axis = 0)
     e = np.delete(e, idx, axis = 0)
     dq = np.linalg.lstsq(J, e, rcond=None)
     dq = dq[0]

     dq = dq.flatten()

     return dq

if __name__ == '__main__':
     q_in= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
     v_in = np.array([1,1,np.nan])
     omega_in = np.array([1,1,1])

     dq = IK_velocity(q_in, v_in, omega_in)
