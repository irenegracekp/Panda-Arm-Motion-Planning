import sys
import numpy as np
from copy import deepcopy
from math import pi, cos, sin

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

#Earlier IK and FK Files
from lib.calculateFK import FK
from lib.calculateIK6n import IK

fk = FK()
ik = IK()

def blockTarget(T0b):
    #Code to transform the axis of block
    R = T0b[0:3,0:3]
    print('R:', R)

    last_row_plus = np.absolute(R[2,:] - 1)
    last_row_minus = np.absolute(R[2,:] + 1)
    epsilon = 0.01
    
    if last_row_plus[0] < epsilon:
        
        print("xup")
        theta = -pi/2
        R_y_90 = np.array([[cos(theta), 0, sin(theta)],[0, 1, 0],[-sin(theta), 0, cos(theta)]])
        R_new = R@R_y_90
    elif last_row_minus[0] <  epsilon:
        print("xdown")   
        theta = pi/2
        R_y_90 = np.array([[cos(theta), 0, sin(theta)],[0, 1, 0],[-sin(theta), 0, cos(theta)]])
        R_new = R@R_y_90
    elif last_row_plus[1] < epsilon:
        print("yup")
        theta = pi/2
        R_x_90 = np.array([[1, 0, 0],[0, cos(theta), -sin(theta)],[0, sin(theta), cos(theta)]])
        R_new = R@R_x_90
    elif last_row_minus[1] < epsilon:
        print("ydown")
        theta = -pi/2
        R_x_90 = np.array([[1, 0, 0],[0, cos(theta), -sin(theta)],[0, sin(theta), cos(theta)]])
        R_new = R@R_x_90
    elif last_row_plus[2] < epsilon:
        print("zup")
        theta = pi
        R_x_180 = np.array([[1, 0, 0],[0, cos(theta), -sin(theta)],[0, sin(theta), cos(theta)]])
        R_new = R@R_x_180
    elif last_row_minus[2] < epsilon:
        print("zdown")
        R_new = R

    R_out = R_new
    print(R_out)
    t_out = T0b[0:3,3].reshape(3,1)
    T0b[0:3,0:3] = R_out
    T0b[0:3,3] = t_out.flatten()
    return R_out, t_out, T0b

def motion2target(R_out, t_out):
    t_block = deepcopy(t_out)
    t_safe = t_out
    t_safe[2,0] = 0.35
    target_safe = {'R': R_out, 't': t_safe}
    print(target_safe)
    q = ik.panda_ik(target_safe)
    try: 
        q_safe = q[0,:]
        arm.safe_move_to_position(q_safe)
        print("Hovering") 
        arm.exec_gripper_cmd( 0.08, 0)
    except IndexError:
        print("No solution found, going to next block") 
        return 'continue'

    target_block = {'R': R_out, 't': t_block}
    print(target_block)
    q_b = ik.panda_ik(target_block)
    q_block = q_b[0,:]
    arm.safe_move_to_position(q_block)
    print("At Block")
    arm.exec_gripper_cmd( 0.049,15)
    arm.safe_move_to_position(q_safe)
    return 'All good'

def ik_solution(q):
    epsilon = 0.01
    for i in range(len(q)):
        jointpos, T0e = fk.forward(q[i])
        axis = T0e[2,2]
        if abs(axis - 1) < epsilon:
            q_r = q[i]
    return q_r

def placingPosition(i, team):
    if team == 'red':
        if i == 1:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, 0.181, 0.22]).reshape((3,1))
        elif i == 2:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, 0.181, 0.27]).reshape((3,1))
        elif i == 3:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, 0.181, 0.32]).reshape((3,1))
        elif i == 4:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, 0.181, 0.37]).reshape((3,1))
    elif team == 'blue':
        if i == 1:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, -0.181, 0.227]).reshape((3,1))
            print("i", i)
        elif i == 2:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, -0.181, 0.277]).reshape((3,1))
            print("i", i)
        elif i == 3:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, -0.181, 0.327]).reshape((3,1))
            print("i", i)
        elif i == 4:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, -0.181, 0.37]).reshape((3,1))
            print("i", i)
    return R_out, t_out

def motion2place(R_out, t_out):
    t_block = deepcopy(t_out)
    t_safe = t_out
    t_safe[2,0] = 0.55
    target_safe = {'R': R_out, 't': t_safe}
    q = ik.panda_ik(target_safe)
    q_safe = q[0,:]
    arm.safe_move_to_position(q_safe)
    print("Hovering") 

    target_block = {'R': R_out, 't': t_block}
    q_b = ik.panda_ik(target_block)
    q_block = q_b[0,:]
    arm.safe_move_to_position(q_block)
    print("At Block")
    arm.exec_gripper_cmd( 0.08, 0)

    arm.safe_move_to_position(q_safe)

def getGripperState():
    gripper_state = arm.get_gripper_state()
    gripper_position = gripper_state['position']
    gripper_open_state = np.array([0.0399984693827032, 0.03999966955423893])
    gripper_closed_state = np.array([5.4882845068834714e-05, 5.512154274201849e-05])

    open_norm = np.linalg.norm(gripper_position - gripper_open_state)
    closed_norm = np.linalg.norm(gripper_position - gripper_closed_state)
    epsilon = 0.00001
    
    if gripper_position[0] < 0.03:
        gripper_state = 0
    else:
        gripper_state = 1

    return gripper_state

def staticBlocks(team):
    if team == 'red':
        print("Red Calculations")
        R_scan = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
        t_scan = np.array([0.562,-0.15,0.55]).reshape((3,1))
        target_scan = {'R': R_scan, 't': t_scan}
        q_scan = ik.panda_ik(target_scan)
        arm.safe_move_to_position(q_scan[0]) 
        
        H_ee_camera = detector.get_H_ee_camera()
        joints, T0e = fk.forward(q_scan[0])
        i = 1
        for (name, pose) in detector.get_detections():
            print(name,'\n',pose)

            Teb = H_ee_camera @ pose
            T0b = T0e @ Teb
            
            R_out, t_out, T0b = blockTarget(T0b)
            worked = motion2target(R_out, t_out)
            if worked == 'continue':
                continue
            R_out, t_out = placingPosition(i, team)
            motion2place(R_out, t_out)
            i = i + 1


    elif team == 'blue':
        print("Blue Calculations")
        R_scan = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
        t_scan = np.array([0.562,0.15,0.55]).reshape((3,1))
        target_scan = {'R': R_scan, 't': t_scan}
        q_scan = ik.panda_ik(target_scan)
        arm.safe_move_to_position(q_scan[0]) 
        
        H_ee_camera = detector.get_H_ee_camera()
        joints, T0e = fk.forward(q_scan[0])
        i = 1
        for (name, pose) in detector.get_detections():
            print(name,'\n',pose)

            Teb = H_ee_camera @ pose
            T0b = T0e @ Teb
            
            R_out, t_out, T0b = blockTarget(T0b)
            worked = motion2target(R_out, t_out)
            if worked == 'continue':
                continue

            R_out, t_out = placingPosition(i, team)
            motion2place(R_out, t_out)
            i = i + 1

def moveArmto(R_scan, t_scan):
    target_scan = {'R': R_scan, 't': t_scan}
    q_scan = ik.panda_ik(target_scan)
    arm.safe_move_to_position(q_scan[0]) 

def dynamicBlockPicking():

    for i in range(5):
        arm.exec_gripper_cmd( 0.030,15)
        if getGripperState():
            return 'blockin'
        else:
            R_scan = np.array([0,0, 1,0,-1,0,1,0,0]).reshape((3,3))
            t_scan = np.array([0, -0.690, 0.228]).reshape((3,1))
            target_scan = {'R': R_scan, 't': t_scan}
            q_scan = ik.panda_ik(target_scan)
            arm.safe_move_to_position(q_scan[1]) 
            arm.exec_gripper_cmd( 0.095,0)
    
    return 'noblock'


def dynamicBlocks(team):
    if team == 'red':
        print("Red Calculations")
        #pos 1
        arm.exec_gripper_cmd( 0.095, 0)
        R_scan = np.array([0,0, 1,0,-1,0,1,0,0]).reshape((3,3))
        t_scan = np.array([0, -0.450, 0.225]).reshape((3,1))
        moveArmto(R_scan, t_scan)


        #pos 2
        R_scan = np.array([0,0, 1,0,-1,0,1,0,0]).reshape((3,3))
        t_scan = np.array([0, -0.690, 0.229]).reshape((3,1))
        target_scan = {'R': R_scan, 't': t_scan}
        q_scan = ik.panda_ik(target_scan)
        arm.safe_move_to_position(q_scan[1]) 
        
        blockstat = dynamicBlockPicking() 

        if blockstat == 'blockin':
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, -0.181, 0.227]).reshape((3,1))
            moveArmto(R_scan, t_scan) 

    elif team == 'blue':
        print("Blue Calculations")
        #pos 1
        arm.exec_gripper_cmd(0.095, 0)
        R_scan = np.array([0,0, 1,0,-1,0,1,0,0]).reshape((3,3))
        t_scan = np.array([0, -0.450, 0.25]).reshape((3,1))
        moveArmto(R_scan, t_scan)

        #pos 2
        R_scan = np.array([0,0, 1,0,-1,0,1,0,0]).reshape((3,3))
        t_scan = np.array([0, -0.690, 0.25]).reshape((3,1))
        target_scan = {'R': R_scan, 't': t_scan}
        q_scan = ik.panda_ik(target_scan)
        arm.safe_move_to_position(q_scan[1]) 
        
        blockstat = dynamicBlockPicking() 

        if blockstat == 'blockin':
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, -0.181, 0.227]).reshape((3,1))
            moveArmto(R_scan, t_scan) 

        

if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE
    
    dynamicBlocks(team)
    #staticBlocks(team)
    
    # Move around...

    # END STUDENT CODE
    
    # Move around...

    # END STUDENT CODE