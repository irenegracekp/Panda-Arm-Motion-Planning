import sys
import numpy as np
from copy import deepcopy
from math import pi, cos, sin

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector
import time

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

#Earlier IK and FK Files
from lib.calculateFK import FK
from lib.calculateIK6n import IK

fk = FK()
ik = IK()
i = 0

def blockTarget(T0b):
    '''
    Code to transform the axes of block
    '''
    R = T0b[0:3,0:3]
    t = T0b[0:3,3].reshape(3,1)

    last_row_plus = np.absolute(R[2,:] - 1)
    last_row_minus = np.absolute(R[2,:] + 1)
    epsilon = 0.01
    
    try:
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

    except UnboundLocalError:
        print("UnboundLocalError: could not find R_out, skipping, going to next block")
        return R, t, T0b, "continue"

    R_out = R_new
    t_out = T0b[0:3,3].reshape(3,1)
    T0b[0:3,0:3] = R_out
    T0b[0:3,3] = t_out.flatten()
    return R_out, t_out, T0b, "Allgood"

def motion2target(R_out, t_out):
    '''
    Code to safely go to above target --> to the target --> grip the target --> aobe the target again
    '''
    t_block = deepcopy(t_out)
    t_safe = t_out
    t_safe[2,0] += 0.12
    target_safe = {'R': R_out, 't': t_safe}
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
    q_b = ik.panda_ik(target_block)
    q_block = q_b[0,:]
    arm.safe_move_to_position(q_block)
    print("At Block")
    arm.exec_gripper_cmd( 0.025, 30)
    arm.safe_move_to_position(q_safe)
    return 'All good'

def ik_solution(q):
    '''
    Code to return the desired ik solution
    '''
    epsilon = 0.01
    for i in range(len(q)):
        jointpos, T0e = fk.forward(q[i])
        axis = T0e[2,2]
        if abs(axis - 1) < epsilon:
            q_r = q[i]
    return q_r

def placingPositionSide(team):
    '''
    defined placing position (center of the platform) for blocks (static and dynamic) depeneding on global variable i
    '''
    global i
    if team == 'red':
        i = i + 1
        if i == 1:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, 0.106, 0.22]).reshape((3,1))
        elif i == 2:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, 0.106, 0.27]).reshape((3,1))
        elif i == 3:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, 0.106, 0.32]).reshape((3,1))
        elif i == 4:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, 0.106, 0.37]).reshape((3,1))
        elif i == 5:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, 0.106, 0.42]).reshape((3,1))
        elif i == 6:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, 0.106, 0.47]).reshape((3,1))
        elif i == 7:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, 0.106, 0.52]).reshape((3,1))
    elif team == 'blue':
        i = i + 1
        if i == 1:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, -0.106, 0.227]).reshape((3,1))
            print("i", i)
        elif i == 2:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, -0.106, 0.277]).reshape((3,1))
            print("i", i)
        elif i == 3:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, -0.106, 0.327]).reshape((3,1))
            print("i", i)
        elif i == 4:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, -0.106, 0.37]).reshape((3,1))
            print("i", i)
        elif i == 5:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, -0.106, 0.427]).reshape((3,1))
            print("i", i)
        elif i == 6:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, -0.106, 0.47]).reshape((3,1))
            print("i", i)
        elif i == 7:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.612, -0.106, 0.527]).reshape((3,1))
            print("i", i)
    return R_out, t_out

def placingPositionCenter(team):
    '''
    defined placing position (center of the platform) for blocks (static and dynamic) depeneding on global variable i
    '''
    global i
    if team == 'red':
        i = i + 1
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
        elif i == 5:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, 0.181, 0.42]).reshape((3,1))
        elif i == 6:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, 0.181, 0.47]).reshape((3,1))
        elif i == 7:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, 0.181, 0.52]).reshape((3,1))
    elif team == 'blue':
        i = i + 1
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
        elif i == 5:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, -0.181, 0.427]).reshape((3,1))
            print("i", i)
        elif i == 6:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, -0.181, 0.47]).reshape((3,1))
            print("i", i)
        elif i == 7:
            R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
            t_out = np.array([0.562, -0.181, 0.527]).reshape((3,1))
            print("i", i)
    return R_out, t_out

def motion2place(R_out, t_out):
    '''
    Code to safely go above the placing position --> to placing position --> placing --> going back
    '''
    t_block = deepcopy(t_out)
    t_safe = t_out
    t_safe[2,0] += 0.12
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


def staticBlocksSide(team):
    '''
    Code to place the static blocks
    '''
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
        iter_num = 0
        for (name, pose) in detector.get_detections():
            iter_num = iter_num + 1
            if iter_num == 5:
                break

            print(name,'\n',pose)

            Teb = H_ee_camera @ pose
            T0b = T0e @ Teb
            
            R_out, t_out, T0b, worked2 = blockTarget(T0b)
            if worked2 == 'continue':
                print("block target epsilon")
                continue

            worked = motion2target(R_out, t_out)
            if worked == 'continue':
                print("block target no ik")
                continue

            if not getGripperState():
                print("no gripping")
                continue

            print("motion to place starting")
            R_out, t_out = placingPositionSide(team)

            motion2place(R_out, t_out)
            print("motion to place complete")

    elif team == 'blue':
        print("Blue Calculations")
        R_scan = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
        t_scan = np.array([0.562,0.15,0.55]).reshape((3,1))
        target_scan = {'R': R_scan, 't': t_scan}
        q_scan = ik.panda_ik(target_scan)
        arm.safe_move_to_position(q_scan[0]) 
        
        H_ee_camera = detector.get_H_ee_camera()
        joints, T0e = fk.forward(q_scan[0])
        for (name, pose) in detector.get_detections():
            print(name,'\n',pose)

            Teb = H_ee_camera @ pose
            T0b = T0e @ Teb
            
            R_out, t_out, T0b, worked2 = blockTarget(T0b)
            if worked2 == 'continue':
                continue
            worked = motion2target(R_out, t_out)
            if worked == 'continue':
                continue

            if not getGripperState():
                continue

            R_out, t_out = placingPositionSide(team)
            motion2place(R_out, t_out)

def staticBlocksCenter(team):
    '''
    Code to place the static blocks (Center)
    '''
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

            R_out, t_out, T0b, worked2 = blockTarget(T0b)
            if worked2 == 'continue':
                continue

            worked = motion2target(R_out, t_out)
            if worked == 'continue':
                continue

            if not getGripperState():
                continue
            R_out, t_out = placingPositionCenter(team)
            motion2place(R_out, t_out)

    elif team == 'blue':
        print("Blue Calculations")
        R_scan = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
        t_scan = np.array([0.562,0.15,0.55]).reshape((3,1))
        target_scan = {'R': R_scan, 't': t_scan}
        q_scan = ik.panda_ik(target_scan)
        arm.safe_move_to_position(q_scan[0]) 
        
        H_ee_camera = detector.get_H_ee_camera()
        joints, T0e = fk.forward(q_scan[0])
        for (name, pose) in detector.get_detections():
            print(name,'\n',pose)

            Teb = H_ee_camera @ pose
            T0b = T0e @ Teb
            
            R_out, t_out, T0b = blockTarget(T0b)

            R_out, t_out, T0b, worked2 = blockTarget(T0b)
            if worked2 == 'continue':
                continue
                
            worked = motion2target(R_out, t_out)
            if worked == 'continue':
                continue

            if not getGripperState():
                continue

            R_out, t_out = placingPositionCenter(team)
            motion2place(R_out, t_out)  
            

def moveArmto(R_scan, t_scan):
    '''
    Uses Rotation matrix and position matrix to get the required ik solution from the IK Code
    '''
    target_scan = {'R': R_scan, 't': t_scan}
    q_scan = ik.panda_ik(target_scan)
    arm.safe_move_to_position(q_scan[0]) 

def getGripperState():
    '''
    Code to check if the grippper has gripped the block or not
    '''
    gripper_state = arm.get_gripper_state()
    gripper_position = gripper_state['position']
    gripper_open_state = np.array([0.0399984693827032, 0.03999966955423893])
    gripper_closed_state = np.array([0.0247281691092467, 0.024727231180735634])

    max = (gripper_open_state - gripper_position)
    min = (gripper_position - gripper_closed_state)
    
    if max[0]>0 and min[0]>0:
        state = True
    else:
        state = False

    return state

def dynamicBlocksSide(team):
    '''
    Code to place the dynamic blocks
    '''
    if team == 'red':
        print("Red Calculations for Dynamic Side")
        arm.exec_gripper_cmd(0.095, 0)
        #pos 1
        arm.safe_move_to_position([0.56357846, 0.68441664, 0.6364463, -1.86937153, 0.89971877, 1.47693233, -1.50356391]) 
        #pose2
        arm.safe_move_to_position([0.87163083, 1.04889575, 0.61964027, -1.26954743, 1.0154893, 1.7721214, -1.28108336])

        time.sleep(8)
        arm.exec_gripper_cmd(0.025, 15)

        j = 0
        while True and j<5:
            if getGripperState():
                print("Gripper Locked")
                R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
                t_out = np.array([0.48, 0.221, 0.227]).reshape((3,1))
                t_check = np.array([0.48, 0.221, 0.3]).reshape((3,1))
                arm.safe_move_to_position([0.56357846, 0.68441664, 0.6364463, -1.86937153, 0.89971877, 1.47693233, -1.50356391]) 
                moveArmto(R_out, t_check)
                arm.exec_gripper_cmd(0.025, 15)
                if getGripperState():
                    print("Gripper still Locked")
                    moveArmto(R_out, t_out)
                    arm.exec_gripper_cmd(0.095, 0)
                break
            else:
                arm.exec_gripper_cmd(0.095, 0)
                time.sleep(4)
                arm.exec_gripper_cmd(0.025, 15)
                j = j + 1

    elif team == 'blue':
        print("Blue Calculations for Dynamic Side")
        
        arm.exec_gripper_cmd(0.095, 0)
        #pos 1
        arm.safe_move_to_position([-0.09107165 ,-1.57077993 ,-1.728371 ,  -2.10707002 ,-0.07269432,  2.06661315,-0.91413047]) 
        #pose2
        arm.safe_move_to_position([ 0.70438784, -1.60488331 ,-1.72085014, -0.86243221 ,-0.09426209,  1.64267055,-0.91160096]) 
        
        time.sleep(8)
        arm.exec_gripper_cmd(0.025, 15)

        j = 0
        while True and j<8:
            if getGripperState():
                print("Gripper Locked")
                R_out = np.array([1,0,0,0,-1,0,0,0,-1]).reshape((3,3))
                t_out = np.array([0.484, -0.221, 0.227]).reshape((3,1))
                t_check = np.array([0.48, -0.221, 0.3]).reshape((3,1))
                arm.safe_move_to_position([-0.09107165 ,-1.57077993 ,-1.728371 ,  -2.10707002 ,-0.07269432,  2.06661315,-0.91413047]) 
                moveArmto(R_out, t_check)
                arm.exec_gripper_cmd(0.025, 15)
                if getGripperState():
                    print("Gripper still Locked")
                    moveArmto(R_out, t_out)
                    arm.exec_gripper_cmd(0.095, 0)
                break
            else:
                arm.exec_gripper_cmd(0.095, 0)
                time.sleep(4)
                arm.exec_gripper_cmd(0.025, 15)
                j = j + 1

def dynamicstackingSide(team):
    '''
    Code to place the placed dynamic blocks at the top of the stacking
    '''
    if team == 'red':

        R_safe = np.array([-1,0,0,0,1,0,0,0,-1]).reshape((3,3))
        t_safe = np.array([0.612, 0.106, 0.53]).reshape((3,1))
        moveArmto(R_safe, t_safe)

        R_out = np.array([-1,0,0,0,1,0,0,0,-1]).reshape((3,3))
        t_out = np.array([0.48, 0.221, 0.227]).reshape((3,1))           
        worked = motion2target(R_out, t_out)

        R_safe = np.array([-1,0,0,0,1,0,0,0,-1]).reshape((3,3))
        t_safe = np.array([0.612, 0.106, 0.53]).reshape((3,1))
        moveArmto(R_safe, t_safe)

        R_out, t_out = placingPositionSide(team)
        motion2place(R_out, t_out)

    if team == 'blue':

        R_safe = np.array([-1,0,0,0,1,0,0,0,-1]).reshape((3,3))
        t_safe = np.array([0.612, -0.106, 0.53]).reshape((3,1))
        moveArmto(R_safe, t_safe)

        R_out = np.array([-1,0,0,0,1,0,0,0,-1]).reshape((3,3))
        t_out = np.array([0.48, -0.221, 0.227]).reshape((3,1))

        worked = motion2target(R_out, t_out)

        R_safe = np.array([-1,0,0,0,1,0,0,0,-1]).reshape((3,3))
        t_safe = np.array([0.48, -0.221, 0.53]).reshape((3,1))
        moveArmto(R_safe, t_safe)
        R_out, t_out = placingPositionSide(team)
        motion2place(R_out, t_out)

def dynamicBlocksCenter(team):
    '''
    Code to place the dynamic blocks at the center of the platform
    '''
    if team == 'red':
        print("Red Calculations for Dynamic Center")
        arm.exec_gripper_cmd(0.095, 0)
        #pos 1
        arm.safe_move_to_position([0.56357846, 0.68441664, 0.6364463, -1.86937153, 0.89971877, 1.47693233, -1.50356391]) 
        #pos 2
        arm.safe_move_to_position([0.87163083, 1.04889575, 0.61964027, -1.26954743, 1.0154893, 1.7721214, -1.28108336])

        time.sleep(8)
        arm.exec_gripper_cmd(0.025, 15)

        j = 0
        while True and j<4:
            if getGripperState():
                print("Gripper Locked")
                arm.safe_move_to_position([0.56357846, 0.68441664, 0.6364463, -1.86937153, 0.89971877, 1.47693233, -1.50356391]) 
                arm.exec_gripper_cmd(0.025, 15)
                R_out, t_out = placingPositionCenter(team)
                motion2place(R_out, t_out)
                break
            else:
                arm.exec_gripper_cmd(0.095, 0)
                time.sleep(4)
                arm.exec_gripper_cmd(0.025, 15)
                j = j + 1

    elif team == 'blue':
        print("Blue Calculations for Dynamic Center")
        
        arm.exec_gripper_cmd(0.095, 0)
        #pos 1
        arm.safe_move_to_position([-0.09107165 ,-1.57077993 ,-1.728371 ,  -2.10707002 ,-0.07269432,  2.06661315,-0.91413047]) 
        #pose2
        arm.safe_move_to_position([ 0.70438784, -1.60488331 ,-1.72085014, -0.86243221 ,-0.09426209,  1.64267055,-0.91160096]) 
        
        time.sleep(8)
        arm.exec_gripper_cmd(0.025, 15)

        j = 0
        while True and j<4:
            if getGripperState():
                print("Gripper Locked")
                arm.safe_move_to_position([0.56357846, 0.68441664, 0.6364463, -1.86937153, 0.89971877, 1.47693233, -1.50356391]) 
                R_out, t_out = placingPositionCenter(team)
                motion2place(R_out, t_out)
                break
            else:
                arm.exec_gripper_cmd(0.095, 0)
                time.sleep(4)
                arm.exec_gripper_cmd(0.025, 15)
                j = j + 1

def sabotoge(team):
    '''
    Code to sabotoge the dynamic blocks
    '''
    print("BE READY FOR SABOTOGE")
    if team == 'red':
        arm.safe_move_to_position([0.87163083 - pi/15, 1.04889575, 0.61964027, -1.26954743, 1.0154893, 1.7721214 + pi/2, -1.28108336])
        print('At the start position (SABOTAGE)')
        for i in range(3):
            arm.safe_move_to_position([0.87163083 + pi/15, 1.04889575, 0.61964027, -1.26954743, 1.0154893, 1.7721214 + pi/2, -1.28108336])
            time.sleep(9)
            print("Lets begin, HA HA HA")
            arm.safe_move_to_position([0.87163083 + pi/4, 1.04889575, 0.61964027, -1.26954743, 1.0154893, 1.7721214 + pi/2, -1.28108336])
            time.sleep(3)

    elif team == 'blue':
        arm.safe_move_to_position([ 0.70438784 - pi/15, -1.60488331 ,-1.72085014, -0.86243221 ,-0.09426209, 1.64267055 + pi/2,-0.91160096])
        print('At the start position (SABOTAGE)')
        for i in range(3):
            arm.safe_move_to_position([ 0.70438784 + pi/15, -1.60488331 ,-1.72085014, -0.86243221 ,-0.09426209, 1.64267055 + pi/2,-0.91160096])
            time.sleep(9)
            print("Lets begin, HA HA HA")
            arm.safe_move_to_position([ 0.70438784 + pi/4, -1.60488331 ,-1.72085014, -0.86243221 ,-0.09426209, 1.64267055 + pi/2,-0.91160096]) 
            time.sleep(3)

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
    dynamicBlocksCenter(team)
    staticBlocksCenter(team)
    
    dynamicBlocksCenter(team)
    dynamicBlocksCenter(team)


    #sabotoge(team)


    #dynamicBlocksSide(team)
    #staticBlocksSide(team)
    #dynamicstackingSide(team)
    #sabotoge(team)
    

    
    # Move around...

    # END STUDENT CODE
    
    # Move around...

    # END STUDENT CODE