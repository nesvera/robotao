#!/usr/bin/env python

import rospy
import std_msgs
from geometry_msgs.msg import Twist

import numpy as np
import time

robot_vel = np.array([0.0,0.0,0.0])
robot_jump = 0

class Gait:

    """
    Attributes
    ----------
    C1   - Halt Position Leg Extension   e.g. 0 = fully extended, 1 = fully contrated
    C2   - Halt Position Leg Roll Angle
    C3   - Halt Position Leg Pitch Angle
    C4   - Halt Position Foot Roll Angle
    C5   - Halt Position Foot Pitch Angle
    C6   - Constant Ground Push
    C7   - Proportional Ground Push
    C8   - Constant Step Height
    C9   - Proportional Step Height
    Ct0  - Swing Start Timing
    Ct1  - Swing Stop Timing
    C10  - Sagittal Swing Amplitude Fwd
    C11  - Sagittal Swing Amplitude Bwd
    C12  - Lateral Swing Amplitude
    C13  - Lateral Swing Amplitude Offset
    C14  - Turning Lateral Swing Amplitude Offset
    C15  - Rotational Swing Amplitude
    C16  - Rotational Swing Amplitude Offset
    C17  - Lateral Hip Swing Amplitude
    C18  - Forward Lean
    C19  - Backward Lean
    C20  - Forward and Turning Lean
    C21  - Gait Velocity Limiting Norm p
    C22  - Sagittal Acceleration
    C23  - Lateral Acceleration
    C24  - Rotational Acceleration
    C25  - Constant Step Frequency
    C26  - Sagittal Proportional Step Frequency
    C27  - Lateral Proportional Step Frequency
    """    
    def __init__(self, side, step_freq=0.09):
        self.C1 =   0.03            
        self.C2 =   0.15
        self.C3 =   0.02
        self.C4 =   0.00*side
        self.C5 =   0.03
        self.C6 =   0.025            # talvez
        self.C7 =   0
        self.C8 =   0.02            # talvez
        self.C9 =   0.12
        self.Ct0 =  0
        self.Ct1 =  2.3876
        self.C10 =  0.17
        self.C11 =  0.25
        self.C12 =  0.1
        self.C13 =  0.05
        self.C14 =  0.015
        self.C15 =  0.2
        self.C16 =  0.05
        self.C17 =  0.035
        self.C18 =  0
        self.C19 =  0
        self.C20 =  0       #-0.07
        self.C21 =  3.5
        self.C22 =  0.0085
        self.C23 =  0.01
        self.C24 =  0.009
        self.C25 =  step_freq
        self.C26 =  0.008
        self.C27 =  0

        self.sigma = side

        # halt position
        self.p_halt_n = self.C1
        self.p_halt_leg_r = self.sigma*self.C2
        self.p_halt_leg_p = self.C3
        self.p_halt_foot_r = self.C4
        self.p_halt_foot_p = self.C5

        # [lin_vel_x, lin_vel_y, ang_vel_z]
        self.vel = np.array([0.0, 0.0, 0.0])

        # motion phase
        self.tau = 0            # left

        if side == 1:
            self.tau = np.pi    # right

        self.jump_n = 0

    def control_interface(self, vel_vector):
        """
        
        ----------
        vel_vector : array
            Desided walking velocity expressed in SI units 
            (lin_vel_x, lin_vel_y, ang_vel_z)
        
        Returns
        -------
        vel : tensor
            Desired velocity  
        phase: tensor
            Polenta
        """

        # make sure that the velocity vector is contained within a convex
        # region defined by p-norm
        c21_norm = np.linalg.norm(vel_vector, self.C21)
        if c21_norm > 1:
            vel_vector = vel_vector/c21_norm

        # increment/decrement the current velocity in steps
        self.vel[0] = self.vel[0] + max(-self.C22, min((vel_vector[0]-self.vel[0]), self.C22))
        self.vel[1] = self.vel[1] + max(-self.C23, min((vel_vector[1]-self.vel[1]), self.C23))
        self.vel[2] = self.vel[2] + max(-self.C24, min((vel_vector[2]-self.vel[2]), self.C24))

        # motion phase
        self.tau = self.tau + self.C25 + np.linalg.norm(self.vel[0])*self.C26 + np.linalg.norm(self.vel[1])*self.C27

        if self.tau > np.pi:
            self.tau = self.tau - 2*np.pi
        
        return self.vel, self.tau


    def motion_pattern(self, vel, phase):
        
        # leg lifting
        p_leg_lift = 0

        # leg support phase
        if phase <= 0:
            p_leg_lift = np.sin(phase)*(self.C6+self.C7*max(np.linalg.norm(vel[0]), np.linalg.norm(vel[1])))

        # leg swing phase
        else:
            p_leg_lift = np.sin(phase)*(self.C8+self.C9*max(np.linalg.norm(vel[0]), np.linalg.norm(vel[1])))

        # leg swing 
        gamma = 0 

        if (phase >= self.Ct0) and (phase < self.Ct1):
            gamma = np.cos((phase-self.Ct0)/(self.Ct1-self.Ct0)*np.pi)
        
        elif (phase >= self.Ct1) and (phase < np.pi):
            gamma = (2*(phase-self.Ct1)/(2*np.pi-self.Ct1+self.Ct0))-1

        else:
            gamma = (2*(phase+2*np.pi-self.Ct1)/(2*np.pi-self.Ct1+self.Ct0))-1

        p_legswing_pitch = 0

        if vel[0] >= 0:
            p_legswing_pitch = gamma*vel[0]*self.C10
        else:
            p_legswing_pitch = gamma*vel[0]*self.C11

        p_legswing_roll = -gamma*vel[1]*self.C12 - self.sigma*max(np.linalg.norm(vel[1])*self.C13, np.linalg.norm(vel[2])*self.C14)

        p_legswing_yaw = gamma*vel[2]*self.C15 - self.sigma*np.linalg.norm(vel[2])*self.C16

        # lateral hip swing
        tau_l = 0
        if phase < self.Ct0:
            tau_l = phase - self.Ct1 + 2*np.pi
        
        elif phase > self.Ct1:
            tau_l = phase - self.Ct1

        else:
            tau_l = 0

        tau_r = 0
        if (phase+np.pi) < self.Ct0:
            tau_r = phase - self.Ct1 + 3*np.pi
        
        elif (phase+np.pi) > self.Ct1:
            tau_r = phase - self.Ct1 + np.pi

        else:
            tau_r = 0

        p_hipswing = self.C17*(np.sin(tau_l*np.pi/(self.Ct0-self.Ct1+2*np.pi))-np.sin(tau_l*np.pi/(self.Ct0-self.Ct1+2*np.pi)))

        # leaning
        p_lean_pitch = 0

        if vel[0] >= 0:
            p_lean_pitch = vel[0]*self.C18
        else:
            p_lean_pitch = vel[0]*self.C19

        p_lean_roll = -vel[2]*np.linalg.norm(vel[0])*self.C20

        # motion pattern
        n = self.p_halt_n + p_leg_lift
        theta_leg_roll = self.p_halt_leg_r + p_hipswing + p_legswing_roll + p_lean_roll
        theta_leg_pitch = self.p_halt_leg_p + p_legswing_pitch + p_lean_pitch
        theta_leg_yaw = p_legswing_yaw
        theta_foot_roll = self.p_halt_foot_r
        theta_foot_pitch = self.p_halt_foot_p


        return n, (theta_leg_roll, theta_leg_pitch, theta_leg_yaw), (theta_foot_roll, theta_foot_pitch)


    def leg_interface(self, n, theta_leg, theta_foot):
        """
        theta_leg = (theta_roll, theta_pitch, theta_yaw)
        theta_foot = (theta_roll, theta_pitch)
        """

        theta_leg = np.array(theta_leg, dtype='float')
        theta_foot = np.array(theta_foot, dtype='float')

        lamb = np.arccos(1-n)

        rotation_mat = np.array([[np.cos(-theta_leg[2]),    -np.sin(-theta_leg[2])],
                                 [np.sin(-theta_leg[2]),    np.cos(-theta_leg[2])]])

        # [theta_leg_roll' , theta_leg_pitch' ]
        n_theta_leg = theta_leg[:2]
        n_theta_leg = np.matmul(rotation_mat, np.transpose(n_theta_leg))

        theta_hip_yaw = theta_leg[2]
        theta_hip_roll = n_theta_leg[0]
        theta_hip_pitch = -n_theta_leg[1] - lamb

        theta_knee = 2*lamb

        theta_ankle_pitch = theta_foot[1] - n_theta_leg[1] - lamb
        theta_ankle_roll = theta_foot[0] - n_theta_leg[0]

        #print(theta_hip_pitch, theta_knee, theta_ankle_pitch)

        return (theta_hip_roll, theta_hip_pitch, theta_hip_yaw), theta_knee, (theta_ankle_roll, theta_ankle_pitch)

    def update(self, vel_vector):
        
        vel, tau = self.control_interface(vel_vector)
        n, theta_leg, theta_foot = self.motion_pattern(vel, tau)
        theta_hip, theta_knee, theta_ankle = self.leg_interface(n, theta_leg, theta_foot)

        return theta_hip, theta_knee, theta_ankle

    def jump_charge(self):
        lamb = np.arccos(1-self.jump_n)

        theta_hip_yaw = 0
        theta_hip_roll = 0
        theta_hip_pitch = -0.02 - lamb

        theta_knee = 2*lamb

        theta_ankle_pitch = 0.06 - lamb
        theta_ankle_roll = 0

        if self.jump_n < 0.4:
            self.jump_n += 0.01

        return (theta_hip_roll, theta_hip_pitch, theta_hip_yaw), theta_knee, (theta_ankle_roll, theta_ankle_pitch)

    def jump_release(self):

        self.jump_n = 0

        theta_hip_yaw = 0
        theta_hip_roll = 0
        theta_hip_pitch = -0.7
        theta_ankle_pitch = -0.68
        theta_ankle_roll = 0
        theta_knee = 0

        return (theta_hip_roll, theta_hip_pitch, theta_hip_yaw), theta_knee, (theta_ankle_roll, theta_ankle_pitch)

    def zero_position(self):

        self.jump_n = 0

        theta_hip_yaw = 0
        theta_hip_roll = 0
        theta_hip_pitch = 0
        theta_ankle_pitch = 0
        theta_ankle_roll = 0
        theta_knee = 0

        return (theta_hip_roll, theta_hip_pitch, theta_hip_yaw), theta_knee, (theta_ankle_roll, theta_ankle_pitch)

def vel_cmd_callback(data):
    global robot_vel, robot_jump
    
    robot_vel = np.array([0.0,0.0,0.0])
    robot_vel[1] = data.linear.x*0.3                # left/right
    robot_vel[0] = (-1)*data.linear.y*0.3           # front/back
    robot_vel[2] = (-1)*data.angular.z*0.9          # yaw

    robot_jump = data.linear.z

if __name__ == "__main__":

    global robot_vel

    # initialize the node
    rospy.init_node("walk_node")
    
    # Subscribers
    rospy.Subscriber("/robotao/vel_cmd", Twist, vel_cmd_callback, queue_size=1)

    # Publishers
    left_hip_yaw =       rospy.Publisher("/robotao/left_hip_yaw_position/command", std_msgs.msg.Float64, queue_size=1)
    left_hip_roll =      rospy.Publisher("/robotao/left_hip_roll_position/command", std_msgs.msg.Float64, queue_size=1)
    left_hip_pitch =     rospy.Publisher("/robotao/left_hip_pitch_position/command", std_msgs.msg.Float64, queue_size=1)
    left_knee_pitch =    rospy.Publisher("/robotao/left_knee_pitch_position/command", std_msgs.msg.Float64, queue_size=1)
    left_ankle_pitch =   rospy.Publisher("/robotao/left_ankle_pitch_position/command", std_msgs.msg.Float64, queue_size=1)
    left_ankle_roll =    rospy.Publisher("/robotao/left_ankle_roll_position/command", std_msgs.msg.Float64, queue_size=1)

    right_hip_yaw =       rospy.Publisher("/robotao/right_hip_yaw_position/command", std_msgs.msg.Float64, queue_size=1)
    right_hip_roll =      rospy.Publisher("/robotao/right_hip_roll_position/command", std_msgs.msg.Float64, queue_size=1)
    right_hip_pitch =     rospy.Publisher("/robotao/right_hip_pitch_position/command", std_msgs.msg.Float64, queue_size=1)
    right_knee_pitch =    rospy.Publisher("/robotao/right_knee_pitch_position/command", std_msgs.msg.Float64, queue_size=1)
    right_ankle_pitch =   rospy.Publisher("/robotao/right_ankle_pitch_position/command", std_msgs.msg.Float64, queue_size=1)
    right_ankle_roll =    rospy.Publisher("/robotao/right_ankle_roll_position/command", std_msgs.msg.Float64, queue_size=1)

    left_leg = Gait(side=1, step_freq=0.09)
    right_leg = Gait(side=-1, step_freq=0.09)

    print("Starting walking ...")
    #rospy.sleep(5)

    r = rospy.Rate(100)

    i = 0

    timer = time.time()
    state = "idle"

    #while not rospy.is_shutdown():
    while True:      

        if state == "idle":
            
            hip, knee, ankle = left_leg.zero_position()
            left_hip_yaw.publish(hip[2])
            left_hip_roll.publish(hip[0])
            left_hip_pitch.publish(hip[1])
            left_knee_pitch.publish(knee)
            left_ankle_pitch.publish(ankle[1])
            left_ankle_roll.publish(ankle[0])

            right_hip_yaw.publish(hip[2])
            right_hip_roll.publish(hip[0])
            right_hip_pitch.publish(hip[1])
            right_knee_pitch.publish(knee)
            right_ankle_pitch.publish(ankle[1])
            right_ankle_roll.publish(ankle[0])

            if robot_jump == 1:
                state = "jump_charge"
                timer = 0
                continue

            elif ((np.linalg.norm(robot_vel[0]) > 0.1) or 
                 (np.linalg.norm(robot_vel[1]) > 0.1) or 
                 (np.linalg.norm(robot_vel[2]) > 0.1)):
                state = "walking"
                timer = 0
                continue
                
        elif state == "jump_charge":
            print("Charging jump")
            hip, knee, ankle = left_leg.jump_charge()
            left_hip_yaw.publish(hip[2])
            left_hip_roll.publish(hip[0])
            left_hip_pitch.publish(hip[1])
            left_knee_pitch.publish(knee)
            left_ankle_pitch.publish(ankle[1])
            left_ankle_roll.publish(ankle[0])

            right_hip_yaw.publish(hip[2])
            right_hip_roll.publish(hip[0])
            right_hip_pitch.publish(hip[1])
            right_knee_pitch.publish(knee)
            right_ankle_pitch.publish(ankle[1])
            right_ankle_roll.publish(ankle[0])

            timer += 1
            if timer > 200:
                state = "jump_release"
                timer = 0
                continue

        elif state == "jump_release":
            print("Charging release")
            hip, knee, ankle = left_leg.jump_release()
            left_hip_yaw.publish(hip[2])
            left_hip_roll.publish(hip[0])
            left_hip_pitch.publish(hip[1])
            left_knee_pitch.publish(knee)
            left_ankle_pitch.publish(ankle[1])
            left_ankle_roll.publish(ankle[0])

            right_hip_yaw.publish(hip[2])
            right_hip_roll.publish(hip[0])
            right_hip_pitch.publish(hip[1])
            right_knee_pitch.publish(knee)
            right_ankle_pitch.publish(ankle[1])
            right_ankle_roll.publish(ankle[0])

            timer += 1
            if timer > 50:
                state = "idle"
                continue

        elif state == "walking":

            print(robot_vel)

            left_hip, left_knee, left_ankle = left_leg.update(robot_vel)
            left_hip_yaw.publish(left_hip[2])
            left_hip_roll.publish(left_hip[0])
            left_hip_pitch.publish(left_hip[1])
            left_knee_pitch.publish(left_knee)
            left_ankle_pitch.publish(left_ankle[1])
            left_ankle_roll.publish(left_ankle[0])

            right_hip, right_knee, right_ankle = right_leg.update(robot_vel)
            right_hip_yaw.publish(right_hip[2])
            right_hip_roll.publish(right_hip[0])
            right_hip_pitch.publish(right_hip[1])
            right_knee_pitch.publish(right_knee)
            right_ankle_pitch.publish(right_ankle[1])
            right_ankle_roll.publish(right_ankle[0])

            if robot_jump == 1:
                state = "jump_charge"
                timer = 0
                continue

        r.sleep()
        
        
    

    