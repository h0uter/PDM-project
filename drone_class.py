import numpy as np
from pyquaternion import Quaternion

class Drone:

    def __init__(self, s0, s_dot0, dt, l=[0.2, 0.2, 0.2, 0.2], m=0.8):
        self.s = np.asarray(s0) #[0, 0, 0, 0, 0, 0] x, y, z, pitch, roll, yaw
        self.s_dot = np.asarray(s_dot0) #ds/dt
        self.m = m
        self.dt = dt

        self.motor_pos = np.asarray([[0, l[0], 0], [0, -l[2], 0], [l[1], 0, 0], [-l[3], 0, 0]])

        self.t = 0
    
    def update(self):
        T_M = self.get_transformation_matrix(*self.s[3:])[:3, :3]
        
        #calculate motor forces
        f_tot, t_tot = np.asarray([0,0,0]), np.asarray([0,0,0])

        for i, m_pos in enumerate(self.motor_pos):
            #for now just use sines
            motor_force = np.dot(T_M, np.asarray([0, 0, np.sin(self.t + i*0.25*np.pi) * 2]))
            f_tot = np.add(f_tot, motor_force)
            torque = np.cross(np.dot(T_M, m_pos), np.dot(T_M, motor_force))
            t_tot = np.add(t_tot, torque)

        I = np.eye(3) * 0.8

        rot_acc = np.dot(np.linalg.inv(I), t_tot)
        acc = np.add(f_tot, np.asarray([0, 0, -9.81 * self.m])) / self.m

        self.t += self.dt

        #for now no controllers or motors
        a = np.append(acc, rot_acc)
        self.s_dot = np.add(self.s_dot, a * self.dt)
        self.s = np.add(self.s, self.s_dot * self.dt)

        if(self.s[2] <=0):
            self.s_dot = np.asarray([self.s_dot[0], self.s_dot[1], -self.s_dot[2], self.s_dot[3], self.s_dot[4], self.s_dot[5]])
        
        if(self.s[1] <=0.0 or self.s[1] >= 10.0):
            self.s_dot = np.asarray([self.s_dot[0], -self.s_dot[1], self.s_dot[2], self.s_dot[3], self.s_dot[4], self.s_dot[5]])
        
        if(self.s[0] <=0.0 or self.s[0] >=10.0):
            self.s_dot = np.asarray([-self.s_dot[0], self.s_dot[1], self.s_dot[2], self.s_dot[3], self.s_dot[4], self.s_dot[5]])

    def get_transformation_matrix(self, pitch, roll, yaw):
        q1 = Quaternion(axis=[1., 0., 0.], angle=pitch)
        q2 = Quaternion(axis=[0., 1., 0.], angle=roll)
        q3 = Quaternion(axis=[0., 0., 1.], angle=yaw)

        qtot = q3 * q2 * q1
        T = qtot.transformation_matrix
        T[:-1, -1] = self.s[:3]
        return T

    def get_drone(self):
        #return five points, center and the position of all four motors
        drone = np.asarray([self.s[:3]])
        T = self.get_transformation_matrix(*self.s[3:])
        for motor in self.motor_pos:
            drone = np.vstack((drone, np.dot(T, np.append(motor, 1.))[:-1]))
        
        return drone

class Motor:

    def __init__(self, thurst_coefficient, max_rot_vel, max_torque, propellor_intertia, direction, dt):
        self.k = thurst_coefficient
        self.max_rot_vel = max_rot_vel
        self.propellor_intertia = propellor_intertia
        self.max_torque = max_torque
        self.dir = direction
        self.dt = dt

        self.omega = 0
        self.omega_ref = 0
    
    def set_omega_command(self, ref_omega):
        self.ref_omega = ref_omega
    
    def update(self):
        vel_error = self.omega_ref - self.omega
        delta_omega_req = vel_error / self.dt
        max_delta_omega = self.max_torque / self.propellor_intertia

        if (delta_omega_req >= max_delta_omega):
            self.omega = self.omega_ref
        

    


        