import numpy as np
from pyquaternion import Quaternion


"""
Drone overview

A      B
 \\   //
  \\ //
    H
  // \\
 //   \\
C       D
"""

class Drone:

    def __init__(self, s0, s_dot0, dt, l=[0.2, 0.2, 0.2, 0.2], m=1.8, Is=[0.8, 0.8, 0.8]):
        self.s = np.asarray(s0) #[0, 0, 0, 0, 0, 0] x, y, z, pitch, roll, yaw
        self.s_dot = np.asarray(s_dot0) #ds/dt
        self.s_ddot = np.asarray([0, 0, 0, 0, 0, 0])
        self.m = m
        self.dt = dt

        self.I = np.asarray([[Is[0], 0, 0], [0, Is[1], 0], [0, 0, Is[2]]])

        dr = [1, 1, -1, -1]

        self.motor_pos = np.asarray([[0, l[0], 0], [0, -l[2], 0], [l[1], 0, 0], [-l[3], 0, 0]]) # motor positions are [A, D, B, C]
        self.motors = [Motor(0.2, 150, 10, 0.2, dr[i], dt) for i in range(4)]

        self.motor_commands = np.asarray([0, 0, 0, 0])
        
    def update(self):
        T_M = self.get_transformation_matrix(*self.s[3:])[:3, :3]
        
        F_tot, T_tot = np.zeros(3), np.zeros(3)
        #calculate motor forces
        for motor, arm, command in zip(self.motors, self.motor_pos, self.motor_commands):
            motor.set_omega_command(command)
            F, T = motor.get_thrust_values()

            F_global = np.dot(T_M, F)
            T_global = np.add(np.dot(T_M, T), np.cross(np.dot(T_M, arm), np.dot(T_M, F)))

            F_tot = np.add(F_tot, F_global)
            T_tot = np.add(T_tot, T_global)

        ang_acc = np.dot(np.linalg.inv(self.I), T_tot)
        acc = np.add(F_global / self.m, np.asarray([0, 0, -9.81])) 

        #for now no controllers or motors
        a = np.append(acc, ang_acc)
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
    
    def IMU(self):
        return np.append(self.s_dot, self.s_ddot) #vx, vy, vz, vpitch, vroll, vyaw, dvx, dvy, dvz, dvpitch, dvroll, dvyaw
    
    def gyro(self):
        return self.s_dot[3:] #pitch, roll, yaw
    
    def altimeter(self):
        return self.s[2] #altitude
    
    def eye_of_god(self):
        return np.append(self.s, self.s_dot) #x, y, z, roll, pitch, yaw, vx, vy, vz, vpitch, vroll, vyaw 
    
    def set_motor_commands(self, motor_commands): 
        self.motor_commands = np.asarray(motor_commands) #takes 4D array, using order [A, D, B, C]

class Motor:
    def __init__(self, thurst_coefficient, max_omega, max_torque, inertia, direction, dt):
        self.k = thurst_coefficient
        self.max_rot_vel = max_omega
        self.inertia = inertia
        self.direction = direction
        self.dt = dt

        self.max_angular_acceleration = max_torque / inertia

        self.omega = 0
        self.omega_ref = 0
        self.t = 0
    
    def set_omega_command(self, omega_ref):
        if abs(self.omega_ref) > self.max_rot_vel:
            self.omega_ref = self.max_rot_vel * np.sign(omega_ref) * self.direction
        else:
            self.omega_ref = omega_ref * self.direction
        
    def update(self):
        #model is assuming no required torque for constant velocity, model is also assuming max torque is constant over entire operating range of motor
        vel_error = self.omega_ref - self.omega
        ang_acc_req = vel_error / self.dt

        if (np.abs(ang_acc_req) <= self.max_angular_acceleration):
            self.omega = self.omega_ref
            self.t = -ang_acc_req * self.inertia
        else:
            self.omega += self.max_angular_acceleration * np.sign(vel_error) * self.dt
            self.t = self.max_angular_acceleration * -np.sign(vel_error) * self.inertia
        
    def get_thrust_values(self):
        self.update()
        #thrust is modelled as linear relationship with RPM
        F = np.asarray([0, 0, self.k * self.omega])
        T = np.asarray([0, 0, self.t])
        return F, T
