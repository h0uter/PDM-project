import numpy as np
from pyquaternion import Quaternion

"""
Drone overview

A/+X   B/+Y
 \\   //
  \\ //
    H
  // \\
 //   \\
C/-Y    D/-X
"""

class Drone:

    def __init__(self, s0, s_dot0, dt, m, I, L, kf, km, max_motor_torque, max_motor_omega, propellor_inertia):
        self.s = np.asarray(s0)
        self.s_dot = np.asarray(s_dot0)
        self.m = m
        self.I = I
        self.L = L
        self.kf = kf
        self.km = km
        self.dt = dt

        self.motors = {'A': (Motor(max_motor_torque, max_motor_omega, propellor_inertia, self.kf, self.km, 1, self.dt),     np.asarray([L, 0, 0])),
                       'B': (Motor(max_motor_torque, max_motor_omega, propellor_inertia, self.kf, self.km, -1, self.dt),    np.asarray([0, L, 0])),
                       'C': (Motor(max_motor_torque, max_motor_omega, propellor_inertia, self.kf, self.km, -1, self.dt),    np.asarray([0, -L, 0])),
                       'D': (Motor(max_motor_torque, max_motor_omega, propellor_inertia, self.kf, self.km, 1, self.dt),     np.asarray([-L, 0, 0]))}
        
        self.motor_commands = np.zeros(4,)
        self.motor_speeds = np.zeros(4,)
        self.motor_thrust_matrix = np.zeros((4, 3))

    def update(self):
        total_force_vector = np.zeros(3,)
        total_moment_vector = np.zeros(3,)
        T = self.get_transformation_matrix(*self.s[3:])[:3, :3]

        #get values for motor in each frame
        for idx, motor_arm_pair in enumerate(self.motors.values()):
            motor = motor_arm_pair[0]
            arm = motor_arm_pair[1]

            motor.set_motor_command(self.motor_commands[idx])
            force, moment = motor.get_thrust_values()

            force_vector = np.asarray([0, 0, force])
            moment_vector = np.asarray([0, 0, moment])
            combined_moment_vector = np.cross(arm, force_vector) + moment_vector

            self.motor_thrust_matrix[idx] = np.dot(T, force_vector) / 100 + np.dot(T, arm) + self.s[:3]
            self.motor_speeds[idx] = motor.get_motor_speed()

            total_force_vector += force_vector
            total_moment_vector += combined_moment_vector

        acc_vector_local = total_force_vector / self.m

        rot_velocities_local = np.dot(np.linalg.inv(T), self.s_dot[3:])
        ang_acc_vector_local = (total_moment_vector - np.cross(rot_velocities_local, self.I * rot_velocities_local)) / self.I
    
        acc_vector_global = np.dot(T, acc_vector_local) + np.asarray([0, 0, -9.81])

        ang_acc_vector_global = np.dot(T, ang_acc_vector_local)
        acc_vector = np.append(acc_vector_global, ang_acc_vector_global)

        self.s_dot = np.add(self.s_dot, acc_vector * self.dt)
        self.s = np.add(self.s, self.s_dot * self.dt)
        
    def get_transformation_matrix(self, pitch, roll, yaw):
        q1 = Quaternion(axis=[1., 0., 0.], angle=pitch)
        q2 = Quaternion(axis=[0., 1., 0.], angle=roll)
        q3 = Quaternion(axis=[0., 0., 1.], angle=yaw)

        qtot = q3 * q2 * q1
        T = qtot.transformation_matrix
        T[:-1, -1] = self.s[:3]
        return T 
    
    def eye_of_god(self):
        return np.append(self.s, self.s_dot) #x, y, z, roll, pitch, yaw, vx, vy, vz, vpitch, vroll, vyaw 
    
    def set_motor_commands(self, motor_commands): 
        self.motor_commands = np.asarray(motor_commands) #takes 4D array, using order [A, B, C, D]
    
    def get_drone(self):
        #return five points, center and the position of all four motors
        drone = np.asarray([self.s[:3]])
        T = self.get_transformation_matrix(*self.s[3:])
        for _, arm in self.motors.values():
            drone = np.vstack((drone, np.dot(T, np.append(arm, 1.))[:-1]))

        return drone

    def get_thrust_vectors(self):
        return self.motor_thrust_matrix

    def get_motor_speeds(self):
        return self.motor_speeds

class Motor:

    def __init__(self, max_torque, max_omega, propellor_inertia, kf, km, direction, dt):
        self.max_torque = max_torque
        self.max_omega = max_omega
        self.propellor_inertia = propellor_inertia

        self.kf = kf
        self.km = km
        self.direction = direction
        self.dt = dt

        self.omega = 0

        self.t_acc = 0
        self.omega_ref = 0

    def set_motor_command(self, omega_ref):
        self.omega_ref = np.clip(omega_ref, -self.max_omega * 0.8, self.max_omega * 0.8) * self.direction
    
    def calc_max_angular_acceleration(self):
        max_torque = max(abs(self.omega) * (-self.max_torque / self.max_omega) + self.max_torque - (self.km * self.omega**2), 0.1 * self.max_torque)
        return max_torque / self.propellor_inertia

    def update(self):
        vel_error = self.omega_ref - self.omega
        ang_acc_req = vel_error / self.dt

        max_angular_acceleration = self.calc_max_angular_acceleration()

        if abs(ang_acc_req) <= max_angular_acceleration:
            self.omega = self.omega_ref
            self.t_acc = -ang_acc_req * self.propellor_inertia
        else:
            self.omega += max_angular_acceleration * np.sign(vel_error) * self.dt
            self.t_acc = max_angular_acceleration * -np.sign(vel_error) * self.propellor_inertia
        
    def get_thrust_values(self):
        self.update()

        moment_motor = self.km * self.omega**2 * self.direction + self.t_acc
        force_motor = self.kf * self.omega**2
        return (force_motor, moment_motor)
    
    def get_motor_speed(self):
        return self.omega