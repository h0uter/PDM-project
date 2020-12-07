import numpy as np
from numpy.core.arrayprint import dtype_is_implied
from numpy.core.numeric import roll
import random

class Controller:
    def __init__(self, drone):
        self.drone = drone
        self.command_vector = [200, 250, 120, 250]

        # TODO: merge target orientation and position
        self.target = np.array([4, 4, 2.5]) # x, y, z
        self.target_orientation = np.array([0, 0, 0])

        # Height gain
        self.Kp_z = 10
        self.Kd_z = 8

        # Position -> Angle PID
        self.Kp = 0.5
        self.Kd = 0.4

        # Angle -> Thrust Delta PID
        self.Kp_a = 10
        self.Kd_a = 5

        self.prev_local_pos_error = np.array([0, 0, 0])
        self.prev_orientation_error = np.array([0, 0, 0])
        self.dt = 0.01
        self.time = 0

        self.MAX_ROT = np.pi/8
        self.MAX_D_POS_ERROR = 10 # m/s
        self.MAX_D_ORIENT_ERROR = np.pi/5 # rad/s

    def update(self):

        state = self.drone.eye_of_god()

        print('x, y, z: ', state[0:3])

        """OUTER LOOP PITCH & ROLL POSITION CONTROLLER"""
        # x
        # y
        # by feeding into roll and pitch

        # rotation matrix to transform global pos error into local pos error
        yaw = state[5]
        # rotation matrix yaw only
        rot = np.linalg.inv(np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]))

        # world to body conversion of error
        global_pos_error = self.target - state[0:3]
        
        local_pos_error = rot@global_pos_error
        print("global_pos_error:", global_pos_error, " local pos error: ", local_pos_error)

        d_local_pos_error = np.maximum(np.minimum((local_pos_error - self.prev_local_pos_error)/self.dt, self.MAX_D_POS_ERROR), -self.MAX_D_POS_ERROR)

        angle_reference = self.Kp * \
            local_pos_error[0:2] + self.Kd*d_local_pos_error[0:2]

        roll_reference = -angle_reference[1]
        pitch_reference = angle_reference[0]

        # rotate for me baby
        yaw_reference = 0

        """YAW + INNER LOOP PITCH & ROLL ORIENTATION CONTROLLER"""
        # roll
        # pitch 

        # limit target orientation to 45deg
        roll_reference = np.maximum(
            np.minimum(roll_reference, self.MAX_ROT),  -self.MAX_ROT)
        pitch_reference = np.maximum(
            np.minimum(pitch_reference, self.MAX_ROT),  -self.MAX_ROT)

        self.target_orientation = np.array(
            [roll_reference, pitch_reference, yaw_reference])


        orientation_error = self.target_orientation - rot@state[3:6]

        print('[Roll, Pitch, Yaw] state: ', rot@state[3:6])
        print('[Roll, Pitch, Yaw] target: ', self.target_orientation)
        print("[Roll, Pitch, Yaw] error:", orientation_error)


        d_orientation_error = np.maximum(np.minimum((orientation_error -
                               self.prev_orientation_error)/self.dt, self.MAX_D_ORIENT_ERROR), -self.MAX_D_ORIENT_ERROR)


        d_orientation = self.Kp_a*orientation_error + self.Kd_a*d_orientation_error

        roll_cmd = -d_orientation[0]/self.drone.motor_pos[0][1]  # + moves in +y
        pitch_cmd = d_orientation[1]/self.drone.motor_pos[2][0] # + moves in +x
        yaw_cmd = d_orientation[2]

        """THRUST CONTROLLER"""
        # thrust

        g = 9.81

        # trust controller
        thrust_cmd = self.drone.m * \
            (g + self.Kp_z*local_pos_error[2] + self.Kd_z*d_local_pos_error[2])/4

        print('Thrust, roll, pitch, yaw command:', thrust_cmd, roll_cmd, pitch_cmd, yaw_cmd)

        """MOTOR MIXING ALGORITHM"""

        motor_A = thrust_cmd - roll_cmd - yaw_cmd
        motor_B = thrust_cmd - pitch_cmd + yaw_cmd
        motor_C = thrust_cmd + pitch_cmd + yaw_cmd
        motor_D = thrust_cmd + roll_cmd - yaw_cmd

        # motor cmd array as [ADBC]
        motor_forces = np.array([motor_A, motor_D, motor_B, motor_C])
        print("motor force pre:", motor_forces)

        motor_forces = np.maximum(1, motor_forces)


        print("motor force post:", motor_forces)

        # TODO: better way of sharing drone parameters
        self.command_vector = np.sqrt(motor_forces/self.drone.motors[0].k)

        self.drone.set_motor_commands(self.command_vector)

        self.prev_local_pos_error = local_pos_error
        self.prev_orientation_error = orientation_error

        self.time += self.dt
        # print("time:", self.time)

        # if self.time > 3:
        #     self.set_target(np.array([5, 10, 5]))


    def set_target(self, pos):
        self.target = pos

    def get_command_vector(self):
        return self.command_vector