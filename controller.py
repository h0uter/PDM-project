import numpy as np
from numpy.core.arrayprint import dtype_is_implied
from numpy.core.numeric import roll
import random

class Controller:
    def __init__(self, drone):
        self.drone = drone
        self.command_vector = [200, 250, 120, 250]

        # TODO: merge target orientation and position
        self.target = np.array([4, 4, 5]) # x, y, z
        self.target_orientation = np.array([0, 0, 0])

        self.Kp_z = 10
        self.Kd_z = 8

        self.Kp = 0.1
        self.Kd = 0.1

        self.Kp_a = 2
        self.Kd_a = 0.5

        self.prev_local_pos_error = np.array([0, 0, 0])
        self.prev_orientation_error = np.array([0, 0, 0])
        self.dt = 0.01
        self.time = 0

        self.MAX_ROT = np.pi/8

    def update(self):
        # self.command_vector = [random.randint(100, 500), random.randint(100, 500), random.randint(100, 500), random.randint(100, 500)]
        # self.drone.set_motor_commands(self.command_vector)
    
        state = self.drone.eye_of_god()
        # print("state", state)

        print('x, y, z: ', state[0:3])

        """OUTER LOOP PITCH & ROLL POSITION CONTROLLER"""
        # x
        # y
            # by feeding into roll and pitch

        # rotation matrix to transform global pos error into local pos error
        rot_inv = np.linalg.inv(self.drone.get_transformation_matrix(state[3], state[4], state[5]))[:3, :3]
        #print(rot_inv)

        yaw = state[5]
        # rotation matrix yaw only
        rot = np.linalg.inv(np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]))
        
        #rot

        # world to body conversion of error
        global_pos_error = self.target - state[0:3]
        # local_pos_error = rot_inv@global_pos_error
        local_pos_error = rot@global_pos_error
        print("global_pos_error:", global_pos_error, " local pos error: ", local_pos_error)

        d_local_pos_error = (local_pos_error - self.prev_local_pos_error)/self.dt

        angle_reference = self.Kp * \
            local_pos_error[0:2] + self.Kd*d_local_pos_error[0:2]

        roll_reference = -angle_reference[1]
        pitch_reference = angle_reference[0]

        # rotate for me baby
        # yaw_reference = np.pi
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


        d_orientation_error = (orientation_error -
                               self.prev_orientation_error)/self.dt
        d_orientation = self.Kp_a*orientation_error + self.Kd_a*d_orientation_error

        roll_cmd = -d_orientation[0]/self.drone.motor_pos[0][1]  # + moves in +y
        pitch_cmd = d_orientation[1]/self.drone.motor_pos[2][0] # + moves in +x
        yaw_cmd = d_orientation[2]

        """THRUST CONTROLLER"""
        # thrust

        g = 9.81
        # total_force = self.drone.m * g + self.Kp_z*local_pos_error[2] + self.Kd_z*d_local_pos_error[2]
        # TODO: verify. other drone code had everythig times mass
        # https://github.com/AtsushiSakai/PythonRobotics/blob/40df009180e7541e6bc85e7f455d7d973a064aed/AerialNavigation/drone_3d_trajectory_following/drone_3d_trajectory_following.py#L83
        # total_force = self.drone.m * (g + self.Kp_z*local_pos_error[2] + self.Kd_z*d_local_pos_error[2])

        # trust controller
        thrust_cmd = self.drone.m * \
            (g + self.Kp_z*local_pos_error[2] + self.Kd_z*d_local_pos_error[2])

        # thrust_cmd = self.drone.m * g + self.Kp_z*local_pos_error[2] + self.Kd_z*d_local_pos_error[2]

        print('Thrust, roll, pitch, yaw command:', thrust_cmd, roll_cmd, pitch_cmd, yaw_cmd)

        """MOTOR MIXING ALGORITHM"""

        # motor_A = thrust_cmd + yaw_cmd
        # motor_B = thrust_cmd - yaw_cmd
        # motor_C = thrust_cmd - yaw_cmd
        # motor_D = thrust_cmd + yaw_cmd

        motor_A = thrust_cmd - roll_cmd - yaw_cmd
        motor_B = thrust_cmd - pitch_cmd + yaw_cmd
        motor_C = thrust_cmd + pitch_cmd + yaw_cmd
        motor_D = thrust_cmd + roll_cmd - yaw_cmd
        # motor cmd array as [ADBC]
        motor_forces = np.array([motor_A, motor_D, motor_B, motor_C])
        print("motor force pre:", motor_forces)


        # TODO: drone does not know how to deal with negative forces, leading to zero motor input after reaching target.
        # causing crash
        motor_forces = np.maximum(1, motor_forces)

        # std motor speed. add or substract difference

        # if motor_forces.any() < 0:
        #     motor_forces

        # motor_force = np.maximum(
        #     np.minimum(target_orientation, np.pi/8),  -np.pi/8)

        print("motor force post:", motor_forces)

        # TODO: better way of sharing drone parameters
        # omega = np.sqrt(forces/self.drone.motors[0].k)


        # omega = np.sqrt(motor_forces/self.drone.motors[0].k)
        self.command_vector = np.sqrt(motor_forces/self.drone.motors[0].k)

        # print('Setpoint: ', omega)
        # self.drone.set_motor_commands(omega)
        self.drone.set_motor_commands(self.command_vector)

        self.prev_local_pos_error = local_pos_error
        # self.prev_angle_error = angle_error
        self.prev_orientation_error = orientation_error


        # new target point
        self.time += self.dt
        # print("time:", self.time)

        # if self.time > 3:
        #     self.set_target(np.array([5, 10, 5]))


    def set_target(self, pos):
        self.target = pos

    def get_command_vector(self):
        return self.command_vector