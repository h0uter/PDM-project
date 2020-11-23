import numpy as np
from numpy.core.arrayprint import dtype_is_implied

class Controller:
    def __init__(self, drone):
        self.drone = drone
        self.target = np.array([8, 5, 5]) # x, y, z

        self.Kp_z = 30
        self.Kd_z = 14

        self.Kp = 1
        self.Kd = 0.5

        self.Kp_a = 1
        self.Kd_a = 0.5

        self.prev_error = np.array([0, 0, 0])
        self.prev_angle_error = np.array([0, 0])
        self.dt = 0.01
        self.time = 0

    def update(self):
        state = self.drone.eye_of_god()
        # print("state", state)

        # rotation matrix to transform global pos error into local pos error
        rot_inv = np.linalg.inv(self.drone.get_transformation_matrix(state[3], state[4], state[5]))[:3, :3]
        #print(rot_inv)

        pos_error = rot_inv@(self.target - state[0:3])

        d_error = (pos_error - self.prev_error)/self.dt

        print('Roll, Pitch, Yaw: ', state[3:6])
        print('Pos: ', state[0:3])

        g = 9.81
        # total_force = self.drone.m * g + self.Kp_z*pos_error[2] + self.Kd_z*d_error[2]
        # TODO: verify. other drone code had everythig times mass 
        # https://github.com/AtsushiSakai/PythonRobotics/blob/40df009180e7541e6bc85e7f455d7d973a064aed/AerialNavigation/drone_3d_trajectory_following/drone_3d_trajectory_following.py#L83
        total_force = self.drone.m * (g + self.Kp_z*pos_error[2] + self.Kd_z*d_error[2])

        #  Position to angle PD
        angle_reference = self.Kp*pos_error[0:2] + self.Kd*d_error[0:2]

        # # test with constant angle ref
        # angle_reference = np.array([0, -np.pi/10])

        # limit drone angle to 45deg
        angle_reference = np.maximum(np.minimum(angle_reference, np.pi/4), -np.pi/4)
        print("Angle ref:", angle_reference)

        # Angle to forces PD
        angle_error = angle_reference - state[3:5] #np.array([state[4], state[3]])
        angle_d_error = (angle_error - self.prev_angle_error)/self.dt

        print("angle_error:", angle_error)

        d_angle = self.Kp_a*angle_error + self.Kd_a*angle_d_error

        forces = np.ones((4))*total_force/4

        # # pitch
        # forces[2] -= d_angle[0]
        # forces[3] += d_angle[0]

        # # roll
        # forces[0] -= d_angle[1]
        # forces[1] += d_angle[1]

        # pitch
        forces[2] -= d_angle[0]
        forces[3] += d_angle[0]

        # roll
        forces[0] -= d_angle[1]
        forces[1] += d_angle[1]


        forces = np.maximum(0, forces)

        #forces = np.array([0, 0, 10, 10])

        print("forces:", forces)

        # TODO: better way of sharing drone parameters
        omega = np.sqrt(forces/self.drone.motors[0].k)
        #print('Setpoint: ', omega)
        self.drone.set_motor_commands(omega)

        self.prev_error = pos_error
        self.prev_angle_error = angle_error


        # new target point
        self.time += self.dt
        # print("time:", self.time)

        if self.time > 3:
            self.set_target(np.array([5, 10, 5]))


    def set_target(self, pos):
        self.target = pos
