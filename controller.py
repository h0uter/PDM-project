import numpy as np
from numpy.core.arrayprint import dtype_is_implied
from numpy.core.numeric import roll
import random

class Controller:
    def __init__(self, drone):
        self.drone = drone
        self.command_vector = [200, 200, 200, 200]

        # TODO: merge target orientation and position
        self.internal_target = np.array([0, 5, 2.5]) # x, y, z
        self.target = np.array([0, 5, 2.5]) # x, y, z
        self.target_orientation = np.array([0, 0, 0])

        # Disables derivative errors when a new target is set
        self.new_target = True

        # Height gain
        self.Kp_z = 10
        self.Kd_z = 5

        # Position -> Angle PID
        self.Kp = 0.25
        self.Kd = 0.1

        # Angle -> Thrust Delta PID
        self.Kp_a = 5.0
        self.Kd_a = 0.3

        # Yaw Angle -> Thrust Delta PID
        self.Kp_ya = 2#0.2
        self.Kd_ya = 2#0.2
        
        self.dt = 0.01
        self.time = 0

        # Drone control constants
        self.MAX_ROT = np.pi/8 # rad
        self.MAX_D_POS_ERROR = 10 # m/s
        self.MAX_D_ORIENT_ERROR = np.pi/5 # rad/s
        self.VEL_ONLY_DERIVATIVE = True

        # Target constants
        self.TARGET_REACHED_THRESHOLD = 0.3 # m
        self.CHASE_DISTANCE = 0.5 # m

        # Path following settings
        self.path = None
        self.current_path_node = 0
        self.path_finished = False
        self.chase = False

        self.prev_local_pos_error = np.array([0, 0, 0])
        self.prev_orientation_error = np.array([0, 0, 0])

    def update(self):

        if self.path is not None:
            if (self.target != self.path[self.current_path_node, :]).any():
                self.set_target(self.path[self.current_path_node, :])

        state = self.drone.eye_of_god()

        """OUTER LOOP PITCH & ROLL POSITION CONTROLLER"""

        pos = state[0:3]

        # rotation matrix to transform global pos error into local pos error
        yaw = state[5]
        # rotation matrix yaw only
        rot = np.linalg.inv(np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]))

        # Calculate the chase position if enabled
        if self.chase:
            self.internal_target = self.chase_target(pos, self.target, self.path[self.current_path_node-1, :])
        else:
            self.internal_target = self.target

        # World to body conversion of error
        global_pos_error = self.internal_target - pos
        
        local_pos_error = rot@global_pos_error

        if self.VEL_ONLY_DERIVATIVE:
            d_local_pos_error = -state[6:9]
        else:
            d_local_pos_error = (local_pos_error - self.prev_local_pos_error)/self.dt

        d_local_pos_error = np.maximum(np.minimum(d_local_pos_error, self.MAX_D_POS_ERROR), -self.MAX_D_POS_ERROR)

        if not self.VEL_ONLY_DERIVATIVE and self.new_target:
            d_local_pos_error = np.zeros((3))

        # Angle PD
        angle_reference = self.Kp * local_pos_error[0:2] + self.Kd*d_local_pos_error[0:2]

        roll_reference = -angle_reference[1]
        pitch_reference = angle_reference[0]

        # Don't rotate for me baby
        yaw_reference = 0

        """YAW + INNER LOOP PITCH & ROLL ORIENTATION CONTROLLER"""

        # limit target orientation to MAX_ROT
        roll_reference = np.maximum(
            np.minimum(roll_reference, self.MAX_ROT),  -self.MAX_ROT)
        pitch_reference = np.maximum(
            np.minimum(pitch_reference, self.MAX_ROT),  -self.MAX_ROT)
        yaw_reference = np.maximum(
            np.minimum(yaw_reference, self.MAX_ROT),  -self.MAX_ROT)

        self.target_orientation = np.array(
            [roll_reference, pitch_reference, yaw_reference])


        orientation_error = self.target_orientation - rot@state[3:6]

        d_orientation_error = np.maximum(np.minimum((orientation_error -
                               self.prev_orientation_error)/self.dt, self.MAX_D_ORIENT_ERROR), -self.MAX_D_ORIENT_ERROR)
        
        if self.new_target:
            d_orientation_error = np.zeros((3))

        # Orientation PDs
        d_orientation = self.Kp_a*orientation_error[0:2] + self.Kd_a*d_orientation_error[0:2]
        d_orientation = np.append(d_orientation, self.Kp_ya*orientation_error[2] + self.Kd_ya*d_orientation_error[2])

        roll_cmd = -d_orientation[0]/self.drone.L  # + moves in +y
        pitch_cmd = d_orientation[1]/self.drone.L # + moves in +x
        yaw_cmd = d_orientation[2]*self.drone.kf/self.drone.km/2

        """THRUST CONTROLLER"""
        # thrust

        g = 9.81

        # Trust PD + feedforward gravity
        thrust_cmd = self.drone.m * \
            (g + self.Kp_z*local_pos_error[2] + self.Kd_z*d_local_pos_error[2])/4

        """MOTOR MIXING ALGORITHM"""

        motor_A = thrust_cmd - pitch_cmd + yaw_cmd
        motor_B = thrust_cmd - roll_cmd - yaw_cmd
        motor_C = thrust_cmd + roll_cmd - yaw_cmd
        motor_D = thrust_cmd + pitch_cmd + yaw_cmd

        motor_forces = np.array([motor_A, motor_B, motor_C, motor_D])
        motor_forces = np.maximum(0, motor_forces)

        self.command_vector = np.sqrt(motor_forces/self.drone.kf)

        self.drone.set_motor_commands(self.command_vector)
        self.new_target = False

        self.prev_local_pos_error = local_pos_error
        self.prev_orientation_error = orientation_error

        self.time += self.dt

        # This will switch to the next location when within the threshold
        # If no path is set this will set random locations.
        if sum(abs(self.target - pos)) < self.TARGET_REACHED_THRESHOLD:
            if self.path is not None:
                if self.current_path_node < len(self.path[:, 0]) - 1:
                    self.current_path_node += 1
                elif not self.path_finished:
                    print("Finished path!")
                    self.path_finished = True
            else:
                self.set_target(np.random.uniform(0, 5, (3)))

            if not self.path_finished:
                print("New target", self.target)


    def set_target(self, pos):
        """Sets a new target position for the controller to reach"""
        self.target = pos
        self.new_target = True

    def get_command_vector(self):
        return self.command_vector

    def steer(self, origin, target):
        """Takes an origin and target and returns a Nx3 matrix of coordinates of a path spline"""

        # For now we will just return a straight line with a 0.2m margin for overshoot
        return np.vstack((origin, target + 0.2*(target-origin)/np.linalg.norm(target-origin)))
    
    def follow_path(self, path, control_mode=2):
        """
        Takes a Nx3 matrix of node locations and follows the path between them.

        Will transform the path to equal distance targets between the nodes for
        more accurate path tracking if control_mode is 1.

        Will enable chase mode when control_mode is 2 (recommended).
        """

        # Add initial location
        path = np.append(self.drone.eye_of_god()[None, 0:3], path, axis=0)

        if control_mode == 1:
            self.path = transform_path(path)
        else:
            self.path = path
            if control_mode == 2:
                self.chase = True
        self.current_path_node = 1
        self.path_finished = False
    
    def chase_target(self, pos, target, previous=None):
        """
        Transforms the target position to an equaldistance position from the current location.
        If a previous target is giving it will project onto the line between the two points and
        give a constant offset from the clostest point on the line (recommended).
        """

        if previous is not None:
            curr = pos-previous
            line = target-previous

            line_norm = np.linalg.norm(line)

            scalar_projection = np.dot(curr, line) / line_norm**2

            return min((self.CHASE_DISTANCE/line_norm) + scalar_projection, 1)*line + previous
        else:
            distance = np.linalg.norm(target-pos)  
            direction = (target-pos)/distance

            return min(self.TARGET_REACHED_THRESHOLD, distance)*direction + pos

    
def transform_path(path):
    """
    Transforms a Nx3 matrix of node locations to a path of equally distanced locations
    between the node locations. Returns a Mx3 matrix.
    """

    path_out = np.empty((0, 3))
    
    step = 0.1
    for (p0, p1) in zip(path[0:-1], path[1:]):
        distance = np.linalg.norm(p1-p0)  
        direction = (p1-p0)/distance
        steps = int(np.floor(distance/step))+1

        path_out = np.append(path_out, np.linspace(0, steps*step, num=steps)[:, None]*direction + p0, axis=0)
    
    return path_out
