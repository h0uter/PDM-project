import datetime as dt
from functools import total_ordering
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from drone_class import Drone
from controller import Controller

import os

dt = 0.01
controller_data, motor_data = np.zeros(4), np.zeros(4)
reference_data, state_data = np.zeros(6), np.zeros(6)

def plot_motor_data(controller_data, motor_data):
    controller_data = np.asarray(controller_data)
    motor_data = np.asarray(motor_data)

    fig, axs = plt.subplots(4, 1)

    for i, motor_id in enumerate(['A', 'B', 'C', 'D']):
        axs[i].set_title("motor {}".format(motor_id))
        y = controller_data[:, i]
        axs[i].plot(np.arange(len(y))*dt, y, 'b-', label='motor commands')
        y = np.abs(motor_data[:, i])
        axs[i].plot(np.arange(len(y))*dt, y, 'r-', label='motor speeds')
        axs[i].set_xlabel('time (s)')
        axs[i].set_ylabel('speed')

    plt.legend()
    plt.show()

def update(frame):
    global controller_data, motor_data
    global reference_data, state_data

    controller.update()
    drone.update()
    p = drone.get_drone()
    thrust_vectors = drone.get_thrust_vectors()
    command_vector = controller.get_command_vector()
    speed_vector = drone.get_motor_speeds()

    #os.system('cls' if os.name == 'nt' else 'clear')
    #motors = ['A', 'C', 'B', 'D']
    #for motor_id, command, speed in zip(motors, command_vector, speed_vector):
    #    print("motor {} command velocity: {}".format(motor_id, ''.join(['#']*((int) (command / 10))) ))
    #    print("motor {} current velocity: {}".format(motor_id, ''.join(['+']*((int) (abs(speed) / 10))) ))
    
    # controller data for comand vs motor response
    controller_data = np.vstack((controller_data, command_vector))
    motor_data = np.vstack((motor_data, speed_vector))
    
    # references for controller tuning
    reference_data = np.vstack((reference_data, np.append(controller.target, controller.target_orientation)))
    state_data = np.vstack((state_data, drone.eye_of_god()[0:6]))


    # NOTE: there is no .set_data() for 3 dim data...
    anim[0].set_data(p[:,0], p[:,1])
    anim[0].set_3d_properties(p[:,2])

    p_frame = np.vstack((p[1:3], p[0], p[3:]))
    anim[1].set_data(p_frame[:,0], p_frame[:,1])
    anim[1].set_3d_properties(p_frame[:,2])

    for an, start, end in zip(anim[2:], p[1:], thrust_vectors):
        if (np.linalg.norm(end > 1e-5)):
            an.set_data(np.asarray([start[0], end[0]]), np.asarray([start[1], end[1]]))
            an.set_3d_properties(np.asarray([start[2], end[2]]))
        else:
            an.set_data(np.asarray([start[0], start[0]]), np.asarray([start[1], start[1]]))
            an.set_3d_properties(np.asarray([start[2], start[2]]))

    anim[-1].set_data(controller.target[0], controller.target[1])
    anim[-1].set_3d_properties(controller.target[2])


    return anim

fig = plt.figure()
ax = fig.add_subplot(projection="3d")

ax.set_zlim3d([0.0, 5.0])
ax.set_xlim3d([0.0, 5.0])
ax.set_ylim3d([0.0, 5.0])

x0,y0,z0 = 2.5,2.5,2.5
ax.set_xlabel('$X$', fontsize=20)
ax.set_ylabel('$Y$', fontsize=20)
ax.set_zlabel('$Z$', fontsize=20)
ax.view_init(azim=0, elev=90)

drone = Drone(s0=np.asarray([x0, y0, z0, 0, 0, 0]), #initial state  
              s_dot0 = np.zeros(6,),                #intitial velocities
              dt = 0.01,                            #seconds, timestep
              m = 2,                                #kg, total mass of drone
              I = 0.1,                              #kg*m^2, total drone inertia
              L = 0.2,                              #m, length of each motor arm
              kf = 0.0009,                          #kg*m, force coefficient
              km = 0.0001,                          #kg*m^2, moment coefficient
              max_motor_torque = 15,                #Nm, max load torque motors
              max_motor_omega = 350,                #rad/s, max speed motors
              propellor_inertia = 0.001             #kg*m^2, propellor inertia
              )

controller = Controller(drone)
p0 = drone.get_drone()

#points on drone
motor_locations = ax.plot(p0[:, 0], p0[:, 1], p0[:, 2], 'ro')
#frame
p_frame = np.vstack((p0[1:3], p0[0], p0[3:]))
frame = ax.plot(p_frame[:, 0], p_frame[:, 1], p_frame[:, 2], 'k-')
#thrust vectors
thrust1 = ax.plot([0,0], [0,0], [0,0], 'b-')
thrust2 = ax.plot([0,0], [0,0], [0,0], 'b-')
thrust3 = ax.plot([0,0], [0,0], [0,0], 'b-')
thrust4 = ax.plot([0,0], [0,0], [0,0], 'b-')

#Target Location
target_location = ax.plot(controller.target[0], controller.target[1], controller.target[2], 'go')

anim = motor_locations + frame + thrust1 + thrust2 + thrust3 + thrust4 + target_location
ani = animation.FuncAnimation(fig, update, interval = dt**1000, blit=False)

plt.show()

fig, axs = plt.subplots(6, 1)

for i, direction_id in enumerate(['x', 'y', 'z', "roll", "pitch", "yaw"]):

    axs[i].set_title("reference target {}".format(direction_id))
    y = reference_data[:, i]
    axs[i].plot(np.arange(len(y))*dt, y, 'b-', label='reference')
    y = state_data[:, i]
    axs[i].plot(np.arange(len(y))*dt, y, 'r-', label='position')
    axs[i].set_xlabel('time (s)')
    axs[i].set_ylabel('pos (m)')



plt.legend()
plt.show()

plot_motor_data(controller_data, motor_data)
