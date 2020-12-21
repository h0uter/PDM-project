import datetime as dt
from functools import total_ordering
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import config as cfg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from drone_class import Drone
from controller import Controller
from RRT import RRT
from A_star import A_star
from collision_detector import CollisionDetector
from obstacles import SphereManager, BeamManager, PrismManager

import os

dt = 0.01
xs, ys, zs = [0.0, 5.0], [0.0, 5.0], [0.0, 5.0]
x0,y0,z0 = 2.5,2.5,2.5
x_target, y_target, z_target = 4.1, 3.2, 5.3

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

sphere_manager = SphereManager(cfg.n_spheres, cfg.spheres_pos, cfg.spheres_r, cfg.dronehitbox_r, cfg.safety_margin)
prism_manager = PrismManager(cfg.n_prisms, cfg.prisms, cfg.prisms_pos, cfg.dronehitbox_r, cfg.safety_margin)
beam_manager = BeamManager(cfg.n_beams, cfg.beams, cfg.beams_pos, cfg.dronehitbox_r, cfg.safety_margin)

controller = Controller(drone)
rrt = RRT(np.asarray([x0, y0, z0]), np.asarray([x_target, y_target, z_target]), search_range=0.5, domain=(xs, ys, zs), max_iters=500)
rrt.compute_paths()

graph = rrt.get_graph()
graph.plot_graph((xs, ys, zs))

a_star_planner = A_star(graph)
path, cost = a_star_planner.find_path(graph.get_graph()['start'], graph.get_graph()['440'])
path_pos = np.zeros((len(path), 3))

for i, node in enumerate(path): path_pos[i] = node.pos
controller.follow_path(path_pos[1:])

def update(frame):
    controller.update()
    drone.update()

    p = drone.get_drone()
    thrust_vectors = drone.get_thrust_vectors()

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

    # plot the current setpoint
    anim[6].set_data(controller.internal_target[0], controller.internal_target[1])
    anim[6].set_3d_properties(controller.internal_target[2])

    # plot the entire path
    anim[7].set_data(controller.path[:, 0], controller.path[:, 1])
    anim[7].set_3d_properties(controller.path[:, 2])

    return anim

fig = plt.figure()
ax = fig.add_subplot(projection="3d")

ax.set_zlim3d(xs)
ax.set_xlim3d(ys)
ax.set_ylim3d(zs)

ax.set_xlabel('$X$', fontsize=20)
ax.set_ylabel('$Y$', fontsize=20)
ax.set_zlabel('$Z$', fontsize=20)
ax.view_init(azim=0, elev=90)

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
target_location = ax.plot([controller.internal_target[0]], [controller.internal_target[1]], [controller.internal_target[2]], 'go')
# path plot
path_plot = ax.plot(controller.path[0,:], controller.path[1, :], controller.path[2, :], 'm:')

#create obstacle objects
sphere_array = sphere_manager.create_spheres()
prism_array = prism_manager.create_prisms()
beam_array = beam_manager.create_beams()

#draw obstacles
sphere_manager.draw(ax, sphere_array)
prism_manager.draw(ax, prism_array)
beam_manager.draw(ax, beam_array)

#initialise collision detector
collision_detector = CollisionDetector(cfg.safety_margin, sphere_array, prism_array, beam_array, cfg.dronehitbox_r)

# index anim as an array
anim = motor_locations + frame + thrust1 + thrust2 + thrust3 + thrust4 + target_location + path_plot
ani = animation.FuncAnimation(fig, update, interval = dt**1000, blit=False)

plt.show()

