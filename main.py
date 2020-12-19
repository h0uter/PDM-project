import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import config as cfg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from drone_class import Drone
from controller import Controller
from collision_detector import DroneHitbox, CollisionDetector
from obstacles import SphereManager, BeamManager, PrismManager

import os

dt = 0.001
controller_data, motor_data = np.zeros(4), np.zeros(4)

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

    drone.set_motor_commands([150, 150, 150, 150])
    drone.update()
    drone_hitbox.update(drone.s)
    test_collision_line = [np.array([0,0,0]),np.array([2,2,2])] # collision line for debugging purposes
    print(collision_detector.check_collision(drone_hitbox, sphere_array, prism_array, beam_array, test_collision_line))
    if collision_detector.sphere_collision_point:
        x = collision_detector.sphere_collision_point[0]
        y = collision_detector.sphere_collision_point[1]
        z = collision_detector.sphere_collision_point[2]
        ax.plot(x, y, z, 'bo')
        print(collision_detector.sphere_collision_point)
        a
        collision_detector.sphere_collision_point = []

    elif collision_detector.polygon_collision_point:
        x = collision_detector.polygon_collision_point[0]
        y = collision_detector.polygon_collision_point[1]
        z = collision_detector.polygon_collision_point[2]
        ax.plot(x, y, z, 'bo')
        print(collision_detector.polygon_collision_point)
        a
        collision_detector.polygon_collision_point = []


    p = drone.get_drone()
    thrust_vectors = drone.get_thrust_vectors()
    command_vector = controller.get_command_vector()
    speed_vector = drone.get_motor_speeds()


    os.system('cls' if os.name == 'nt' else 'clear')
    motors = ['A', 'C', 'B', 'D']
    """
    for motor_id, command, speed in zip(motors, command_vector, speed_vector):
        print("motor {} command velocity: {}".format(motor_id, ''.join(['#']*((int) (command / 10))) ))
        print("motor {} current velocity: {}".format(motor_id, ''.join(['+']*((int) (abs(speed) / 10))) ))
    """

    controller_data = np.vstack((controller_data, command_vector))
    motor_data = np.vstack((motor_data, speed_vector))

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

    return anim

fig = plt.figure()
ax = fig.add_subplot(projection="3d")

ax.set_zlim3d([0.0, 5.0])
ax.set_xlim3d([5.0, 0.0])
ax.set_ylim3d([0.0, 5.0])

ax.view_init(azim=90, elev=0)
ax.set_xlabel('$X$', fontsize=20)
ax.set_ylabel('$Y$', fontsize=20)
ax.set_zlabel('$Z$', fontsize=20)

x0,y0,z0 = 2.5,2.5,2.5

drone = Drone(s0=np.asarray([x0, y0, z0, 0, 0, 0]), #initial state
              s_dot0 = np.zeros(6,),                #intitial velocities
              dt = 0.005,                            #seconds, timestep
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
drone_hitbox = DroneHitbox(drone.s[:3], cfg.dronehitbox_r)
sphere_manager = SphereManager(cfg.n_spheres, cfg.spheres_pos, cfg.spheres_r, cfg.dronehitbox_r, cfg.safety_margin)
prism_manager = PrismManager(cfg.n_prisms, cfg.prisms, cfg.prisms_pos, cfg.dronehitbox_r, cfg.safety_margin)
beam_manager = BeamManager(cfg.n_beams, cfg.beams, cfg.beams_pos, cfg.dronehitbox_r, cfg.safety_margin)
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

#create obstacle objects
sphere_array = sphere_manager.create_spheres()
prism_array = prism_manager.create_prisms()
beam_array = beam_manager.create_beams()

#initialise collision detector
collision_detector = CollisionDetector(cfg.safety_margin)
#edges = polygon_manager.get_edges(polygon_array)

#plot spheres
for sphere in sphere_array:
    x, y, z = sphere.create_sphere()
    ax.plot_wireframe(x, y, z, color="r")

#plot prisms
for prism in prism_array:
    for polygon in prism.polygons_array:
        edges = polygon.create_polygon()
        drawing = Poly3DCollection(edges)
        ax.add_collection3d(drawing)
        drawing.set_alpha(0.3)

#plot beams
for beam in beam_array:
    for polygon in beam.polygons_array:
        edges = polygon.create_polygon()
        drawing = Poly3DCollection(edges)
        ax.add_collection3d(drawing)
        drawing.set_alpha(0.3)

anim = motor_locations + frame + thrust1 + thrust2 + thrust3 + thrust4
ani = animation.FuncAnimation(fig, update, interval = dt**1000, blit=False)

plt.show()

plot_motor_data(controller_data, motor_data)
