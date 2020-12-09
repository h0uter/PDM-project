import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import config as cfg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from drone_class import Drone
from controller import Controller
from collision_detector import DroneHitbox, CollisionDetector
from obstacles import SphereManager, PolygonManager

import os

dt = 0.01
controller_data, motor_data = np.zeros(4), np.zeros(4)

def update(frame):
    global controller_data, motor_data

    controller.update()
    drone.update()
    drone_hitbox.update(drone.s)
    print(collision_detector.check_collision(drone_hitbox, sphere_array, polygon_array, edges))
    if collision_detector.sphere_collision_point:
        print(collision_detector.sphere_collision_point)
        x = collision_detector.sphere_collision_point[0]
        y = collision_detector.sphere_collision_point[1]
        z = collision_detector.sphere_collision_point[2]
        ax.plot(x, y, z, 'bo')
        collision_detector.sphere_collision_point = []

    elif collision_detector.polygon_collision_point:
        x = collision_detector.polygon_collision_point[0]
        y = collision_detector.polygon_collision_point[1]
        z = collision_detector.polygon_collision_point[2]
        ax.plot(x, y, z, 'bo')
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

ax.view_init(azim=45, elev=0)
ax.set_xlabel('$X$', fontsize=20)
ax.set_ylabel('$Y$', fontsize=20)
ax.set_zlabel('$Z$', fontsize=20)

x0,y0,z0 = 2.5,2.5,2.5
drone = Drone([x0, y0, z0, 0, 0, 0], [0, 0, 0, 0, 0, 0], dt, l=[0.2,0.2,0.2,0.2])
controller = Controller(drone)
drone_hitbox = DroneHitbox(drone.s[:3])
sphere_manager = SphereManager(cfg.n_spheres, cfg.spheres_pos, cfg.spheres_r)
polygon_manager = PolygonManager(cfg.n_polygons, cfg.polygons, cfg.polygons_pos)
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
polygon_array = polygon_manager.create_polygons()

#initialise collision detector
collision_detector = CollisionDetector()
edges = polygon_manager.get_edges(polygon_array)


#plot spheres
for sphere in sphere_array:
    x, y, z = sphere.create_sphere()
    ax.plot_wireframe(x, y, z, color="r")

#plot polygons
for polygon in polygon_array:
    edges = polygon.create_polygon()
    ax.add_collection3d(Poly3DCollection(edges))

anim = motor_locations + frame + thrust1 + thrust2 + thrust3 + thrust4

ani = animation.FuncAnimation(fig, update, interval = dt**1000, blit=False)

plt.show()
controller_data = np.asarray(controller_data)
motor_data = np.asarray(motor_data)

fig, axs = plt.subplots(4, 1)

for i, motor_id in enumerate(['A', 'C', 'B', 'D']):
    axs[i].set_title("motor {}".format(motor_id))
    y = controller_data[:, i]
    axs[i].plot(np.arange(len(y))*dt, y, 'b-', label='motor commands')
    y = np.abs(motor_data[:, i])
    axs[i].plot(np.arange(len(y))*dt, y, 'r-', label='motor speeds')
    axs[i].set_xlabel('time (s)')
    axs[i].set_ylabel('speed')

plt.legend()
plt.show()
