import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from drone_class import Drone
from controller import Controller

dt = 0.01

def update(frame):
    controller.update()
    #drone.set_motor_commands([120, 250, 120, 250])
    drone.update()
    p = drone.get_drone()

    # NOTE: there is no .set_data() for 3 dim data...
    anim[0].set_data(p[:,0], p[:,1])
    anim[0].set_3d_properties(p[:,2])

    p_frame = np.vstack((p[1:3], p[0], p[3:]))
    anim[1].set_data(p_frame[:,0], p_frame[:,1])
    anim[1].set_3d_properties(p_frame[:,2])

    thrust_vectors = drone.get_thrust_vectors()
    for an, start, end in zip(anim[2:], p[1:], thrust_vectors):
        if (np.linalg.norm(end > 1e-5)):
            an.set_data([start[0], end[0]], [start[1], end[1]])
            an.set_3d_properties([start[2], end[2]])
        else:
            an.set_data([start[0], start[0]], [start[1], start[1]])
            an.set_3d_properties([start[2], start[2]])

    return anim

fig = plt.figure()
ax = fig.add_subplot(projection="3d")

ax.set_zlim3d([0.0, 5.0])
ax.set_xlim3d([0.0, 5.0])
ax.set_ylim3d([0.0, 5.0])

x0,y0,z0 = 2.5,2.5,2.5
drone = Drone([x0, y0, z0, 0, 0, 0], [0, 0, 0, 0, 0, 0], dt, l=[0.2,0.2,0.2,0.2])
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

anim = motor_locations + frame + thrust1 + thrust2 + thrust3 + thrust4

ani = animation.FuncAnimation(fig, update, interval = dt**1000, blit=False)
plt.show()