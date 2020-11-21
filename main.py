import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from drone_class import Drone
from controller import Controller

dt = 0.01

def update(frame):
    controller.update()
    drone.update()
    p = drone.get_drone()

    # NOTE: there is no .set_data() for 3 dim data...
    anim[0].set_data(p[:,0], p[:,1])
    anim[0].set_3d_properties(p[:,2])

    p_frame = np.vstack((p[1:3], p[0], p[3:]))
    anim[1].set_data(p_frame[:,0], p_frame[:,1])
    anim[1].set_3d_properties(p_frame[:,2])

    return anim

fig = plt.figure()
ax = fig.add_subplot(projection="3d")

ax.set_zlim3d([0.0, 10.0])
ax.set_xlim3d([0.0, 10.0])
ax.set_ylim3d([0.0, 10.0])

x0,y0,z0 = 5,5,5
drone = Drone([x0, y0, z0, 0, 0, 0], [0, 0, 0, 0, 0, 0], dt, l=[0.2,0.2,0.2,0.2])
controller = Controller(drone)
p0 = drone.get_drone()

#points on drone
motor_locations = ax.plot(p0[:, 0], p0[:, 1], p0[:, 2], 'ro')
#frame
p_frame = np.vstack((p0[1:3], p0[0], p0[3:]))
frame = ax.plot(p_frame[:, 0], p_frame[:, 1], p_frame[:, 2], 'k-')

anim = motor_locations + frame
print(anim)

ani = animation.FuncAnimation(fig, update, interval = dt**1000, blit=False)
plt.show()