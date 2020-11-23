import datetime as dt
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

# # wouter -------------------------
# # doel is om live control waardes te plotten
# # ax_debug = fig.add_subplot(projection="2d")

# # t = np.linspace(0, 2*np.pi)
# # x = np.sin(t)

# fig2, ax2 = plt.subplots()
# # l, = ax2.plot([0, 2*np.pi], [-1, 1])
# # animate = lambda i: l.set_data(t[:i], x[:i])

# x = 0
# line, = ax2.plot(x, np.sin(x))
# tijd = 0

# xs = []
# ys = []

# def update2(i, xs, ys):

#     tijd= 2

#     # Add x and y to lists
#     xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
#     ys.append(tijd)

#     # Limit x and y lists to 20 items
#     xs = xs[-20:]
#     ys = ys[-20:]

#     # Draw x and y lists
#     ax2.clear()
#     ax2.plot(xs, ys)

#     # Format plot
#     plt.xticks(rotation=45, ha='right')
#     plt.subplots_adjust(bottom=0.30)
#     plt.title('TMP102 Temperature over Time')
#     plt.ylabel('Temperature (deg C)')
#     # return line,


# ani2 = animation.FuncAnimation(fig2, update2, fargs=(xs, ys), interval=dt**1000, blit=False)
# plt.show()
# # wouter --------------------------------

ax.set_zlim3d([0.0, 20.0])
ax.set_xlim3d([0.0, 20.0])
ax.set_ylim3d([0.0, 20.0])
ax.set_xlabel('$X$', fontsize=20)
ax.set_ylabel('$Y$', fontsize=20)
ax.set_zlabel('$Z$', fontsize=20)

ax.view_init(azim=0, elev=90)


x0,y0,z0 = 5,5,5
# drone = Drone([x0, y0, z0, 0, 0, 0], [0, 0, 0, 0, 0, 0], dt, l=[0.2,0.2,0.2,0.2])
drone = Drone([x0, y0, z0, 0, 0, 0], [0, 0, 0, 0, 0, 0], dt, l=[1,1,1,1])
controller = Controller(drone)
p0 = drone.get_drone()

#points on drone
motor_locations = ax.plot(p0[:, 0], p0[:, 1], p0[:, 2], 'ro')
#frame
p_frame = np.vstack((p0[1:3], p0[0], p0[3:]))
frame = ax.plot(p_frame[:, 0], p_frame[:, 1], p_frame[:, 2], 'k-')

anim = motor_locations + frame

ani = animation.FuncAnimation(fig, update, interval = dt**1000, blit=False)

#TODO: plot error values
plt.show()
