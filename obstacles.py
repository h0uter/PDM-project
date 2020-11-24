import numpy as np

class Sphere:

    def __init__(self, r, pos):
        self.r = r # radius of sphere
        self.pos = pos # position of sphere (x, y, z)

    def create_sphere(self):
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x = self.r * np.cos(u) * np.sin(v) + self.pos[0]
        y = self.r * np.sin(u) * np.sin(v) + self.pos[1]
        z = self.r * np.cos(v) + self.pos[2]
        ax.plot_wireframe(x, y, z, color="r")

class SphereManager:

    def __init__(self, n_spheres, pos_array, r_array):
        self.n_spheres = n_spheres
        self.pos_array = pos_array
        self.r_array   = r_array

    def create_spheres(self):
        sphere_array = []

        for i in n_spheres:
            sphere = Sphere(r_array[i], pos_array[i])
            sphere_array.append(sphere.create_sphere)

        return sphere_array
