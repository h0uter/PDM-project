import numpy as np

class DroneHitbox:

    def __init__(self, s0, s_dot0, dt, r=0.2):
        self.s = np.asarray(s0[0:3])
        self.r = r

    def update(self, s, s_dot, dt):
        self.s = np.add(s, s_dot * dt)

class CollisionDetector:

    def __init__(self, DroneHitbox, SphereArray):
        self.s = DroneHitbox.s
        self.r = DroneHitbox.r
        self.spheres = SphereArray

    def check_collision(self):
        if sphere_collision_detector(self.spheres):
            return True

    def sphere_collision_detector(self, spheres):
        if spheres == []:
            pass
        else:
            for sphere in spheres:
                if np.linalg.norm(self.s - sphere.s) < (self.r + sphere.r):
                    return True
                else:
                    return False
