import numpy as np

class DroneHitbox:

    def __init__(self, s0, r=0.2):
        self.s = np.asarray(s0[:3]) # starting location of drone
        self.r = r # radius of drone hitbox

    def update(self, s):
        self.s = s

class CollisionDetector:

    def __init__(self, DroneHitbox, sphere_array):
        self.s = DroneHitbox.s
        self.r = DroneHitbox.r
        self.spheres = sphere_array

    def check_collision(self):
        return sphere_collision_detector(self.spheres)

    def sphere_collision_detector(self, spheres):
        if spheres == []:
            pass
        else:
            for sphere in spheres:
                if np.linalg.norm(self.s - sphere.s) < (self.r + sphere.r): # checks for collision using euclidian distance
                    return True
                else:
                    return False
