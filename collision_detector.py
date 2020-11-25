import numpy as np

class DroneHitbox:

    def __init__(self, s0, r=0.2):
        self.s = np.asarray(s0[:3]) # starting location of drone
        self.r = r # radius of drone hitbox

    def update(self, s):
        self.s = s[:3]

class CollisionDetector:

    def __init__(self):
        pass

    def check_collision(self, dronehitbox, spheres):
        return self.sphere_collision_detector(dronehitbox, spheres)

    def sphere_collision_detector(self, dronehitbox, spheres):
        if spheres == []:
            pass
        else:
            for sphere in spheres:
                if np.linalg.norm(dronehitbox.s - sphere.pos) < (dronehitbox.r + sphere.r): # checks for collision using euclidian distance
                    return True
                else:
                    return False

    def polygon_collision_detector(self, dronehitbox, polygons):
        if polygons == []:
            pass
