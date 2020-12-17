import random

class Controller:
    def __init__(self, drone):
        self.drone = drone
        self.command_vector = [200, 200, 200, 200]

    def update(self):
        #self.command_vector = [random.randint(100, 500), random.randint(100, 500), random.randint(100, 500), random.randint(100, 500)]
        self.drone.set_motor_commands(self.command_vector)

    def get_command_vector(self):
        return self.command_vector
