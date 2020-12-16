import random

class Controller:
    def __init__(self, drone):
        self.drone = drone
        self.command_vector = [200, 250, 120, 250]

    def update(self):
        self.command_vector = [random.randint(100, 150), random.randint(100, 150), 0, random.randint(100, 150)]
        self.drone.set_motor_commands(self.command_vector)
    
    def get_command_vector(self):
        return self.command_vector