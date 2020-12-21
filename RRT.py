import numpy as np
from graph import Graph
from A_star import A_star
from controller import Controller

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy

class RRT:

    def __init__(self, start, goal, search_range, domain, collision_manager, controller, drone, max_iters=10000):
        self.graph = Graph(start, goal)
        self.x_domain, self.y_domain, self.z_domain = domain
        self.search_range = search_range
        self.collision_manager = collision_manager
        self.controller = controller
        self.drone = drone
        self.max_iters = max_iters

        np.random.seed(69)

    def get_closest_point(self, point):
        shortest_distance = 1e6
        closest_node = None

        for label, node in self.graph.get_graph().items():
            if (label == 'goal'):
                continue
            else:
                dis = np.linalg.norm(point - node.pos)
                if dis < shortest_distance:
                    shortest_distance = dis
                    closest_node = node
                
        return closest_node

    def get_new_node(self):
        random_point = np.random.rand(3) * np.asarray([self.x_domain[1] - self.x_domain[0],
                                                       self.y_domain[1] - self.y_domain[0],
                                                       self.z_domain[1] - self.z_domain[0]]) \
                        + np.asarray([self.x_domain[0], self.y_domain[0], self.z_domain[0]]) 

        closest_node = self.get_closest_point(random_point)
        dir_vector = random_point - closest_node.pos
        dir_vector_length = np.linalg.norm(dir_vector)

        if dir_vector_length > self.search_range:
            new_pos = (dir_vector / dir_vector_length) * self.search_range + closest_node.pos
        else:
            new_pos = random_point

        #node to connect to, new proposed node position
        return closest_node, new_pos

    def check_collision(self, pos1, pos2):
        return self.collision_manager.update([pos1, pos2])
    
    def check_line_of_sight(self, node):
        #TODO method checks whether or not checked node has line of sight to target, if so connect these immeadiatly 
        # there are no obstacles now so this would always connect these
        pass

    def check_spline(self, new_node_pos, connected_node):
        g = copy.copy(self.graph)
        target_node = g.add_node(new_node_pos, connected_node)

        #define new planner based on updated graph and extract optimal path to new node
        a_star = A_star(g)
        path, _ = a_star.find_path(self.graph.get_graph()['start'], target_node)

        #convert this path to nx3 array for controller
        path_pos = np.zeros((len(path), 3))
        for i, node in enumerate(path): path_pos[i] = node.pos

        #create new instances of drone and controller and pass arguments
        drone = copy.copy(self.drone)
        controller = Controller(drone)
        controller.follow_path(path_pos[1:])

        pos_data = drone.eye_of_god()[:3]

        #run full simulation and gather pos data at each timestep
        while not controller.path_finished:
            controller.update()
            drone.update()
            pos_data = np.vstack((pos_data, drone.eye_of_god()[:3]))
        
        #check wheter at any point the drone collides
        return self.collision_manager.update(pos_data)

    def compute_paths(self):
        for _ in range(self.max_iters):
            closest_node, new_node_pos = self.get_new_node()
            if not self.check_collision(closest_node.pos, new_node_pos):
                _ = self.graph.add_node(new_node_pos, closest_node)
    
    def get_graph(self):
        return self.graph
    