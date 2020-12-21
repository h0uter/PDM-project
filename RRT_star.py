import numpy as np
from graph import Graph
from A_star import A_star
from controller import Controller

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy

class RRT_star:

    def __init__(self, start, goal, search_range, domain, collision_manager, controller, max_iters=10000):
        self.start = start
        self.goal = goal
        self.graph = Graph(start, goal)
        self.x_domain, self.y_domain, self.z_domain = domain
        self.search_range = search_range
        self.collision_manager = collision_manager
        self.controller = controller
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
        return new_pos

    def check_collision(self, pos1, pos2):
        return self.collision_manager.update([pos1, pos2])

    def check_line_of_sight(self, node):
        if not self.check_collision(node.pos, self.goal):
            self.graph.connect(node, self.graph.get_graph()['goal'])

    def step(self, origin_pos, search_factor=2):
        a_star_planner = A_star(self.graph)
        node_values = []
        optimal_cost, optimal_node = 1e6, None

        #find optimal connection point with lowest cost from start node
        for node in self.graph.get_graph().values():
            if node != self.graph.get_graph()['goal']:
                dis_to_new_node = np.linalg.norm(node.pos - origin_pos)
                if dis_to_new_node <= self.search_range * search_factor and not self.check_collision(node.pos, origin_pos):
                    if node != self.graph.get_graph()['start']:
                        path, cost = a_star_planner.find_path(self.graph.get_graph()['start'], node)
                    else:
                        path, cost = [], 0

                    node_values.append((node, path, cost))
                    cost += dis_to_new_node 

                    if cost < optimal_cost:
                        optimal_cost = cost
                        optimal_node = node
        
        if len(node_values) > 0:
            new_node = self.graph.add_node(origin_pos, optimal_node)
            self.check_line_of_sight(new_node)

            #rewire phase, check for each node whether a more optimal path via the new node exists
            for node, path, cost in node_values:
                if optimal_cost + np.linalg.norm(node.pos - origin_pos) < cost:
                    self.graph.rewire(path[-2], node, new_node)
        
    def compute_paths(self):
        self.check_line_of_sight(self.graph.get_graph()['start'])

        for _ in range(self.max_iters):
            new_node_pos = self.get_new_node()
            self.step(new_node_pos)

    def get_graph(self):
        return self.graph
    