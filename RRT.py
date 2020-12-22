import numpy as np
from graph import Graph
from A_star import A_star
from controller import Controller
from ellipse import Ellipse

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import config as cfg

class RRT:

    def __init__(self, start, goal, search_range, domain, collision_manager, controller, informed, kinodynamic, initial_state, max_iters=1000):
        self.start = start
        self.goal = goal
        self.graph = Graph(start, goal, state0=initial_state)
        self.x_domain, self.y_domain, self.z_domain = domain
        self.search_range = search_range
        self.collision_manager = collision_manager
        self.controller = controller
        self.informed = informed
        self.kinodynamic = kinodynamic
        self.max_iters = max_iters

        self.ellipse = Ellipse(start, goal)

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
        if self.ellipse.ellipse_exists() and self.informed:
            random_point = self.ellipse.get_random_point()
        else:
            random_point = np.random.rand(3) * np.asarray([self.x_domain[1] - self.x_domain[0] - cfg.dronehitbox_r,
                                                        self.y_domain[1] - self.y_domain[0] - cfg.dronehitbox_r,
                                                        self.z_domain[1] - self.z_domain[0] - cfg.dronehitbox_r]) \
            + np.asarray([self.x_domain[0] + cfg.dronehitbox_r, self.y_domain[0] + cfg.dronehitbox_r, self.z_domain[0] + cfg.dronehitbox_r])

        closest_node = self.get_closest_point(random_point)
        dir_vector = random_point - closest_node.pos
        dir_vector_length = np.linalg.norm(dir_vector)

        if dir_vector_length > self.search_range:
            new_pos = (dir_vector / dir_vector_length) * self.search_range + closest_node.pos
        else:
            new_pos = random_point

        #node to connect to, new proposed node position
        return closest_node, new_pos

    def check_collision(self, node0, pos2):
        if self.kinodynamic:
            path_points, new_state = self.controller.steer(node0.pos, pos2, full_state=node0.state)
            return self.collision_manager.update(path_points), new_state
        else:
            return self.collision_manager.update([node0.pos, pos2]), None
    
    def check_line_of_sight(self, node):
        collides, _ = self.check_collision(node, self.goal)
        if not collides:
            self.graph.connect(node, self.graph.get_graph()['goal'])
            if self.informed:
                a_star_planner = A_star(self.graph)
                path, _ = a_star_planner.find_path(self.graph.get_graph()['start'], self.graph.get_graph()['goal'])
                self.ellipse.define_ellipse(path)

    def compute_paths(self):
        self.check_line_of_sight(self.graph.get_graph()['start'])

        for n in range(self.max_iters):
            print(f"building graph {n}/{self.max_iters}")
            closest_node, new_node_pos = self.get_new_node()
            collides, new_state = self.check_collision(closest_node, new_node_pos)
        
            if not collides:
                new_node = self.graph.add_node(new_node_pos, closest_node, state=new_state)
                self.check_line_of_sight(new_node)

    def get_graph(self):
        return self.graph
    