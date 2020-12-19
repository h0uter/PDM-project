import numpy as np
from graph import Graph
from A_star import A_star

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RRT:

    def __init__(self, start, goal, search_range, domain, max_iters=10000):
        self.graph = Graph(start, goal)
        self.x_domain, self.y_domain, self.z_domain = domain
        self.search_range = search_range
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
        #TODO, check position 2 on collisions and vector pos2 - pos1, possible with steering function
        return False
    
    def check_line_of_sight(self, node):
        #TODO method checks whether or not checked node has line of sight to target, if so connect these immeadiatly 
        # there are no obstacles now so this would always connect these
        pass

    def compute_paths(self):
        for _ in range(self.max_iters):
            closest_node, new_node_pos = self.get_new_node()
            if (not self.check_collision(closest_node.pos, new_node_pos)):
                self.graph.add_node(new_node_pos, closest_node)
            else:
                continue
    
    def get_graph(self):
        return self.graph

start = np.asarray([3,3,4])
goal = np.asarray([7,3,7])
search_range = 0.5
domain = ((0, 10), (0, 10), (0, 10))
rrt = RRT(start, goal, search_range, domain, max_iters=1000)
rrt.compute_paths()
rrt.get_graph().plot_graph(domain)

graph = rrt.get_graph()
a_star_planner = A_star(graph)
path, cost, success = a_star_planner.find_path(graph.get_graph()['start'], graph.get_graph()['542'])
print(path, cost)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x_domain, y_domain, z_domain = domain
ax.set_zlim3d(x_domain)
ax.set_xlim3d(y_domain)
ax.set_ylim3d(z_domain)

for n, node in enumerate(path):
    ax.scatter(node.pos[0], node.pos[1], node.pos[2], s=10, label=node.label)
    if n < len(path) - 1:
        node1 = node
        node2 = path[n+1]
        x1, y1, z1 = node1.pos[0], node1.pos[1], node1.pos[2]
        x2, y2, z2 = node2.pos[0], node2.pos[1], node2.pos[2]
        ax.plot([x1, x2], [y1, y2], [z1, z2])

plt.show()

