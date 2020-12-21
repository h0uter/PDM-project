import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Graph:
    def __init__(self, start, goal):
        self.graph = {}
        self.goal_pos = goal
        self.graph['start'] = Node('start', start, self.goal_pos)
        self.graph['goal'] = Node('goal', goal, self.goal_pos)
        self.label = 0
    
    def add_node(self, pos, connected_node):
        self.label += 1
        new_node = Node(self.label, pos, self.goal_pos)
        new_node.add_edge(connected_node)
        connected_node.add_edge(new_node)

        self.graph[str(self.label)] = new_node

        return new_node
    
    def connect(self, node1, node2):
        node1.add_edge(node2)
        node2.add_edge(node1)
    
    def get_graph(self):
        return self.graph
    
    def plot_graph(self, domain, sphere_manager=None, sphere_array=None, prism_manager=None, prism_array=None, beam_manager=None, beam_array=None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        x_domain, y_domain, z_domain = domain
        ax.set_zlim3d(x_domain)
        ax.set_xlim3d(y_domain)
        ax.set_ylim3d(z_domain)

        #plot nodes
        for _, node in self.graph.items():
            ax.scatter(node.pos[0], node.pos[1], node.pos[2], s=10, label=node.label)
            for edge in node.get_edges():
                x1, y1, z1 = edge.node1.pos[0], edge.node1.pos[1], edge.node1.pos[2]
                x2, y2, z2 = edge.node2.pos[0], edge.node2.pos[1], edge.node2.pos[2]
                ax.plot([x1, x2], [y1, y2], [z1, z2])
        
        if (sphere_manager is not None) and (sphere_array is not None):
            sphere_manager.draw(ax, sphere_array)

        if (prism_manager is not None) and (prism_array is not None):
            prism_manager.draw(ax, prism_array)

        if (beam_manager is not None) and (beam_array is not None):
            beam_manager.draw(ax, beam_array)
        
        plt.show()
        
class Node:
    def __init__(self, label, pos, goal_pos, edges=[]):
        self.label = str(label)
        self.pos = pos
        self.dis_to_goal = np.linalg.norm(pos - goal_pos)
        self.edges = edges
    
    def add_edge(self, connected_node):
        new_edge = Edge(self, connected_node)
        self.edges = np.append(self.edges, new_edge)
    
    def get_edges(self):
        return self.edges

class Edge:
    def __init__(self, node1, node2):
        self.node1 = node1
        self.node2 = node2
        self.cost = self.calc_distance()

    def calc_distance(self):
        p1 = self.node1.pos
        p2 = self.node2.pos

        return np.linalg.norm(p1 - p2)
