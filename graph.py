import numpy as np
import matplotlib.pyplot as plt

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

        self.graph[self.label] = new_node
    
    def get_graph(self):
        return self.graph

class Node:
    def __init__(self, label, pos, goal_pos, edges=[]):
        self.label = label
        self.pos = pos
        self.dis_to_goal = np.linalg.norm(pos - goal_pos)
        self.edges = edges
    
    def add_edge(self, connected_node):
        new_edge = Edge(connected_node, self)
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

start = np.asarray([2,2])
goal = np.asarray([5,3])
g = Graph(start, goal)

g.add_node(np.asarray([4,4]), g.get_graph()['start'])
g.add_node(np.asarray([3,-2]), g.get_graph()[1])
#g.add_node(np.asarray([8,-2]), 'goal')

def visualize_graph():
    graph = g.get_graph()
    for label, node in graph.items():
        print(label, node.dis_to_goal)
        plt.scatter(node.pos[0], node.pos[1], label=label)
        for edge in node.get_edges():
            x1, y1 = edge.node1.pos[0], edge.node1.pos[1]
            x2, y2 = edge.node2.pos[0], edge.node2.pos[1]
            plt.plot([x1, x2], [y1, y2])
            print(label, edge.node1.label, edge.node2.label, edge.cost)

    plt.legend()
    plt.show()

visualize_graph()