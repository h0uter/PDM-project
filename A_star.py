import numpy as np

class A_star:

    def __init__(self, graph_object):
        self.graph = graph_object
        self.labels = graph_object.get_graph().keys()
        self.explored_nodes = {}
        self.search_dictionary = {}
    
    def find_connected_edges(self, start_node):
        return start_node.get_edges()

    def calc_cost(self, edge, goal_node, target_node):
        dis_to_target = np.linalg.norm(target_node.pos - goal_node.pos)
        edge_cost = edge.cost
        return edge_cost, edge_cost + dis_to_target

    def sort_dictionary(self, unsorted_dict):
        lowest_value = 1e6
        lowest_edge_cost = 0
        top_node_label = None
        has_content = bool(unsorted_dict)

        if has_content:
            for k,v in unsorted_dict.items():
                if v[1] < lowest_value:
                    lowest_value = v[1]
                    lowest_edge_cost = v[2]
                    top_node_label = k
        
        return top_node_label, lowest_edge_cost, has_content

    def step(self, node_to_explore, goal_node):
        for edge in node_to_explore.get_edges():
            if (edge.node2.label in list(self.explored_nodes.keys())):
                continue
            else:
                connected_node = edge.node2
                edge_cost, cost = self.calc_cost(edge, connected_node, goal_node)
                self.search_dictionary[connected_node.label] = (node_to_explore, cost, edge_cost)

        top_node_label, edge_cost, not_done = self.sort_dictionary(self.search_dictionary)
        top_node = None

        if (not_done):
            top_node = self.graph.get_graph()[top_node_label]

            self.explored_nodes[top_node_label] = (node_to_explore, edge_cost)
            self.search_dictionary.pop(top_node_label)
        
            if top_node == goal_node:
                top_node = None
            
        return top_node

    def find_previous_step(self, start_node, last_node):
        current_node = self.explored_nodes[last_node.label]

        self.path = np.append(self.path, self.graph.get_graph()[last_node.label])
        self.total_cost += current_node[1]
        
        if current_node[0] != start_node:
            self.find_previous_step(start_node, current_node[0])
        else:
            self.path = np.append(self.path, start_node)

    def backward_search(self, start_node, goal_node):
        if goal_node.label in list(self.explored_nodes.keys()):
            self.total_cost = 0
            self.path = []

            self.find_previous_step(start_node, goal_node)
            return self.path, self.total_cost, True
        else:
            return None, 0, False

    def find_path(self, start_node, goal_node):
        self.explored_nodes[start_node.label] = (start_node, 0)
        current_node = start_node

        while current_node is not None:
            current_node = self.step(current_node, goal_node)
        
        return self.backward_search(start_node, goal_node)
