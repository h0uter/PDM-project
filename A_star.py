import numpy as np

class A_star:

    def __init__(self, graph_object):
        self.graph = graph_object

    def calc_cost(self, edge):
        current_node = edge.node1
        target_node = edge.node2
        edge_cost = edge.cost

        heuristic = np.linalg.norm(self.goal_node.pos - target_node.pos) 
        path_cost = self.explored_nodes[current_node.label]["path_cost"] + edge_cost
        total_cost = path_cost + heuristic

        return (path_cost, total_cost)
    
    def sort_dic(self):
        lowest_total_cost = 1e6
        top_node_label = None

        for connected_node_label, properties in self.priority_queue.items():
            if properties["total_cost"] < lowest_total_cost:
                lowest_total_cost = properties["total_cost"]
                top_node_label = connected_node_label
        
        return top_node_label
    
    def step(self, node_to_explore_data):
        node_to_explore = node_to_explore_data["object"]
        self.explored_nodes[node_to_explore.label] = node_to_explore_data

        for edge in node_to_explore.get_edges():
            if edge.node2.label in self.explored_nodes:
                continue
            else:
                connected_node = edge.node2
                path_cost, total_cost = self.calc_cost(edge)
                update = True

                if connected_node.label in self.priority_queue:
                    if self.priority_queue[connected_node][path_cost] <= path_cost:
                        update = False

                if update:
                    self.priority_queue[connected_node.label] = {"object": connected_node,
                                                                 "previous_node": node_to_explore, 
                                                                 "path_cost": path_cost, 
                                                                 "total_cost": total_cost}

        next_node_to_explore_label = self.sort_dic()
        node_data = self.priority_queue.pop(next_node_to_explore_label)

        done = (node_data["object"] == self.goal_node)
        if done: self.explored_nodes[self.goal_node.label] = node_data

        return node_data, done

    def backtrack(self):
        path = []
        current_label = self.goal_node.label
        searching = True 

        while searching:
            last_visited_node_data = self.explored_nodes[current_label]
            current_node = last_visited_node_data["previous_node"]

            if current_node is not None:
                current_label = current_node.label
            else:
                searching = False

            path.append(last_visited_node_data["object"])
        
        return np.flip(path), self.explored_nodes[self.goal_node.label]["path_cost"]

    def find_path(self, start_node, goal_node):
        self.start_node = start_node
        self.goal_node = goal_node
        self.explored_nodes = {}
        self.priority_queue = {}

        if start_node.label not in self.graph.get_graph() or goal_node.label not in self.graph.get_graph():
            raise KeyError(f"either target {start_node.label} or goal {goal_node.label} not in provided graph")

        done = False
        current_node = {"object": start_node,
                        "previous_node": None, 
                        "path_cost": 0., 
                        "total_cost": np.linalg.norm(goal_node.pos - start_node.pos)}

        while not done:
            current_node, done = self.step(current_node)

        return self.backtrack()
        
