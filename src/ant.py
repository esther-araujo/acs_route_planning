#!/usr/bin/env python3

class Ant:
    def __init__(self, start_node, target_node, num_nodes, found_goal):
        self.start_node = start_node
        self.target_node = target_node
        self.visited_nodes = [start_node]
        self.num_nodes = num_nodes
        self.total_distance = 0.0
        self.found_goal = found_goal
        self.steps = 0
    
    def has_visited_all_nodes(self):
        return len(self.visited_nodes) == self.num_nodes
    
    def visit(self, node, edge):
        # Add the visited node to the list and update the total distance
        self.visited_nodes.append(node)
        if len(self.visited_nodes) > 1 and edge:
            self.total_distance += edge['weight']
    
    def get_current_node(self):
        return self.visited_nodes[-1]
    
    def get_visited_nodes(self):
        return self.visited_nodes
    
    def get_total_distance(self):
        return self.total_distance
        
    def reset(self, start_node):
        self.visited_nodes = [start_node]
        self.found_goal = False
        self.total_distance = 0.0