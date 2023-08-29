#!/usr/bin/env python3

class Ant:
    def __init__(self, start_node, target_node, num_nodes, distances):
        self.start_node = start_node
        self.target_node = target_node
        self.visited_nodes = [start_node]
        self.num_nodes = num_nodes
        self.distances = distances  # Distance matrix
        self.total_distance = 0.0
    
    def has_visited_all_nodes(self):
        return len(self.visited_nodes) == self.num_nodes
    
    def visit(self, node):
        # Add the visited node to the list and update the total distance
        self.visited_nodes.append(node)
        if len(self.visited_nodes) > 1:
            prev_node = self.visited_nodes[-2]
            self.total_distance += self.distances[prev_node][node]
    
    def get_current_node(self):
        return self.visited_nodes[-1]
    
    def get_visited_nodes(self):
        return self.visited_nodes
    
    def get_total_distance(self):
        return self.total_distance
        
    def reset(self, start_node):
        self.visited_nodes = [start_node]
        self.total_distance = 0.0
