#!/usr/bin/env python3
import networkx as nx
import rospy
import numpy as np
from ant import Ant 
from tuw_multi_robot_msgs.msg import Graph
import matplotlib.pyplot as plt
from tabulate import tabulate
import math

ants = []

class AntColonySystem:
    def __init__(self, graph, num_ants, num_iterations, alpha, beta, rho, q0, rho_local, tau_0_local):
        self.graph = graph
        self.num_nodes = graph.number_of_nodes()
        self.num_ants = num_ants
        self.num_iterations = num_iterations
        self.alpha = alpha
        self.beta = beta
        self.rho = rho
        self.q0 = q0

        self.rho_local = rho_local
        self.tau_0_local = tau_0_local
        
        self.pheromone = np.ones((self.num_nodes, self.num_nodes))
        
    def select_next_node(self, ant):
        current_node = ant.visited_nodes[-1]
        unvisited_nodes = [node for node in range(self.num_nodes) if node not in ant.visited_nodes]
        
        probabilities = []
        total_prob = 0.0
        
        for node in unvisited_nodes:
            pheromone_value = self.pheromone[current_node][node]
            heuristic_value = 1.0 / ant.distances[current_node][node]  # Use ant's distances            
            probability = (pheromone_value ** self.alpha) * (heuristic_value ** self.beta)
            probabilities.append(probability)
            total_prob += probability
        
        # Normalize probabilities
        epsilon = 1e-10  # evitar que total_prob seja zero, divisao por zero
        probabilities = [prob / (total_prob + epsilon) for prob in probabilities]
        
        # Choose the next node based on the ACS probability formula
        random_value = np.random.rand()

        if random_value < self.q0:
            next_node = unvisited_nodes[np.argmax(probabilities)]
        else:
            roulette_wheel = np.cumsum(probabilities)
            max_cumulative_prob = roulette_wheel[-1]
            
            # Adjust random_value if it's very close to or slightly above 1
            if random_value >= max_cumulative_prob:
                random_value = max_cumulative_prob - 1e-10  # Adjust to ensure it's within range
            
            next_node_index = np.searchsorted(roulette_wheel, random_value)
            
            # Handle index out of range error
            if next_node_index >= len(unvisited_nodes):
                next_node = unvisited_nodes[-1]  # Choose the last unvisited node
            else:
                next_node = unvisited_nodes[next_node_index]
        
        return next_node

    # Fazer modificações para ACS (esta em AS)
    def update_pheromone(self, ant):
        #Evaporate
        self.evaporate_pheromones()
        
        tour_length = ant.total_distance
        
        # Calculate the amount of pheromone to deposit on each edge
        pheromone_deposit = 1 / tour_length
        
        for i in range(len(ant.visited_nodes) - 1):
            from_node = ant.visited_nodes[i]
            to_node = ant.visited_nodes[i + 1]
            
            # Update pheromone level on the edge (from_node, to_node)
            self.pheromone[from_node][to_node] += pheromone_deposit
            self.pheromone[to_node][from_node] += pheromone_deposit  

    # Local Pheromone Update for ACS
    def local_pheromone_update(self, from_node, to_node):
        self.pheromone[from_node][to_node] = (1-self.rho_local)*self.pheromone[from_node][to_node]+(self.rho_local*self.tau_0_local)
        self.pheromone[to_node][from_node] = (1-self.rho_local)*self.pheromone[from_node][to_node]+(self.rho_local*self.tau_0_local)

    def evaporate_pheromones(self):
        self.pheromone *= (1 - self.rho)

    def construct_solutions(self, ants):
        # Perform ant tours and update pheromones
        best_solution = []
        best_distance = float('inf')
        for ant in ants:
            while not ant.has_visited_all_nodes():
                next_node = self.select_next_node(ant)
                ant.visit(next_node)
                # Always update local pheromone after an ant moves to the next node
                self.local_pheromone_update(ant.visited_nodes[-2], next_node) 

            if ant.total_distance < best_distance:
                best_solution = ant.get_visited_nodes()
                best_distance = ant.get_total_distance()
            
            self.update_pheromone(ant)
            
            # Reset ant for the next iteration
            ant.reset(np.random.choice(ant.get_visited_nodes()))

        return best_solution, best_distance
        
    def run(self):
            #ReadInstance
            G = self.graph

            #ComputeDistances
            # Create a mapping of node IDs to matrix indices
            n_nodes = len(G.nodes())
            distance_matrix = np.full((n_nodes, n_nodes), np.inf)

            # Fill the diagonal with zeros
            np.fill_diagonal(distance_matrix, 0)

            # Compute the shortest path distances
            for node in G.nodes():
                shortest_paths = nx.shortest_path_length(G, source=node, weight='weight')
                for target, distance in shortest_paths.items():
                    distance_matrix[node-1][target-1] = distance

            ## To verify distance matrix
            # node_ids = list(G.nodes())
            # table_headers = [""] + node_ids
            # table_data = [[node_id] + distances_row.tolist() for node_id, distances_row in zip(node_ids, distance_matrix)]
            # table = tabulate(table_data, headers=table_headers, tablefmt="grid")
            # print("Distance Matrix:")
            # print(table)
            
            #ComputeNearestNeighborLists
            # ......... ???

            #ComputeChoiceInformation
            # ......... ???

            #InitializeAnts
            ants = []

            for _ in range(self.num_ants):
                start_node = np.random.randint(self.num_nodes)
                ant = Ant(start_node, self.num_nodes, distance_matrix)
                ants.append(ant)
                
            best_solution = []
            best_distance = float('inf')

            # Pode ser num de iterações, tempo limite de cpu, encontrou uma solução aceitavel dentro de um limite especificado, 
            # algoritmo demonstrou estagnação, etc
            for _ in range(self.num_iterations):
                #ConstructSolutions
                best_solution, best_distance = self.construct_solutions(ants) 
                #LocalSearch
                # .........

            return best_solution, best_distance

def xml_to_dict(input_data):
    message_list = []
    for graph_msg in input_data:
        message_dict = {
            "id": graph_msg.id,
            "valid": graph_msg.valid,
            "path": [{"x": point.x, "y": point.y, "z": point.z} for point in graph_msg.path],
            "weight": graph_msg.weight,
            "width": graph_msg.width,
            "successors": list(graph_msg.successors),
            "predecessors": list(graph_msg.predecessors)
        }
        message_list.append(message_dict)

    return message_list

def create_nx_graph(vertice):
    vertices = xml_to_dict(vertice)
    
    G = nx.Graph()

    # Add nodes with coordinates and edges
    for vertex in vertices:
        G.add_node(vertex["id"], pos=(vertex["path"][0]['x'], vertex["path"][0]['y']))

    vertex_len = (len(vertex["path"]))
    x_start_edge = vertex["path"][0]['x']
    y_start_edge = vertex["path"][0]['y']

    x_end_edge = vertex["path"][vertex_len-1]['x']
    y_end_edge = vertex["path"][vertex_len-1]['y']

    # Add edges to the graph based on the predecessor and successor information
    for vertex in vertices:
        for successor_id in vertex["successors"]:
            G.add_edge(vertex["id"], successor_id, weight=calculate_edge_weight(x_start_edge, y_start_edge, x_end_edge, y_end_edge))

    # Draw the graph using Matplotlib
    pos = nx.get_node_attributes(G, 'pos')
    nx.draw(G, pos, with_labels=True, node_size=40, node_color='skyblue', font_size=10, font_color='black', arrows=False)
    plt.show()

    return G

def calculate_edge_weight(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.vertices)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('acs_running', anonymous=True)

    graph_data = rospy.wait_for_message('segments', Graph)

    G = create_nx_graph(graph_data.vertices)

    num_ants = 10
    num_iterations = 200
    alpha = 1.0
    beta = 2.0
    rho = 0.1
    rho_local = 0.1
    tau_0_local = 0.1
    q0 = 0.5

    acs = AntColonySystem(G, num_ants, num_iterations, alpha, beta, rho, q0, rho_local, tau_0_local)
    best_solution, best_distance = acs.run()

    print("Best solution:", best_solution)
    print("Best distance:", best_distance)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
