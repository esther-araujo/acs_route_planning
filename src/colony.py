#!/usr/bin/env python3
import networkx as nx
import rospy
import numpy as np
from ant import Ant 
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from tuw_multi_robot_msgs.msg import Graph
from  tuw_multi_robot_msgs.msg import Vertex
import matplotlib.pyplot as plt
import math
from scipy.spatial import cKDTree
from itertools import groupby

ants = []

class AntColonySystem:
    def __init__(self, graph, start, goal, num_ants, num_iterations, alpha, beta, rho, q0, rho_local, tau_0_local):
        self.graph = graph
        self.start = start
        self.num_nodes = graph.number_of_nodes()
        self.num_ants = num_ants
        self.num_iterations = num_iterations
        self.alpha = alpha
        self.beta = beta
        self.rho = rho
        self.q0 = q0
        self.goal = goal


        self.rho_local = rho_local
        self.tau_0_local = tau_0_local
        
        self.pheromone = np.ones((self.num_nodes, self.num_nodes))
        
    def select_next_node(self, ant):
        current_node = ant.visited_nodes[-1]

        # Get available nodes
        available_nodes = [node for node in self.graph.neighbors(current_node)]

        # Mínimo local, volta pro ponto no grafo onde tem mais de duas opções (sai do mínimo) e remove o feromonio do 
        # caminho que leva ao mínimo local
        if len(available_nodes) == 1 and current_node is not self.start:
            pre = ant.visited_nodes[len(ant.visited_nodes)-2]
            for curr in reversed(ant.visited_nodes[:-1]):
                # remove pheromone
                self.remove_pheromone(curr, pre)
                available_nodes = available_nodes = [node for node in self.graph.neighbors(curr)]
                current_node = curr
                ant.visited_nodes.append(current_node)
                if len(available_nodes) > 2:
                    available_nodes = available_nodes = [node for node in self.graph.neighbors(curr) if node is not self.start]
                    break 
                pre = curr
                
        # Force ant to go to new nodes if its possible
        if len(available_nodes) > 1:
            unvisited = [node for node in available_nodes if node not in ant.visited_nodes]
            if unvisited :
                available_nodes = unvisited

        probabilities = []
        total_prob = 0.0

        if self.goal in available_nodes:
            return self.goal
        
        # Heuristica sem a matriz de distancias
        for node in available_nodes:
            pheromone_value = self.pheromone[current_node][node]
            heuristic_value = 1.0 / self.graph.get_edge_data(current_node, node)['weight'] 
            probability = (pheromone_value ** self.alpha) * (heuristic_value ** self.beta)
            probabilities.append(probability)
            total_prob += probability
        
        # Normalize probabilities
        epsilon = 1e-10  # evitar que total_prob seja zero, divisao por zero
        probabilities = [prob / (total_prob + epsilon) for prob in probabilities]
        
        # Choose the next node based on the ACS probability formula
        random_value = np.random.rand()

        if random_value < self.q0:
            next_node = available_nodes[np.argmax(probabilities)]
        else:
            roulette_wheel = np.cumsum(probabilities)
            max_cumulative_prob = roulette_wheel[-1]
            
            # Adjust random_value if it's very close to or slightly above 1
            if random_value >= max_cumulative_prob:
                random_value = max_cumulative_prob - 1e-10  # Adjust to ensure it's within range
            
            next_node_index = np.searchsorted(roulette_wheel, random_value)
            
            # Handle index out of range error
            if next_node_index >= len(available_nodes):
                next_node = available_nodes[-1]  # Choose the last unvisited node
            else:
                next_node = available_nodes[next_node_index]
        
        return next_node

    # Ant Colony System - Pheromone is only update by the successfull ants
    def update_pheromone(self, ants):
        #Evaporate
        self.evaporate_pheromones()
        # pheromone update based on successful ants
        for ant in ants:
            tour_length = ant.total_distance
            epsilon = 1e-10  # evitar que total_prob seja zero, divisao por zero
            # Calculate the amount of pheromone to deposit on each edge
            pheromone_deposit = self.q0 / (tour_length+epsilon)
            for i in range(len(ant.visited_nodes) - 1):
                self.pheromone[(ant.visited_nodes[i], ant.visited_nodes[i + 1])] += pheromone_deposit
 

    # Local Pheromone Update for ACS
    def local_pheromone_update(self, from_node, to_node):
        self.pheromone[from_node][to_node] = (1-self.rho_local)*self.pheromone[from_node][to_node]+(self.rho_local*self.tau_0_local)
        self.pheromone[to_node][from_node] = (1-self.rho_local)*self.pheromone[from_node][to_node]+(self.rho_local*self.tau_0_local)
    
    def remove_pheromone(self, from_node, to_node):
        self.pheromone[from_node][to_node] = 0.0
        self.pheromone[to_node][from_node] = 0.0

    def evaporate_pheromones(self):
        self.pheromone *= (1 - self.rho)

    def construct_solutions(self, ants):

        best_solution = []
        best_distance = float('inf')

        # Máximo de passos do tour de cada formiga
        max_steps = self.num_nodes * 2
        ants_done_tour = []
        # Perform ant tours and update pheromones
        for ant in ants:
            while ant.steps < max_steps:
                next_node = self.select_next_node(ant)
                curr_node = ant.visited_nodes[-1]
                edge = self.graph.get_edge_data(curr_node, next_node)
                ant.visit(next_node, edge)
                ant.steps = ant.steps + 1
                self.local_pheromone_update(curr_node, next_node)
                if next_node == ant.target_node:
                    ants_done_tour.append(ant)
                    
            if ant.total_distance < best_distance:
                best_solution = ant.get_visited_nodes()
                best_distance = ant.get_total_distance()

            self.update_pheromone(ants_done_tour)
            # Reset ant for the next iteration
            ant.reset(np.random.choice(self.num_nodes))
        return best_solution
        
    def run(self):
            #InitializeAnts
            ants = []

            for _ in range(self.num_ants):
                start_node = np.random.randint(self.num_nodes)
                ant = Ant(start_node, self.goal, self.num_nodes, found_goal=False)
                ants.append(ant)

            # Pode ser num de iterações, tempo limite de cpu, encontrou uma solução aceitavel dentro de um limite especificado, 
            # algoritmo demonstrou estagnação, etc
            for _ in range(self.num_iterations):
                #ConstructSolutions
                self.construct_solutions(ants)
                #LocalSearch
                # .........
            # Última formiga, saindo do ponto inicial, utilizando as informações prévias
            ant = Ant(self.start, self.goal, self.num_nodes, found_goal=False)
            best_solution = self.construct_solutions([ant])
            return best_solution

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
    positions = {}

    # Add nodes with coordinates and edges
    i = 0
    for vertex in vertices:
        vertex_len = (len(vertex["path"]))
        start_edge = (vertex["path"][0]['x'], vertex["path"][0]['y'])

        if start_edge not in positions.values():
            G.add_node(i, pos=start_edge)
            positions[i] = start_edge
            i+=1
        end_edge = (vertex["path"][vertex_len-1]['x'], vertex["path"][vertex_len-1]['y'])
        if end_edge not in positions.values():
            G.add_node(i, pos=end_edge)
            positions[i] = end_edge
            i+=1

        start_node = None
        end_node = None
        for node, data in G.nodes(data=True):
            if 'pos' in data and data['pos'] == start_edge:
                start_node = node
            elif 'pos' in data and data['pos'] == end_edge:
                end_node = node

        # Check if both nodes were found
        if start_node == None and end_node == None:
            print("Nodes with the specified positions not found.")
        else:
            # Add an edge between the identified nodes
            G.add_edge(start_node, end_node, weight=calculate_edge_weight(start_edge[0], start_edge[1], end_edge[0], end_edge[1]))

    return G

def calculate_edge_weight(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def find_closest_node_efficient(graph, kdtree, target_node):
    position_target = target_node['pos']
    _, index = kdtree.query(position_target)
    closest_node = list(graph.nodes())[index]
    return closest_node

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('acs_running', anonymous=True)
    #move_robot = rospy.Publisher('/move_robot', String, queue_size=10)

    graph_data = rospy.wait_for_message('segments', Graph)

    G = create_nx_graph(graph_data.vertices)

    # Initialize a counter to keep track of received messages
    message_count = 0
    required_message_count = 2  # Change this to the number of messages you want to wait for
    positions = nx.get_node_attributes(G, 'pos')
    v_count = len(positions)
    position_list = list(positions.values())


    while message_count < required_message_count:
        try:
            # Wait for a message on the desired topic with a timeout
            message = rospy.wait_for_message('/clicked_point', PointStamped)  # Adjust the topic and message type
            id_v = v_count if message_count == 0 else (v_count+1)
            # 8 x 8 do mapa
            G.add_node(id_v, pos=(message.point.x+8, message.point.y+8))
            
            kdtree = cKDTree(position_list)

            node = G.nodes[id_v]

            # Use the find_closest_node_efficient function to find the closest node for each node and create edges
            closest = find_closest_node_efficient(G, kdtree, node)
            if closest is not None and closest != node:
                G.add_edge(id_v, closest, weight=calculate_edge_weight(message.point.x+8, message.point.y+8, G.nodes[closest]['pos'][0], G.nodes[closest]['pos'][1]))

            message_count += 1
        except rospy.ROSException:
            rospy.logwarn("Timeout waiting for a message.")

    start = v_count
    goal = v_count + 1
    num_ants = 100
    num_iterations = 70
    alpha = 1.0
    beta = 2.0
    rho = 0.1
    rho_local = 0.1
    tau_0_local = 0.1
    q0 = 0.5

    acs = AntColonySystem(graph=G,start=start, goal=goal, num_ants=num_ants, num_iterations=num_iterations, alpha=alpha, beta=beta, rho=rho, q0=q0, rho_local=rho_local, tau_0_local=tau_0_local)
    best_solution = acs.run()

    if goal in best_solution:
        idx = best_solution.index(goal)
        best_solution = remove_loops_from_path(best_solution[:idx+1])
        total_cost, best_solution = calculate_path_cost(G, best_solution)
        #move_robot.publish("move")
        print("GOAL FOUNDED")
        print("Best solution:", best_solution)
        print("Best distance:", total_cost)

    edges = [(best_solution[i], best_solution[i + 1]) for i in range(len(best_solution) - 1)]

    edge_colors = ['red' if (u, v) in edges or (v, u) in edges else 'gray' for u, v in G.edges()]

    pos = nx.get_node_attributes(G, 'pos')
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=30, edge_color=edge_colors, width=2.0)
    plt.show()

def calculate_path_cost(graph, path):
    cost = 0  # Initialize the cost to 0
    current_node = path[0]  # Start from the first node in the path
    final_path = [current_node]

    for i in range(1, len(path)):
        next_node = path[i]  # Get the next node in the path
        final_path.append(next_node)
        # Check if the edge exists in the graph
        if graph.has_edge(current_node, next_node):
            edge_weight = graph[current_node][next_node]['weight']
            cost += edge_weight
        else:
            raise ValueError(f"Edge ({current_node}, {next_node}) does not exist in the graph")
        current_node = next_node  # Move to the next node

    return cost, final_path

def remove_loops_from_path(path):
    unique_path = []
    visited = set()

    for vertex in path:
        if vertex not in visited:
            unique_path.append(vertex)
            visited.add(vertex)
        elif vertex == unique_path[0]:
            # If we return to the starting vertex, consider it as part of the unique path
            unique_path.append(vertex)
            visited.add(vertex)
        else:
            # If we revisit a non-starting vertex, remove it and all vertices in between
            while unique_path[-1] != vertex:
                visited.remove(unique_path.pop())
    return unique_path

if __name__ == '__main__':
    listener()
