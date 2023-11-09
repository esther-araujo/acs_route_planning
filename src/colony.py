#!/usr/bin/env python3
import networkx as nx
import rospy
import numpy as np
from ant import Ant 
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import MapMetaData
from tuw_multi_robot_msgs.msg import Graph
import matplotlib.pyplot as plt
import math
from scipy.spatial import cKDTree
import yaml

# Pasta com o gerador de grafos de voronoi
path_route_planning = "/home/esther/catkin_ws/src/acs_route_planning"

ants = []

class AntColonySystem:
    def __init__(self, graph, start, goal, num_ants, num_iterations, alpha, beta, rho, q0, rho_local, tau_0_local, start_x, start_y, goal_x, goal_y):
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
        self.start_x = start_x
        self.start_y = start_y
        self.goal_x = goal_x
        self.goal_y = goal_y


        self.rho_local = rho_local
        self.tau_0_local = tau_0_local
        
        self.pheromone = np.ones((self.num_nodes, self.num_nodes))
        
    def select_next_node(self, ant):
        current_node = ant.visited_nodes[-1]

        # Get available nodes
        available_nodes = [node for node in self.graph.neighbors(current_node)]

        # Mínimo local, sem tratamento de deadlock
        if len(available_nodes) == 1 and current_node is not self.start:
            return None

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
            curr_x, curr_y = self.graph.nodes[current_node]['pos']
            node_x, node_y = self.graph.nodes[node]['pos']
            heuristic_value = original_heuristic(curr_x, curr_y, node_x, node_y)
            probability = (pheromone_value ** self.alpha) * (heuristic_value ** self.beta)
            probabilities.append(probability)
            total_prob += probability
        
        # Normalize probabilities
        epsilon = 1e-10  # evitar que total_prob seja zero, divisao por zero
        probabilities_exploit = probabilities
        probabilities_explore = [prob / (total_prob + epsilon) for prob in probabilities]
        
        # Choose the next node based on the ACS probability formula
        random_value = np.random.rand()

        if random_value <= self.q0:
            next_node = available_nodes[np.argmax(probabilities_exploit)]
        else:
            random_number = np.random.rand() 
            cumulative_prob = 0
            next_node = available_nodes[-1]
            for i, element in enumerate(probabilities_explore):
                cumulative_prob += element
                if random_number <= cumulative_prob:
                    next_node = available_nodes[i]
        
        return next_node

    # Ant Colony System - Pheromone is only update by the successfull ants
    def global_pheromone_update(self, ants, best_ant):
        #Evaporate
        self.evaporate_pheromones()
        # pheromone update based on successful ants
        for ant in ants:
            epsilon = 1e-10
            delta_tau = 1/(ant.total_distance+epsilon)
            for node in range(len(ant.visited_nodes) - 1):
                i = ant.visited_nodes[node]
                j = ant.visited_nodes[node + 1]
                #Evaporate and Deposite
                if self.has_i_to_j_sequence(best_ant.visited_nodes, i, j):
                    self.pheromone[i, j] = ((1 - self.rho) * self.pheromone[i, j]) + (self.rho * delta_tau)
                    self.pheromone[j, i] = self.pheromone[i, j]
                    
    def has_i_to_j_sequence(self, arr, i, j):
        # Testa se i e j estão no array e se estão em sequencia
        if i in arr and j in arr and (abs(arr.index(i) - arr.index(j)) == 1):
            return True
        return False
    
    # Local Pheromone Update for ACS
    def local_pheromone_update(self, from_node, to_node):
        self.pheromone[from_node][to_node] = (1-self.rho_local)*self.pheromone[from_node][to_node]+(self.rho_local*self.tau_0_local)
        self.pheromone[to_node][from_node] = self.pheromone[from_node][to_node]
    
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
        best_ant = []
        # Perform ant tours and update pheromones
        for ant in ants:
            while ant.steps < max_steps:
                next_node = self.select_next_node(ant)
                if next_node == None:
                    break
                curr_node = ant.visited_nodes[-1]
                edge = self.graph.get_edge_data(curr_node, next_node)
                ant.visit(next_node, edge)
                ant.steps = ant.steps + 1
                if next_node == ant.target_node:
                    ants_done_tour.append(ant)
                self.local_pheromone_update(curr_node, next_node)
            if ant.total_distance < best_distance and self.goal in ant.visited_nodes:
                best_solution = ant.get_visited_nodes()
                best_distance = ant.get_total_distance()
                best_ant = ant
            
        self.global_pheromone_update(ants_done_tour, best_ant)
        return best_solution, best_distance
        
    def run(self):
        #InitializeAnts
        best_distance = float('inf')
        best_path = []

        # Pode ser num de iterações, tempo limite de cpu, encontrou uma solução aceitavel dentro de um limite especificado, 
        # algoritmo demonstrou estagnação, etc
        for _ in range(self.num_iterations):
            ants = []
            for _ in range(self.num_ants):
                start_node = self.start
                ant = Ant(start_node, self.goal, self.num_nodes, found_goal=False)
                ants.append(ant)
            #ConstructSolutions
            path, distance = self.construct_solutions(ants)
            if distance < best_distance:
                best_path = path
        if not best_path:
            best_path = ants[-1].get_visited_nodes()
        return best_path

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

#distancia euclidiana
def calculate_edge_weight(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def original_heuristic(x1, y1, x2, y2):
    return 1 / (calculate_edge_weight(x1, y1, x2, y2))

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
    acs_config = rospy.get_param("~config", "simulate")  # Default simulate if there is no acs yaml param

    room = rospy.get_param("~room", "cave")  # Default cave if there is room param

    graph_data = rospy.wait_for_message('segments', Graph)

    G = create_nx_graph(graph_data.vertices)

    # Initialize a counter to keep track of received messages
    message_count = 0
    required_message_count = 2  # Change this to the number of messages you want to wait for
    positions = nx.get_node_attributes(G, 'pos')
    v_count = len(positions)
    position_list = list(positions.values())

    start = v_count
    goal = v_count + 1
    num_ants = 50 # default
    num_iterations = 30 # default
    num_rep = 1 # default
    alpha = 1.0
    beta = 2.0
    rho = 0.1
    rho_local = 0.1
    tau_0_local = 1
    q0 = 0.5

    map_metadata = rospy.wait_for_message('map_metadata', MapMetaData)

    map_compensation = abs(map_metadata.origin.position.x)

    if acs_config != "simulate":
        with open(acs_config, "r") as yaml_file:

            # Parse the YAML data
            data = yaml.load(yaml_file, Loader=yaml.FullLoader)

            # ACS PARAMETERS
            num_rep = data["repetitions"]

            start_x = data["start_point_x"]
            start_y = data["start_point_y"]
            goal_x = data["end_point_x"]
            goal_y = data["end_point_y"]

            # START
            G.add_node(start, pos=(data["start_point_x"], data["start_point_y"]))
            
            kdtree = cKDTree(position_list)

            node = G.nodes[start]

            # Use the find_closest_node_efficient function to find the closest node for each node and create edges
            closest = find_closest_node_efficient(G, kdtree, node)
            if closest is not None and closest != node:
                G.add_edge(start, closest, weight=calculate_edge_weight(data["start_point_x"], data["start_point_y"], G.nodes[closest]['pos'][0], G.nodes[closest]['pos'][1]))
            
            # GOAL
            G.add_node(goal, pos=(data["end_point_x"], data["end_point_y"]))
            
            kdtree = cKDTree(position_list)

            node = G.nodes[goal]

            # Use the find_closest_node_efficient function to find the closest node for each node and create edges
            closest = find_closest_node_efficient(G, kdtree, node)
            if closest is not None and closest != node:
                G.add_edge(goal, closest, weight=calculate_edge_weight(data["start_point_x"], data["start_point_y"], G.nodes[closest]['pos'][0], G.nodes[closest]['pos'][1]))

        acs = AntColonySystem(graph=G,start=start, goal=goal, num_ants=num_ants, num_iterations=num_iterations, alpha=alpha, beta=beta, rho=rho, q0=q0, rho_local=rho_local, tau_0_local=tau_0_local, start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y)
        best_solution = acs.run()
        total_cost, best_solution = calculate_path_cost(G, best_solution)
        goal_founded = False

        if goal in best_solution:
            goal_founded = True
            idx = best_solution.index(goal)
            best_solution = remove_loops_from_path(best_solution[:idx+1])
            total_cost, best_solution = calculate_path_cost(G, best_solution)
            #move_robot.publish("move")
            print("GOAL FOUNDED")
            print("Best solution:", best_solution)
            print("Best distance:", total_cost)
            
        # GENERATE LOG FILE
        # Specify the directory path where you want to save the file
        log_path = path_route_planning+'/tests/acs_logs/'

        # Specify the file name and extension
        log_file = f'{room}.log'

        file_path = log_path + log_file

        # The content you want to write to the file
        file_content  = {
            'goalFounded': goal_founded,
            'startNode': start,
            'endNode': goal,
            'Ants': num_ants,
            'Iterations': num_iterations,
            'Repetitions': num_rep,
            'distance': total_cost,
            'nNodesBP': len(best_solution)
        }   

        # Attempt to create and write to the file
        try:
            with open(file_path, 'w') as file:
                yaml.dump(file_content, file, default_flow_style=False)

        except FileNotFoundError:
            print(f"Directory '{log_path}' does not exist.")
        except Exception as e:
            print(f"An error occurred: {e}")

        # edges = [(best_solution[i], best_solution[i + 1]) for i in range(len(best_solution) - 1)]

        # edge_colors = ['red' if (u, v) in edges or (v, u) in edges else 'gray' for u, v in G.edges()]

        # pos = nx.get_node_attributes(G, 'pos')
        # nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=30, edge_color=edge_colors, width=2.0)
        # plt.show()

    else:
        while message_count < required_message_count:
            try:
                # Wait for a message on the desired topic with a timeout
                message = rospy.wait_for_message('/clicked_point', PointStamped)  # Adjust the topic and message type
                id_v = v_count if message_count == 0 else (v_count+1)
                if message_count == 0:
                    start_x = message.point.x+map_compensation
                    start_y = message.point.y+map_compensation
                if message_count == 1:
                    goal_x = message.point.x+map_compensation
                    goal_y = message.point.y+map_compensation
                # 16 x 16 do mapa
                G.add_node(id_v, pos=(message.point.x+map_compensation, message.point.y+map_compensation))
                
                kdtree = cKDTree(position_list)

                node = G.nodes[id_v]

                # Use the find_closest_node_efficient function to find the closest node for each node and create edges
                closest = find_closest_node_efficient(G, kdtree, node)
                if closest is not None and closest != node:
                    G.add_edge(id_v, closest, weight=calculate_edge_weight(message.point.x+map_compensation, message.point.y+map_compensation, G.nodes[closest]['pos'][0], G.nodes[closest]['pos'][1]))

                message_count += 1
            except rospy.ROSException:
                rospy.logwarn("Timeout waiting for a message.")

        acs = AntColonySystem(graph=G,start=start, goal=goal, num_ants=num_ants, num_iterations=num_iterations, alpha=alpha, beta=beta, rho=rho, q0=q0, rho_local=rho_local, tau_0_local=tau_0_local, start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y)
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
