#!/usr/bin/env python3
import networkx as nx
import rospy
import numpy as np
from ant import Ant 
from tuw_multi_robot_msgs.msg import Graph
import matplotlib.pyplot as plt
import math

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

        # Enquanto não tiver nenhum nó disponivel, a formiga é resetada pra um nó aleatório
        while not available_nodes:
            ant.reset(np.random.choice(self.num_nodes))
            current_node = ant.visited_nodes[-1]
            available_nodes = [node for node in self.graph.neighbors(current_node)]

        # # Force ant to go to new nodes if its possible
        # if len(available_nodes) > 1:
        #     print(available_nodes)
        #     unvisited = [node for node in available_nodes if node not in [ant.visited_nodes[-2]]]
        #     if unvisited :
        #         available_nodes = unvisited
        #         print(available_nodes)


        probabilities = []
        total_prob = 0.0
        
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
    def update_pheromone(self, ant):
        #Evaporate
        self.evaporate_pheromones()
        
        tour_length = ant.total_distance
        
        epsilon = 1e-10  # evitar que total_prob seja zero, divisao por zero
        # Calculate the amount of pheromone to deposit on each edge
        pheromone_deposit = self.q0 / (tour_length+epsilon)

        # pheromone update based on successful ants
        for ant in ants:
            if ant.found_goal:
                for i in range(len(ant.visited_nodes) - 1):
                    self.pheromone[(ant.visited_nodes[i], ant.visited_nodes[i + 1])] += pheromone_deposit
 

    # Local Pheromone Update for ACS
    def local_pheromone_update(self, from_node, to_node):
        self.pheromone[from_node][to_node] = (1-self.rho_local)*self.pheromone[from_node][to_node]+(self.rho_local*self.tau_0_local)
        self.pheromone[to_node][from_node] = (1-self.rho_local)*self.pheromone[from_node][to_node]+(self.rho_local*self.tau_0_local)

    def evaporate_pheromones(self):
        self.pheromone *= (1 - self.rho)

    def construct_solutions(self, ants):

        best_solution = []
        best_distance = float('inf')

        # Máximo de passos do tour de cada formiga
        max_steps = self.num_nodes *2
        # Perform ant tours and update pheromones
        for ant in ants:
            while ant.get_current_node() is not self.goal and ant.steps < max_steps:
                next_node = self.select_next_node(ant)
                if next_node:
                    curr_node = ant.visited_nodes[-1]
                    edge = self.graph.get_edge_data(curr_node, next_node)
                    ant.visit(next_node, edge)
                    ant.steps = ant.steps + 1
                    #self.local_pheromone_update(curr_node, next_node)
                    if next_node == ant.target_node:
                        ant.found_goal = True
            
            if ant.total_distance < best_distance:
                best_solution = ant.get_visited_nodes()
                best_distance = ant.get_total_distance()
            
            self.update_pheromone(ant)
            
            # Reset ant for the next iteration
            ant.reset(np.random.choice(self.num_nodes))

        return best_solution, best_distance
        
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
            best_solution, best_distance = self.construct_solutions([ant])
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
        G.add_node(vertex["id"], pos=(vertex["path"][0]['x'], vertex["path"][0]['y']), successors=vertex['successors'])

    # Add edges to the graph based on the predecessor and successor information
    for vertex in vertices:
        vertex_len = (len(vertex["path"]))
        x_start_edge = vertex["path"][0]['x']
        y_start_edge = vertex["path"][0]['y']

        x_end_edge = vertex["path"][vertex_len-1]['x']
        y_end_edge = vertex["path"][vertex_len-1]['y']
        for successor_id in vertex["successors"]:
            G.add_edge(vertex["id"], successor_id, weight=calculate_edge_weight(x_start_edge, y_start_edge, x_end_edge, y_end_edge))
    
    # pos = nx.get_node_attributes(G, 'pos')
    # nx.draw(G, pos, with_labels=True, node_size=40, node_color='skyblue', font_size=10, font_color='black', arrows=False)
    # plt.show()

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

    start = 39
    goal = 11
    num_ants = 80
    num_iterations = 100
    alpha = 1.0
    beta = 2.0
    rho = 0.1
    rho_local = 0.1
    tau_0_local = 0.1
    q0 = 0.5

    acs = AntColonySystem(graph=G,start=start, goal=goal, num_ants=num_ants, num_iterations=num_iterations, alpha=alpha, beta=beta, rho=rho, q0=q0, rho_local=rho_local, tau_0_local=tau_0_local)
    best_solution, best_distance = acs.run()

    print("Best solution:", best_solution)
    print("Best distance:", best_distance)
    edges = [(best_solution[i], best_solution[i + 1]) for i in range(len(best_solution) - 1)]


    edge_colors = ['red' if (u, v) in edges or (v, u) in edges else 'gray' for u, v in G.edges()]


    pos = nx.get_node_attributes(G, 'pos')
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=30, edge_color=edge_colors, width=2.0)
    plt.show()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
