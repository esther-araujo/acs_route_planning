import networkx as nx
import numpy as np
from ant import Ant 

ants = []

class AntColonySystem:
    def __init__(self, graph, num_ants, num_iterations, alpha, beta, rho, q0):
        self.graph = graph
        self.num_nodes = graph.number_of_nodes()
        self.num_ants = num_ants
        self.num_iterations = num_iterations
        self.alpha = alpha
        self.beta = beta
        self.rho = rho
        self.q0 = q0
        
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
        probabilities = [prob / total_prob for prob in probabilities]
        
        # Choose the next node based on the ACS probability formula
        if np.random.rand() < self.q0:
            next_node = unvisited_nodes[np.argmax(probabilities)]
        else:
            roulette_wheel = np.cumsum(probabilities)
            random_value = np.random.rand()
            next_node = unvisited_nodes[np.searchsorted(roulette_wheel, random_value)]
        
        return next_node

        
    def update_pheromone(self, ant):
        tour_length = ant.total_distance
        
        # Calculate the amount of pheromone to deposit on each edge
        pheromone_deposit = 1 / tour_length
        
        for i in range(len(ant.visited_nodes) - 1):
            from_node = ant.visited_nodes[i]
            to_node = ant.visited_nodes[i + 1]
            
            # Update pheromone level on the edge (from_node, to_node)
            self.pheromone[from_node][to_node] += pheromone_deposit
            self.pheromone[to_node][from_node] += pheromone_deposit  # Assuming an undirected graph

    def evaporate_pheromones(self):
        self.pheromone *= (1 - self.rho)

    def construct_solutions(self, ants):
        # Perform ant tours and update pheromones
        for ant in ants:
            while not ant.has_visited_all_nodes():
                next_node = self.select_next_node(ant)
                ant.visit(next_node)
            
            if ant.total_distance < best_distance:
                best_solution = ant.get_visited_nodes()
                best_distance = ant.get_total_distance()
            
            # Update pheromones using update_pheromone method
            self.update_pheromone(ant)
            
            # Reset ant for the next iteration
            ant.reset(np.random.choice(ant.get_visited_nodes()))

        return best_distance, best_solution
        
    def run(self):
            #ReadInstance
            G = self.graph

            #ComputeDistances
            distances = nx.floyd_warshall_numpy(G)  # Matriz de distancias com floyd marshall
            
            #ComputeNearestNeighborLists
            # .........

            #InitializeAnts
            ants = []

            for _ in range(self.num_ants):
                start_node = np.random.randint(self.num_nodes)
                ant = Ant(start_node, self.num_nodes, distances)
                ants.append(ant)
                
            best_solution = []
            best_distance = float('inf')

            #InitializeParameters

            #InitializeStatistics


            # Pode ser num de iterações, tempo limite de cpu, encontrou uma solução aceitavel dentro de um limite especificado, 
            # algoritmo demonstrou estagnação, etc
            for iteration in range(self.num_iterations):

                #ConstructSolutions
                best_solution, best_distance = self.construct_solutions(ants) 

                #LocalSearch

                #UpdateStatistics               
                # Evaporate pheromones using evaporate_pheromones method
                self.evaporate_pheromones()

            return best_solution, best_distance

# Example usage
G = nx.Graph()
G.add_weighted_edges_from([(0, 1, 200), (0, 2, 202), (0, 3, 212), (1, 2, 125), (1, 3, 212), (2, 3, 40)])

num_ants = 10
num_iterations = 100
alpha = 1.0
beta = 2.0
rho = 0.1
q0 = 0.9

acs = AntColonySystem(G, num_ants, num_iterations, alpha, beta, rho, q0)
best_solution, best_distance = acs.run()
print("Best solution:", best_solution)
print("Best distance:", best_distance)
