#!/usr/bin/env python3
import networkx as nx
import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import MapMetaData
from tuw_multi_robot_msgs.msg import Graph
import matplotlib.pyplot as plt
import math
from scipy.spatial import cKDTree
import yaml

# Pasta com o gerador de grafos de voronoi
path_route_planning = "/home/esther/catkin_ws/src/acs_route_planning"

def heuristic(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def get_lowest_f_node_from(open_list):
    lowest_node = list(open_list)[0]
    for node in open_list:
        if node.f() < lowest_node.f():
            lowest_node = node
    return lowest_node

def way(node_current_x, node_current_y, node_successor_x, node_successor_y):
    return heuristic(node_current_x, node_current_y, node_successor_x, node_successor_y)

class Node:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.parent = None
        self.g = 0
        self.h = 0
        self.w = 0 
        self.cov = None

    def f(self):
        return self.g + self.h
    
class ASTAR:
    def __init__(self, graph, start, goal, start_x, start_y, goal_x, goal_y):
        self.graph = graph
        self.start = start
        self.goal = goal
        self.num_nodes = graph.number_of_nodes()
        self.start_x = start_x
        self.start_y = start_y
        self.goal_x = goal_x
        self.goal_y = goal_y

    def run(self):

        open_list = []
        close_list = []
        node_current = Node(self.start, self.start_x, self.start_y)
        node_current.h = heuristic(self.start_x, self.start_y, self.goal_x, self.goal_y)
        open_list.append(node_current)

        while len(open_list) > 0:
            node_current = get_lowest_f_node_from(open_list)
            if node_current.id == self.goal:
                break
            print(node_current.id)
            for successor_id in self.graph.neighbors(node_current.id):
                node_succ_x, node_succ_y = self.graph.nodes[successor_id]['pos']
                node_successor = Node(successor_id, node_succ_x, node_succ_y)
                node_curr_x, node_curr_y = self.graph.nodes[node_current.id]['pos']
                successor_current_cost = node_current.g + way(node_curr_x, node_curr_y, node_succ_x, node_succ_y)
                if node_successor in open_list:
                    if node_successor.g <= successor_current_cost:
                        continue
                elif node_successor in close_list:
                    if node_successor.g <= successor_current_cost:
                        continue
                    close_list.remove(node_successor)
                    open_list.append(node_successor)
                else:
                    open_list.append(node_successor)
                    node_x, node_y = self.graph.nodes[node_successor.id]['pos']
                    node_successor.h = heuristic(node_x, node_y, self.goal_x, self.goal_y)
                node_successor.g = successor_current_cost
                node_successor.parent = node_current

            open_list.remove(node_current)
            close_list.append(node_current)

        if node_current.id != self.goal:
            return None
        else:
            path = []
            while node_current is not None:
                path.append(node_current.id)
                node_current = node_current.parent
            return list(reversed(path))


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


    map_metadata = rospy.wait_for_message('map_metadata', MapMetaData)

    map_compensation = abs(map_metadata.origin.position.x)

    if acs_config != "simulate":
        with open(acs_config, "r") as yaml_file:

            # Parse the YAML data
            data = yaml.load(yaml_file, Loader=yaml.FullLoader)

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

        astar = ASTAR(graph=G,start=start, goal=goal, start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y)
        best_solution = astar.run()
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
        log_path = path_route_planning+'/tests/a_star_logs/'

        # Specify the file name and extension
        log_file = f'{room}.log'

        file_path = log_path + log_file

        # The content you want to write to the file
        file_content  = {
            'goalFounded': goal_founded,
            'startNode': start,
            'endNode': goal,
            'distance': total_cost,
            'nNodesBP': len(best_solution),
            'path': best_solution

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

        astar = ASTAR(graph=G,start=start, goal=goal, start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y)
        best_solution = astar.run()

        if goal in best_solution:
            idx = best_solution.index(goal)
            best_solution = remove_loops_from_path(best_solution[:idx+1])
            total_cost, best_solution = calculate_path_cost(G, best_solution)
            #move_robot.publish("move")
            print("GOAL FOUNDED")
            print("Best solution:", best_solution)
            print("Best distance:", total_cost)

        # Filtrar as posições apenas para os nós no caminho
        path_positions = {node: {'x': round(G.nodes[node]['pos'][0],2), 'y': round(G.nodes[node]['pos'][1],2)} for node in best_solution}

        #print(path_positions)

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
