import networkx as nx

G = nx.Graph()
path_nodes = [1, 2, 3, 4]
G.add_path(path_nodes)  # Adds a path of nodes as edges
