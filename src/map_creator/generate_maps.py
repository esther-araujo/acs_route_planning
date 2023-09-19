from PIL import Image, ImageDraw
import random
import os
import yaml
import shutil

# Crie um diretório para armazenar as imagens geradas
# if not os.path.exists("generated_maps"):
#     os.mkdir("generated_maps")

# Pasta com o gerador de grafos de voronoi
path_multi_robot = "/home/esther/catkin_ws/src/tuw_multi_robot/tuw_multi_robot_demo"
path_route_planning = "/home/esther/catkin_ws/src/acs_route_planning"
num_maps = 5


map_width = 800
map_height = 600

# Cores
background_color = "white"
obstacle_color = "black"
border_color = "black"

# graph yaml
graph = {
    'map_topic': "/map",
    'map_inflation': 0.1,
    'segments_topic': "/segments",
    'segment_length': 0.6,
    'opt_crossings': 0.2,
    'opt_end_segments': 0.5
}

# map yaml 
map = {
    'image': "map.pgm",
    'resolution': 0.032,
    'origin': [-8.000000, -8.000000, 0.000000],
    'negate':0,
    'occupied_thresh': 0.65,
    'free_thresh': 0.196
}

# Rviz config
map_rviz = f"{path_route_planning}/src/map_creator/map.rviz"

def generate_island_map(width, height, num_obstacles):
    mapa = Image.new("RGB", (width, height), background_color)
    draw = ImageDraw.Draw(mapa)

    # Cria um contorno preto
    draw.rectangle([(0, 0), (width - 1, height - 1)], outline=border_color)

    obstacles = [] 

    # Gera as ilhas aleatoriamente
    for _ in range(num_obstacles):
        island_size = random.randint(20, 150)
        x1 = random.randint(0, width - island_size)
        y1 = random.randint(0, height - island_size)
        x2 = x1 + island_size
        y2 = y1 + island_size

        # Verifica a sobreposição com os obstáculos existentes
        overlap = False
        for obstacle in obstacles:
            if (x1 < obstacle[2] and x2 > obstacle[0] and
                y1 < obstacle[3] and y2 > obstacle[1]):
                overlap = True
                break

        # Se houver sobreposição, gera um novo obstáculo
        if overlap:
            continue

        draw.rectangle([(x1, y1), (x2, y2)], fill=obstacle_color)

        # Registra a posição do obstáculo
        obstacles.append((x1, y1, x2, y2))

    return mapa

for i in range(num_maps):
    num_obstacles = random.randint(5, 20)
    mapa = generate_island_map(map_width, map_height, num_obstacles)

    if not os.path.exists(f"{path_multi_robot}/cfg/maps/map{i + 1}"):
        os.mkdir(f"{path_multi_robot}/cfg/maps/map{i + 1}")
    if not os.path.exists(f"{path_multi_robot}/cfg/graph/map{i + 1}"):
        os.mkdir(f"{path_multi_robot}/cfg/graph/map{i + 1}")
    
    # PGM
    pgm_filename = f"{path_multi_robot}/cfg/maps/map{i + 1}/map.pgm"
    mapa_gray = mapa.convert("L")
    mapa_gray.save(pgm_filename)

    # Map yaml
    file_path = f"{path_multi_robot}/cfg/maps/map{i + 1}/map.yaml"

    with open(file_path, 'w') as file:
        yaml.dump(map, file)

    # Graph yaml
    file_path = f"{path_multi_robot}/cfg/graph/map{i + 1}/graph.yaml"

    with open(file_path, 'w') as file:
        yaml.dump(map, file)

    # Rviz config
    rviz_voronoi = f"{path_multi_robot}/cfg/rviz/map{i+1}.rviz" 

    shutil.copy(map_rviz, rviz_voronoi)

print("Mapas de obstaculos gerados com sucesso.")
