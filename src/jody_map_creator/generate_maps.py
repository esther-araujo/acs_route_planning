from PIL import Image, ImageDraw
import os
import yaml
import shutil
import os
import shutil
import re

# Pasta com o gerador de grafos de voronoi
path_multi_robot = "/home/esther/catkin_ws/src/tuw_multi_robot/tuw_multi_robot_demo"
path_route_planning = "/home/esther/catkin_ws/src/acs_route_planning"
path_jody_maps = "/home/esther/TCC/random-rect/maps/result1"

# Loop pelos arquivos no diretório de origem
for arquivo in os.listdir(path_jody_maps):
    if arquivo.endswith('.map'):
        # Constrói o caminho completo do arquivo de origem e destino
        caminho_origem = os.path.join(path_jody_maps, arquivo)
        caminho_destino = os.path.join(path_route_planning+"/src/jody_map_creator/maps", arquivo)

        # Copia o arquivo
        shutil.copy(caminho_origem, caminho_destino)

print("Arquivos .map copiados com sucesso!")

num_maps = 1

map_width = 500
map_height = 500

# Cores
background_color = "white"
obstacle_color = "black"
border_color = "black"

# graph yaml
graph_yaml = {
    'map_topic': "/map",
    'map_inflation': 0.1,
    'segments_topic': "/segments",
    'segment_length': 0.70,
    'opt_crossings': 0.2,
    'opt_end_segments': 0.4
}

# map yaml 
map_yaml = {
    'image': "map.pgm",
    'resolution': 0.06,
    'origin': [-15.000000, -15.000000, 0.000000],
    'negate':0,
    'occupied_thresh': 0.65,
    'free_thresh': 0.196
}
limit = 0

# Rviz config
map_rviz = f"{path_route_planning}/src/map_creator/map.rviz"
# Read the .map file
def parse_map_file(file_path):

    with open(file_path, 'r') as file:
        map_data = file.read().splitlines()

        start_point = None
        end_point = None
        colony = []

        # Initialize a list to store line coordinates
        rectangles = []

        # Flag to indicate whether we are currently reading lines
        reading_lines = False

        # Flag to indicate whether we are currently reading colony
        reading_colony = False

        # Iterate through the lines in the map data
        for line in map_data:
            if line.startswith("StartPoint"):
                start_point_x, start_point_y = tuple(map(int, line.split()[1:]))
                start_point = {"x": start_point_x, "y":start_point_y}
            elif line.startswith("EndPoint"):
                end_point_x, end_point_y = tuple(map(int, line.split()[1:]))
                end_point = {"x": end_point_x, "y": end_point_y}            
                continue  # Skip the "LINES" line itself
            elif line.strip() == "LINES":
                reading_lines = True
                reading_colony = False
                continue
            elif line.strip() == "COLONY":
                reading_colony = True
                continue 

            # Check if we've reached the end of the lines section
            if reading_lines and line.strip() == "DATA":
                break

            # If we are in the lines section, parse the line coordinates
            if reading_lines:
                line_parts = line.split()
                if len(line_parts) == 4:
                    x1, y1, x2, y2 = map(int, line_parts)
                    rectangles.append((x1, y1, x2, y2))

            # If we are in the colony section, parse the acs config
            if reading_colony:
                colony_conf = line.split()
                colony.append(int(colony_conf[0]))
                

    return rectangles, start_point, end_point, colony

def generate_island_map(filename):
    
    # Read test infos from .map file
    rectangles, start_point, end_point, colony = parse_map_file(path_route_planning+"/src/jody_map_creator/maps/"+filename+".map")

    # Create a white background image
    mapa = Image.new("RGB", (map_width, map_height), "white")
    draw = ImageDraw.Draw(mapa)

    draw.rectangle([(0, 0), (map_width - 1, map_height - 1)], outline=border_color)

    rec= {}

    obstacles = [] 

    # Draw rectangles and fill them
    for i in range(0, len(rectangles), 4):
        chave = i // 4  # Crie uma chave com um nome único
        sub_array = rectangles[i:i+4]  # Crie o subarray
        rec[chave] = sub_array 
        
    for chave in rec:
        sub_array = rec[chave]

        # Desenhar obstáculo não preenchido
        # x1,y1,x2,y2 = sub_array[0]
        # x3,y3,x4,y4 = sub_array[1]
        # x5,y5,x6,y6 = sub_array[2]
        # x7,y7,x8,y8 = sub_array[3]
        # draw.line([(x1, y1), (x2, y2)], fill=obstacle_color)
        # draw.line([(x3, y3), (x4, y4)], fill=obstacle_color)
        # draw.line([(x5, y5), (x6, y6)], fill=obstacle_color)
        # draw.line([(x7, y7), (x8, y8)], fill=obstacle_color)

        # Desenhar obstaculo preenchido
        x1,y1,x2,y2 = sub_array[0]
        x3,y3,x4,y4 = sub_array[2]

        # Verifica a sobreposição com os obstáculos existentes
        overlap = False
        for obstacle in obstacles:
            # Rever
            if (x1 < obstacle[2] and x3 > obstacle[0] and
                y1 < obstacle[3] and y3 > obstacle[1]):
                overlap = True
                break

        # Se houver sobreposição, nao salva o mapa
        if overlap:
            return False

        # Registra a posição do obstáculo
        obstacles.append((x1, y1, x3, y3))
        draw.rectangle([(x1, y1), (x3, y3)], fill=obstacle_color)

    
    if not os.path.exists(f"{path_multi_robot}/cfg/maps/{filename}"):
        os.mkdir(f"{path_multi_robot}/cfg/maps/{filename}")
    if not os.path.exists(f"{path_multi_robot}/cfg/graph/{filename}"):
        os.mkdir(f"{path_multi_robot}/cfg/graph/{filename}")

    # Normalizing start_point and end_point
    start_point_x = start_point["x"] * map_yaml["origin"][0]*2 / map_width
    end_point_x = end_point["x"] * map_yaml["origin"][0]*2/ map_width
    start_point_y = start_point["y"] * map_yaml["origin"][0]*2/ map_height
    end_point_y = end_point["y"] * map_yaml["origin"][0]*2/ map_height

    # acs config yaml 
    acs_yaml = {
        'num_ants': colony[0],
        'num_it': colony[1],
        'repetitions': colony[2],
        'start_point_x': start_point_x,
        'start_point_y': start_point_y,
        'end_point_x': end_point_x,
        'end_point_y': end_point_y,
    }

    # Map yaml
    file_path = f"{path_multi_robot}/cfg/maps/{filename}/acs_config.yaml"

    with open(file_path, 'w') as file:
        yaml.dump(acs_yaml, file)
    
    # PGM
    pgm_filename = f"{path_multi_robot}/cfg/maps/{filename}/map.pgm"
    mapa_gray = mapa.convert("L")
    mapa_gray.save(pgm_filename)

    # Map yaml
    file_path = f"{path_multi_robot}/cfg/maps/{filename}/map.yaml"

    with open(file_path, 'w') as file:
        yaml.dump(map_yaml, file)

    # Graph yaml
    file_path = f"{path_multi_robot}/cfg/graph/{filename}/graph.yaml"

    with open(file_path, 'w') as file:
        yaml.dump(graph_yaml, file)

    # Rviz config
    rviz_voronoi = f"{path_multi_robot}/cfg/rviz/{filename}.rviz" 

    shutil.copy(map_rviz, rviz_voronoi)
    return True

files = os.listdir(path_route_planning+"/src/jody_map_creator/maps")

limit = 0

#Clean generated_maps directory
maps_dir = f'{path_route_planning}/src/jody_map_creator/generated_maps/'

if os.path.exists(maps_dir) and os.path.isdir(maps_dir):
    shutil.rmtree(maps_dir)
os.mkdir(maps_dir)

# Loop through the files
for filename in files:
    filename = re.sub(r'\.map$', '', filename)
    print(filename)
    map_generated = generate_island_map(filename)
    if map_generated:
        shutil.copy(f'{path_route_planning}/src/jody_map_creator/maps/{filename}.map', f'{path_route_planning}/src/jody_map_creator/generated_maps/')
        limit+=1
    if limit == 100:
        break
  

print("Mapas de obstaculos gerados com sucesso.")

