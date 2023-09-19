from PIL import Image, ImageDraw
import random
import os

# Crie um diretório para armazenar as imagens geradas
if not os.path.exists("generated_maps"):
    os.mkdir("generated_maps")

num_maps = 100


map_width = 800
map_height = 600

# Cores
background_color = "white"
obstacle_color = "black"
border_color = "black"

def generate_island_map(width, height, num_obstacles):
    mapa = Image.new("RGB", (width, height), background_color)
    draw = ImageDraw.Draw(mapa)

    # Cria um contorno preto
    draw.rectangle([(0, 0), (width - 1, height - 1)], outline=border_color)

    # Gera as ilhas aleatoriamente
    for _ in range(num_obstacles):
        island_size = random.randint(20, 150)
        x1 = random.randint(0, width - island_size)
        y1 = random.randint(0, height - island_size)
        x2 = x1 + island_size
        y2 = y1 + island_size

        # Desenhe a ilha
        draw.rectangle([(x1, y1), (x2, y2)], fill=obstacle_color)

    return mapa

for i in range(num_maps):
    num_obstacles = random.randint(5, 20)
    mapa = generate_island_map(map_width, map_height, num_obstacles)

    png_filename = f"generated_maps/mapa_{i + 1}.png"
    mapa.save(png_filename)


    pgm_filename = f"generated_maps/mapa_{i + 1}.pgm"
    mapa_gray = mapa.convert("L")
    mapa_gray.save(pgm_filename)

print("Mapas de obstaculos gerados com sucesso.")
