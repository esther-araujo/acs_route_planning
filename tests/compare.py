import os

def extrair_distancias(filename):
    with open(filename, "r") as file:
        for line in file:
            if "distance" in line:
                distancia = float(line.split(":")[1])
                return distancia
    return None

def extrair_nodesBP(filename):
    with open(filename, "r") as file:
        for line in file:
            if "nNodesBP" in line:
                nodes = float(line.split(":")[1])
                return nodes
    return None

diretorio1 = "tests/acs_logs_v1"
diretorio2 = "tests/acs_logs_jody"

arquivos_diretorio1 = os.listdir(diretorio1)
arquivos_diretorio2 = os.listdir(diretorio2)

distancias_menores = 0
distancias_maiores = 0

caminho_mais_curto_v1 = 0
caminho_mais_curto_jody = 0
caminhos_iguais = 0
menor_num_nos_v1 = 0
menor_num_nos_jody = 0
num_nos_igual = 0
media_nos_jody = 0
media_nos_v1 = 0
media_dist_jody = 0
media_dist_v1 = 0

for arquivo1 in arquivos_diretorio1:
    if arquivo1 in arquivos_diretorio2:
        caminho_arquivo1 = os.path.join(diretorio1, arquivo1)
        caminho_arquivo2 = os.path.join(diretorio2, arquivo1)
        
        v1 = extrair_distancias(caminho_arquivo1)
        jody = extrair_distancias(caminho_arquivo2)

        v1_nodes = extrair_nodesBP(caminho_arquivo1)
        jody_nodes = extrair_nodesBP(caminho_arquivo2)
        if v1_nodes is None:
            print(caminho_arquivo1)
        if jody_nodes is None:
            print(caminho_arquivo2)
        if v1 is not None and jody is not None:
            if v1 < jody:
                media_dist_v1+=v1
                caminho_mais_curto_v1 += 1
            elif v1 > jody:
                media_dist_jody +=jody
                caminho_mais_curto_jody += 1
            else:
                caminhos_iguais +=1

        if v1_nodes is not None and jody_nodes is not None:
            if v1_nodes < jody_nodes:
                media_nos_v1 +=v1_nodes
                menor_num_nos_v1 += 1
            elif v1_nodes > jody_nodes:
                media_nos_jody +=jody_nodes
                menor_num_nos_jody += 1
            else:
                num_nos_igual +=1

print(f"Mapas com Caminhos iguais: {caminhos_iguais}")
print(f"Mapas com Número de nós igual: {num_nos_igual}")
print(f"Menor número de nós BP V1: {menor_num_nos_v1}")
print(f"Menor número de nós BP Jody: {menor_num_nos_jody}")
print(f"Caminho mais curto V1: {caminho_mais_curto_v1}")
print(f"Caminho mais curto Jody: {caminho_mais_curto_jody}")
print(f"Media número de nós BP Jody: {media_nos_jody/menor_num_nos_jody}")
print(f"Media número de nós BP V1: {media_nos_v1/menor_num_nos_v1}")
print(f"Media distância BP Jody: {media_dist_jody/caminho_mais_curto_jody}")
print(f"Media distância BP V1: {media_dist_v1/caminho_mais_curto_v1}")

