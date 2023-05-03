import math
from shapely.geometry import Polygon, Point, LineString
import matplotlib.pyplot as plt
import numpy as np

# Ler as coordenadas do perímetro do polígono em um arquivo
# Aqui vamos criar um exemplo de um polígono com 4 pontos:

                   
                   
coordinates = [[-800, 10], 
[0, 800], 
[800, 0], 
[0, -800]]


                   
                   


# Criar um objeto de polígono a partir dessas coordenadas
polygon = Polygon(coordinates)

# Definir a largura de cada linha de varredura
swath_width = 100

# Gerar as linhas de varredura
swaths = []
y = polygon.bounds[1] + swath_width / 2
while y < polygon.bounds[3]:
    line = LineString([(polygon.bounds[0], y), (polygon.bounds[2], y)])
    if polygon.intersects(line):
        intersection = line.intersection(polygon)
        swaths.append(intersection)
    y += swath_width

# Gerar os caminhos de zigzag para cada linha de varredura
paths = []
for i, swath in enumerate(swaths):
    if i % 2 == 0:
        path = [(swath.coords[0][0], swath.coords[0][1])]
        for j in range(1, int(swath.length / swath_width) + 1):
            if j % 2 == 1:
                x = swath.coords[0][0] + j * swath_width
            else:
                x = swath.coords[0][0] + (j + 1) * swath_width
            y = swath.coords[0][1]
            if x <= swath.coords[1][0]:
                path.append((x, y))
        path[-1] = (swath.coords[1][0], swath.coords[1][1])
        paths.append(path)
    else:
        path = [(swath.coords[1][0], swath.coords[1][1])]
        for j in range(1, int(swath.length / swath_width) + 1):
            if j % 2 == 1:
                x = swath.coords[1][0] - j * swath_width
            else:
                x = swath.coords[1][0] - (j + 1) * swath_width
            y = swath.coords[1][1]
            if x >= swath.coords[0][0]:
                path.append((x, y))
        path[-1] = (swath.coords[0][0], swath.coords[0][1])
        paths.append(path)

# Concatenar os caminhos de zigzag para gerar o caminho completo
path = []
for p in paths:
    path += p[::-1] if len(path) % 2 == 0 else p
    
# Converter o caminho em uma matriz de duas dimensões com pontos de 100m no máximo entre cada ponto
new_path = [path[0]]
for i in range(1, len(path)):
    p1 = path[i - 1]
    p2 = path[i]
    dist = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
    if dist <= 100:
        new_path.append(p2)
    else:
        n_points = int(dist / 100) + 1
        for j in range(1, n_points):
            x = p1[0] + (p2[0] - p1[0]) * j / n_points
            y = p1[1] + (p2[1] - p1[1]) * j / n_points
            new_path.append((x, y))
            print(x, ",", y)
        new_path.append(p2)

# Imprimir o caminho em um gráfico
fig, ax = plt.subplots()
ax.plot([p[0] for p in path], [p[1] for p in path])
ax.plot([p[0] for p in coordinates], [p[1] for p in coordinates], 'bo')
ax.plot([p[0] for p in new_path], [p[1] for p in new_path], 'ro')
ax.set_title('Caminho de CPP')
ax.set_xlabel('Coordenada X')
ax.set_ylabel('Coordenada Y')
ax.set_xlim([polygon.bounds[0]*1.1, polygon.bounds[2]*1.1])
ax.set_ylim([polygon.bounds[1]*1.1, polygon.bounds[3]*1.1])


plt.show()

