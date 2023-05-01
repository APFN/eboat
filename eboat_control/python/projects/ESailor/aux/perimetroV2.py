import math
from shapely.geometry import Polygon, Point, LineString, MultiLineString
import matplotlib.pyplot as plt
import numpy as np

# Ler as coordenadas do perímetro do polígono em um arquivo
# Aqui vamos criar um exemplo de um polígono com 4 pontos:
                   
coordinates = [[601, 181],
               [313, 249],
               [-183, 207],
               [-331, -211],
               [145, -213],
               [345, 69],
               [551, -27],
               [697, -81],
               [727, 147]]
                       
# Criar um objeto de polígono a partir dessas coordenadas
polygon = Polygon(coordinates)

# Definir a largura de cada linha de varredura
swath_width = 5

# Gerar as linhas de varredura
swaths = []
y = polygon.bounds[1] + swath_width / 2
while y < polygon.bounds[3]:
    line = LineString([(polygon.bounds[0], y), (polygon.bounds[2], y)])
    if polygon.intersects(line):
        intersection = line.intersection(polygon)
        if isinstance(intersection, MultiLineString):
            for line in intersection:
                swaths.append(line)
        else:
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


# Imprimir o caminho em um gráfico
fig, ax = plt.subplots()
ax.plot([p[0] for p in path], [p[1] for p in path])
ax.plot([p[0] for p in coordinates], [p[1] for p in coordinates], 'bo')
ax.set_title('Caminho de CPP')
ax.set_xlabel('Coordenada X')
ax.set_ylabel('Coordenada Y')
ax.set_xlim([polygon.bounds[0]*1.1, polygon.bounds[2]*1.1])
ax.set_ylim([polygon.bounds[1]*1.1, polygon.bounds[3]*1.1])


plt.show()

