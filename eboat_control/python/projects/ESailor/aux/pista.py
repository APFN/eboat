import matplotlib.pyplot as plt

# coordenadas dos pontos que formam a pista de Imola (x, y)
pista = [(119.9676, 156.67739999999998), (170.1539, 228.8202), (215.6352, 271.1649), (247.0016, 310.3729), (250.1383, 369.9692), (279.9364, 445.2486), (305.0295, 529.9379000000001), (328.5543, 584.8292000000001), (345.8059, 627.1739), (395.9922, 645.9937), (433.6319, 683.6334), (507.343, 732.2514000000001), (570.0758, 751.0712000000001), (698.6782, 755.7762000000001), (847.6687, 721.2731), (1028.0257, 714.9999), (1150.3547, 733.8197), (1278.9571, 774.5961), (1322.8701, 785.5743), (1340.1216, 754.2079), (1355.8048, 705.589), (1263.2739, 661.6769000000001), (1187.9944, 600.5124), (1068.802, 501.7081), (960.5878, 453.0902), (938.6313, 462.5001), (919.8115, 482.8883), (890.0134, 473.47840000000003), (819.4389, 471.91), (715.9297, 468.7734), (632.8087, 467.2051), (595.169, 484.4566), (568.5075, 464.06840000000003), (548.1193, 426.42870000000004), (559.0976, 373.1058), (563.8025, 285.2798), (560.6659, 217.842), (529.2995, 159.8141), (480.6815, 150.4042), (392.8555, 166.0874), (289.3463, 170.79230000000002), (201.5203, 153.5408)]



# converter as coordenadas para a escala 1:100
pista_scaled = [(x*100, y*100) for x, y in pista]

# criar a matriz com os pontos de 100 em 100 metros
x = [p[0] for p in pista_scaled]
y = [p[1] for p in pista_scaled]
plt.scatter(x, y)

# definir os limites do gráfico para mostrar apenas a pista
plt.xlim(min(x)-1000, max(x)+1000)
plt.ylim(min(y)-1000, max(y)+1000)

plt.show()

