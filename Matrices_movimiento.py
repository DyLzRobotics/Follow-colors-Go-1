# Ejemplo de rotaciones sucesivas en Python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Punto inicial en 3D
P = np.array([1, 0, 0])  # Coordenadas (x, y, z)

# Ángulos de rotación (en grados)
anguloY = 90  # Rotación alrededor del eje Y
anguloZ = 90  # Rotación alrededor del eje Z

# Convertir los ángulos a radianes
anguloY_rad = np.deg2rad(anguloY)
anguloZ_rad = np.deg2rad(anguloZ)

# Matriz de rotación alrededor del eje Y
Ry = np.array([
    [np.cos(anguloY_rad),  0, np.sin(anguloY_rad)],
    [0,                   1, 0],
    [-np.sin(anguloY_rad), 0, np.cos(anguloY_rad)]
])

# Matriz de rotación alrededor del eje Z
Rz = np.array([
    [np.cos(anguloZ_rad), -np.sin(anguloZ_rad), 0],
    [np.sin(anguloZ_rad),  np.cos(anguloZ_rad), 0],
    [0,                   0,                   1]
])

# Rotación sucesiva: primero alrededor de Y, luego de Z
P1 = Ry @ P  # Rotar P alrededor del eje Y
P2 = Rz @ P1  # Rotar el resultado alrededor del eje Z

# Graficar los puntos
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Dibujar el sistema de referencia inicial
ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='Eje X')  # Eje X
ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Eje Y')  # Eje Y
ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Eje Z')  # Eje Z

# Dibujar el punto original
ax.scatter(P[0], P[1], P[2], color='k', s=50, label='Punto original')

# Dibujar el punto después de la primera rotación
ax.scatter(P1[0], P1[1], P1[2], color='r', s=50, label='Después de rotar Y')

# Dibujar el punto después de la segunda rotación
ax.scatter(P2[0], P2[1], P2[2], color='b', s=50, label='Después de rotar Z')

# Configurar la gráfica
ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Rotaciones sucesivas en 3D')
ax.grid(True)
plt.show()
