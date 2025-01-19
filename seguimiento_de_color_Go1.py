import cv2
import numpy as np

# Variables de rango para detectar el color verde en formato HSV
GREEN_LOW = np.array([40, 100, 100], np.uint8)
GREEN_HIGH = np.array([100, 255, 255], np.uint8)

# Inicialización de la cámara (usa 0 para la cámara principal)
cam = cv2.VideoCapture(0)

# Configuración de resolución (opcional, ajusta según tus necesidades)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Verifica si la cámara se abre correctamente
if not cam.isOpened():
    print("Error: No se pudo acceder a la cámara.")
    exit()

print("Presiona 'q' para salir del programa.")

# Bucle de activación
while True:
    # Captura un frame
    ret, frame = cam.read()

    if not ret:
        print("Error al capturar el frame.")
        break

    # Convierte el frame a espacio de color HSV
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Crea una máscara para el color verde dentro del rango especificado
    maskGreen = cv2.inRange(frameHSV, GREEN_LOW, GREEN_HIGH)

    # Aplica la máscara para aislar el color verde en el frame original
    maskGreenView = cv2.bitwise_and(frame, frame, mask=maskGreen)

    # Encuentra los contornos en la máscara binaria
    contours, _ = cv2.findContours(maskGreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        # Ignora contornos pequeños (menos de 500 píxeles)
        if area < 500:
            continue

        # Calcula el centroide del contorno
        M = cv2.moments(contour)
        if M["m00"] != 0:
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            # Dibuja un círculo en el centro del contorno
            cv2.circle(frame, (x, y), 7, (0, 255, 0), -1)
            cv2.putText(frame, f"({x}, {y})", (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            X = 280
            Y = 465
            if X >= x:
                print("Rotar derecha")
            elif X <= x:
                print("Rotar izquierda")
            if Y != y and X == x:
                print("Adelante")

        # Dibuja el rectángulo delimitador alrededor del contorno
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Muestra las imágenes
    cv2.imshow("Frame Original", frame)
    cv2.imshow("Máscara Verde", maskGreen)
    cv2.imshow("Color Detectado", maskGreenView)

    # Finaliza si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Libera la cámara y cierra las ventanas
cam.release()
cv2.destroyAllWindows()
"""        
fgmask = fgbg.apply(frame)
fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
for contour in contours:
    if cv2.contourArea(contour) < 500:
        continue
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
return frame"""


