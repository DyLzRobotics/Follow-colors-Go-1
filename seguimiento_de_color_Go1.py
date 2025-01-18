import cv2
import numpy as np

# Variables de rango para detectar el color verde en formato HSV
GREEN_LOW = np.array([40, 100, 100], np.uint8)
GREEN_HIGH = np.array([100, 255, 255], np.uint8)

# Inicialización de la cámara (usa 0 para la cámara principal)
cam = cv2.VideoCapture(0)

# Verifica si la cámara se abre correctamente
if not cam.isOpened():
    print("Error: No se pudo acceder a la cámara.")
    exit()

# Bucle de activación
while True:
    # Captura un frame
    ret, frame = cam.read()

    # Si el frame se captura correctamente
    if ret:
        # Convierte el frame a espacio de color HSV
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Crea una máscara para el color verde dentro del rango especificado
        maskGreen = cv2.inRange(frameHSV, GREEN_LOW, GREEN_HIGH)
        # Aplica la máscara para mostrar el color verde en la imagen original
        maskGreenView = cv2.bitwise_and(frame, frame, mask=maskGreen)
        # Encuentra los contornos en la máscara binaria
        contours, _ = cv2.findContours(maskGreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            # Ignora contornos muy pequeños
            if cv2.contourArea(contour) < 500:
                M = cv2.moments(contour)
                if (M["m00"] == 0): M["m00"] = 1
                x = int(M["m10"]/M["m00"])
                y = int(M["m01"]/M["m00"])
                # Dibuja el rectángulo sobre el frame original
                cv2.circle(frame, (x, y), 7, (0, 255, 0), -1)
                continue
        # Muestra las imágenes
        cv2.imshow("Frame Original", frame)
        cv2.imshow("Máscara Verde", maskGreen)
        cv2.imshow("Color Detectado", maskGreenView)

        # Finaliza si se presiona la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        print("Error al capturar el frame.")
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