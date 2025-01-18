import cv2
import numpy as np

"""
def blur_frame(frame, kernel_size=(63, 33)):
    
   # Aplica un desenfoque a un fotograma.
    
    return cv2.GaussianBlur(frame, kernel_size, 0)
"""
def adjust_blue_tone(frame, blue_decrease=23, blue_boost=30):
    """
    Reduce el azul inicialmente y luego realza ligeramente su predominancia.
    """
    b, g, r = cv2.split(frame)
    b = cv2.subtract(b, np.full(b.shape, blue_decrease, dtype=np.uint8))
    b = cv2.add(b, np.full(b.shape, blue_boost, dtype=np.uint8))
    return cv2.merge((b, g, r))

def add_uv_effect(frame, purple_boost=55, brightness_increase=37):
    """
    Aplica un efecto UV a un fotograma.
    """
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv_frame)
    s = cv2.add(s, np.full(s.shape, purple_boost, dtype=np.uint8))
    v = cv2.add(v, np.full(v.shape, brightness_increase, dtype=np.uint8))
    modified_hsv = cv2.merge((h, s, v))
    uv_effect_frame = cv2.cvtColor(modified_hsv, cv2.COLOR_HSV2BGR)
    b, g, r = cv2.split(uv_effect_frame)
    b = cv2.add(b, np.full(b.shape, 30, dtype=np.uint8))
    r = cv2.add(r, np.full(r.shape, 20, dtype=np.uint8))
    g = cv2.subtract(g, np.full(g.shape, 10, dtype=np.uint8))
    return cv2.merge((b, g, r))

def detect_and_track_movement(frame, fgbg, kernel):
    """
    Detecta movimiento en el fotograma utilizando el sustractor de fondo.
    """
    fgmask = fgbg.apply(frame)
    fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) < 500:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    return frame

def apply_all_effects_to_camera():

#   Captura la cámara en tiempo real y aplica los efectos: desenfoque, ajuste de azul, efecto UV y detección de movimiento.
 
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise ValueError("No se pudo acceder a la cámara.")

        fgbg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=30, detectShadows=True)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        print("Presiona 'q' para salir.")

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error al capturar el fotograma. Saliendo...")
                break

            # Aplica los efectos en orden
 #           blurred_frame = blur_frame(frame)
            adjusted_frame = adjust_blue_tone(frame)
            uv_effect_frame = add_uv_effect(adjusted_frame)
            final_frame = detect_and_track_movement(uv_effect_frame, fgbg, kernel)

            cv2.imshow('Efectos Combinados con Detección de Movimiento', final_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"Error al procesar la cámara: {e}")

# Ejecutar la función para aplicar todos los efectos en tiempo real
apply_all_effects_to_camera()
