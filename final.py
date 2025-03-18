import numpy as np
import cv2
import threading as thr
import logging
from go1pylib import Go1, Go1Mode
import time
import paho.mqtt.client as mqtt
import asyncio

# Variables globales y constantes de posición
x, y = None, None
X1, X2 = 271, 279   # Rangos en X para el objetivo
Y1, Y2 = 460, 465   # Rangos en Y para la llegada
cap = None

# Estados de la máquina de control
STATE_FOLLOW = "FOLLOW"
STATE_SEARCH = "SEARCH"

def ajuste_color(frame, blue_decrease=23, blue_boost=30):
    b, g, r = cv2.split(frame)
    b = cv2.subtract(b, np.full(b.shape, blue_decrease, dtype=np.uint8))
    b = cv2.add(b, np.full(b.shape, blue_boost, dtype=np.uint8))
    return cv2.merge((b, g, r))

def add_uv_effect(frame, purple_boost=55, brightness_increase=37):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv_frame)
    s = cv2.add(s, np.full(s.shape, purple_boost, dtype=np.uint8))
    v = cv2.add(v, np.full(v.shape, brightness_increase, dtype=np.uint8))
    modified_hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(modified_hsv, cv2.COLOR_HSV2BGR)

def follow_color(frame):
    GREEN_LOW = np.array([40, 100, 100], np.uint8)
    GREEN_HIGH = np.array([100, 255, 255], np.uint8)

    # Convierte el frame a espacio de color HSV y genera la máscara para el verde
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_green = cv2.inRange(frame_hsv, GREEN_LOW, GREEN_HIGH)
    contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    x_center, y_center = None, None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 500:
            continue
        M = cv2.moments(contour)
        if M["m00"] != 0:
            x_center = int(M["m10"] / M["m00"])
            y_center = int(M["m01"] / M["m00"])
            cv2.circle(frame, (x_center, y_center), 7, (0, 255, 0), -1)
            cv2.putText(frame, f"({x_center}, {y_center})", (x_center + 10, y_center - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return x_center, y_center

def process_video():
    global cap, x, y
    while True:
        if cap is None or not cap.isOpened():
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("Error: No se pudo abrir la cámara.")
                time.sleep(2)
                continue  # Intentar nuevamente

        ret, frame = cap.read()
        if not ret:
            print("Error al leer la cámara, intentando reiniciar...")
            cap.release()
            cap = None
            time.sleep(2)
            continue  # Reintentar abrir la cámara

        frame = ajuste_color(frame)
        frame = add_uv_effect(frame)
        x, y = follow_color(frame)

        cv2.imshow("Video Procesado", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Configuración del logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def connect_to_robot():
    try:
        robot = Go1()
        robot.init()
        return robot
    except Exception as e:
        logger.error(f"Error al conectar con el robot: {e}")
        return None

mqtt_broker = "mqtt.eclipse.org"
mqtt_port = 1883

def on_connect(client, userdata, flags, rc):
    logger.info(f"Conectado a MQTT Broker con código {rc}")
    client.subscribe("robot/control")

def on_message(client, userdata, msg):
    logger.info(f"Mensaje recibido: {msg.payload.decode()}")

def mqtt_connect():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect(mqtt_broker, mqtt_port, 60)
        logger.info("Conexión MQTT exitosa.")
        client.loop_start()
    except Exception as e:
        logger.error(f"Error al conectar con MQTT: {e}")

# Funciones de movimiento del robot
async def izquierda(robot):
    if robot:
        try:
            await robot.turn_left(speed=0.5, duration_ms=1000)
        except Exception as e:
            logger.error(f"Error en el movimiento izquierda: {e}")

async def derecha(robot):
    if robot:
        try:
            await robot.turn_right(speed=0.5, duration_ms=1000)
        except Exception as e:
            logger.error(f"Error en el movimiento derecha: {e}")

async def adelante(robot):
    if robot:
        try:
            await robot.go_forward(speed=0.5, duration_ms=3000)
        except Exception as e:
            logger.error(f"Error en el movimiento adelante: {e}")

async def postura(robot):
    if robot:
        try:
            # Se usa RECOVER_STAND para regresar a la postura normal
            await robot.set_mode(Go1Mode.RECOVER_STAND)
        except Exception as e:
            logger.error(f"Error al cambiar a postura normal: {e}")

async def main():
    robot = connect_to_robot()
    if robot is None:
        print("No se pudo conectar con el robot.")
        return

    mqtt_connect()

    try:
        battery_level = robot.get_battery_level()
        print(f"Nivel de batería: {battery_level}%")
    except Exception as e:
        logger.error(f"Error al obtener el nivel de batería: {e}")

    # Poner al robot en modo STAND inicialmente
    robot.set_mode(Go1Mode.STAND)
    await asyncio.sleep(1)

    # Estado inicial: SEARCH (sin objetivo)
    current_state = STATE_SEARCH

    # Bucle de control del robot
    while True:
        global x, y, X1, X2, Y1, Y2

        # Si no se detecta el objetivo en la imagen:
        if x is None or y is None:
            if current_state != STATE_SEARCH:
                print("Objetivo perdido, volviendo a postura normal")
                await postura(robot)
                current_state = STATE_SEARCH
            else:
                print("Buscando objetivo...")
        else:
            # Si se detecta el objetivo, cambiar al estado de seguimiento (FOLLOW)
            current_state = STATE_FOLLOW
            if x < X1:
                print("Rotar derecha")
                await derecha(robot)
            elif x > X2:
                print("Rotar izquierda")
                await izquierda(robot)
            elif X1 <= x <= X2:
                print("Adelante")
                await adelante(robot)
                if Y1 <= y <= Y2:
                    print("Llego")
                    break
        await asyncio.sleep(0.5)

# Ejecutar el procesamiento de video y el control del robot de manera concurrente
async def run():
    # Crear hilo para procesar el video
    video_thread = thr.Thread(target=process_video, daemon=True)
    video_thread.start()
    # Ejecutar el control del robot en el hilo principal
    await main()

if __name__ == "__main__":
    asyncio.run(run())

    
