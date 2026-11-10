
import cv2
import numpy as np
import serial
import time

# CONEXIONES
URL = "http://172.18.4.249:4747/video"
PUERTO = "COM4"

# DETECCION DE PELOTA
AREA_MINIMA = 80
AREA_MAXIMA = 2500

arduino = serial.Serial(PUERTO, 9600, timeout=1)
time.sleep(2)

camara = cv2.VideoCapture(URL)

if not camara.isOpened():
    arduino.close()
    raise RuntimeError("No se pudo conectar con la camara")

print("Camara y Arduino conectados")

# Seleccionar campo
ret, imagen = camara.read()

if not ret:
    camara.release()
    arduino.close()
    raise RuntimeError("No se pudo leer la imagen")

roi = cv2.selectROI("Seleccionar campo", imagen, False)
cv2.destroyWindow("Seleccionar campo")

x_roi, y_roi, ancho_roi, alto_roi = map(int, roi)

if ancho_roi == 0 or alto_roi == 0:
    camara.release()
    arduino.close()
    raise RuntimeError("No se selecciono el campo")

ultimo_envio = 0

try:
    while True:
        ret, imagen = camara.read()

        if not ret:
            print("No se recibio imagen")
            break

        campo = imagen[
            y_roi:y_roi + alto_roi,
            x_roi:x_roi + ancho_roi
        ].copy()

        hsv = cv2.cvtColor(campo, cv2.COLOR_BGR2HSV)

        rojo1 = cv2.inRange(
            hsv,
            np.array([0, 100, 80]),
            np.array([12, 255, 255])
        )

        rojo2 = cv2.inRange(
            hsv,
            np.array([170, 100, 80]),
            np.array([179, 255, 255])
        )

        mascara = cv2.bitwise_or(rojo1, rojo2)

        kernel = np.ones((3, 3), np.uint8)

        mascara = cv2.morphologyEx(
            mascara, cv2.MORPH_OPEN, kernel
        )

        mascara = cv2.morphologyEx(
            mascara, cv2.MORPH_CLOSE, kernel
        )

        contornos, _ = cv2.findContours(
            mascara,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        pelota_x = None

        for contorno in sorted(
            contornos,
            key=cv2.contourArea,
            reverse=True
        ):
            area = cv2.contourArea(contorno)

            if not AREA_MINIMA <= area <= AREA_MAXIMA:
                continue

            perimetro = cv2.arcLength(contorno, True)

            if perimetro == 0:
                continue

            circularidad = (
                4 * np.pi * area / (perimetro ** 2)
            )

            if circularidad < 0.65:
                continue

            (x, y), radio = cv2.minEnclosingCircle(contorno)

            pelota_x = int(x)

            cv2.circle(
                campo,
                (int(x), int(y)),
                int(radio),
                (0, 255, 0),
                2
            )

            break

        if pelota_x is not None:
            ahora = time.monotonic()

            # Enviar como maximo 10 veces por segundo
            if ahora - ultimo_envio >= 0.1:
                mensaje = f"X:{pelota_x},{ancho_roi}\n"
                arduino.write(mensaje.encode("utf-8"))

                print("Pelota enviada:", pelota_x)

                while arduino.in_waiting > 0:
                    respuesta = arduino.readline().decode(
                    "utf-8", errors="replace"
                    ).strip()

                if respuesta:
                    print("Arduino:", respuesta)
                
                ultimo_envio = ahora

        cv2.imshow("Camara + Arduino", campo)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    camara.release()
    arduino.close()
    cv2.destroyAllWindows()

print("Programa finalizado")
