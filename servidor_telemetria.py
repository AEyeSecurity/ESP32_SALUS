#!/usr/bin/env python3
"""
Servidor de telemetría para el cuatriciclo.
Recibe datos del ESP32 via HTTP POST y los embebe en el nombre de la foto.
Formato recibido: data=stem_in_stem_out_ozh  (ej: data=48_1500_1)
Formato del archivo: YYYY-MM-DD_HH-MM-SS_stemIn-48_stemOut-1500_ozh-1.jpg
"""

from flask import Flask, request, jsonify
import cv2
import os
from datetime import datetime

app = Flask(__name__)

SAVE_DIR = "/home/admin/camara_imagenes"
os.makedirs(SAVE_DIR, exist_ok=True)

cap = None

def get_camera():
    global cap
    if cap is None or not cap.isOpened():
        # Intenta abrir la cámara (0 = primera disponible, USB o CSI con libcamera-v4l2)
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            cap = cv2.VideoCapture(1)
    return cap


@app.route("/update", methods=["POST"])
def update():
    raw = request.form.get("data", "").strip()
    if not raw:
        return jsonify({"error": "Parametro 'data' vacio o ausente"}), 400

    partes = raw.split("_")
    if len(partes) < 3:
        return jsonify({"error": f"Se esperaban 3 valores separados por '_', se recibio: '{raw}'"}), 400

    stem_in  = partes[0]
    stem_out = partes[1]
    ozh      = partes[2]

    camara = get_camera()
    if not camara.isOpened():
        return jsonify({"error": "No se pudo abrir la camara"}), 500

    ret, frame = camara.read()
    if not ret:
        return jsonify({"error": "Fallo la captura de imagen"}), 500

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    nombre_archivo = f"{timestamp}_stemIn-{stem_in}_stemOut-{stem_out}_ozh-{ozh}.jpg"
    ruta_completa = os.path.join(SAVE_DIR, nombre_archivo)

    cv2.imwrite(ruta_completa, frame)

    print(f"[OK] Imagen guardada: {nombre_archivo}")
    return jsonify({
        "status": "ok",
        "archivo": nombre_archivo,
        "stem_in": stem_in,
        "stem_out": stem_out,
        "ozh": ozh
    }), 200


@app.route("/healthz", methods=["GET"])
def healthz():
    return jsonify({"status": "running"}), 200


if __name__ == "__main__":
    print(f"[INFO] Servidor escuchando en 0.0.0.0:5000")
    print(f"[INFO] Imagenes en: {SAVE_DIR}")
    app.run(host="0.0.0.0", port=5000, debug=False)
