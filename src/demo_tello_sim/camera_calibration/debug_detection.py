#!/usr/bin/env python3
"""
Diagnóstico: prueba detección de marcadores ArUco con distintos diccionarios
en una imagen de calibración y muestra el resultado.

Uso:
  python3 camera_calibration/debug_detection.py
  python3 camera_calibration/debug_detection.py 25   # probar imagen índice 25
"""
import os, sys
import cv2
import numpy as np

IMAGES_DIR = os.path.join(os.path.dirname(__file__), "images")

DICTS_TO_TRY = [
    ("DICT_4X4_50",    cv2.aruco.DICT_4X4_50),
    ("DICT_4X4_100",   cv2.aruco.DICT_4X4_100),
    ("DICT_4X4_250",   cv2.aruco.DICT_4X4_250),
    ("DICT_5X5_50",    cv2.aruco.DICT_5X5_50),
    ("DICT_5X5_100",   cv2.aruco.DICT_5X5_100),
    ("DICT_6X6_50",    cv2.aruco.DICT_6X6_50),
    ("DICT_6X6_250",   cv2.aruco.DICT_6X6_250),
    ("DICT_ARUCO_ORIGINAL", cv2.aruco.DICT_ARUCO_ORIGINAL),
]

COLS, ROWS = 9, 6
SQUARE_SIZE, MARKER_SIZE = 0.025, 0.018


def try_detect(gray, dict_id, dict_name):
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    params = cv2.aruco.DetectorParameters()
    # Relajar parámetros para mejorar detección en imágenes difíciles
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate = 0.02
    params.maxMarkerPerimeterRate = 4.0
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    corners, ids, rejected = detector.detectMarkers(gray)
    n = len(ids) if ids is not None else 0
    return corners, ids, n


def main():
    images = sorted([f for f in os.listdir(IMAGES_DIR) if f.endswith(".png")])
    if not images:
        print("No hay imágenes en images/")
        return

    idx = int(sys.argv[1]) if len(sys.argv) > 1 else len(images) // 2
    idx = min(idx, len(images) - 1)
    path = os.path.join(IMAGES_DIR, images[idx])

    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print(f"Imagen: {images[idx]}  resolución: {img.shape[1]}×{img.shape[0]}")
    print(f"Brillo medio: {gray.mean():.1f}  std: {gray.std():.1f}\n")

    best_name, best_n, best_corners, best_ids = None, 0, None, None

    print(f"{'Diccionario':<25} {'Marcadores detectados':>22}")
    print("-" * 50)
    for name, dict_id in DICTS_TO_TRY:
        corners, ids, n = try_detect(gray, dict_id, name)
        marker = " <-- MEJOR" if n > best_n else ""
        print(f"  {name:<23} {n:>5}{marker}")
        if n > best_n:
            best_n, best_name = n, name
            best_corners, best_ids = corners, ids

    print(f"\nMejor diccionario: {best_name}  ({best_n} marcadores)")

    if best_n == 0:
        print("\nNINGÚN marcador detectado en ningún diccionario.")
        print("Posibles causas:")
        print("  - Imagen borrosa o con poca luz")
        print("  - El tablero está demasiado lejos o pequeño en el frame")
        print("  - El tablero no es ChArUco (podría ser un tablero de ajedrez estándar)")
        print("\nMostrando imagen original para inspección visual...")
        cv2.imshow("Imagen de calibración (sin detección)", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    # Mostrar con mejor diccionario y también intentar ChArUco
    display = img.copy()
    cv2.aruco.drawDetectedMarkers(display, best_corners, best_ids)

    print(f"\nIDs detectados: {best_ids.ravel().tolist() if best_ids is not None else []}")

    # Intentar ChArUco con el mejor diccionario
    dictionary = cv2.aruco.getPredefinedDictionary(
        next(d for n, d in DICTS_TO_TRY if n == best_name)
    )
    board = cv2.aruco.CharucoBoard((COLS, ROWS), SQUARE_SIZE, MARKER_SIZE, dictionary)
    charuco_det = cv2.aruco.CharucoDetector(board)
    ch_corners, ch_ids, _, _ = charuco_det.detectBoard(gray)
    n_ch = len(ch_ids) if ch_ids is not None else 0
    print(f"Esquinas ChArUco (6×9, {best_name}): {n_ch}")

    if ch_ids is not None and n_ch > 0:
        cv2.aruco.drawDetectedCornersCharuco(display, ch_corners, ch_ids)
        print("  -> Los parámetros del tablero son CORRECTOS")
    else:
        print("  -> Marcadores ArUco OK pero el tablero ChArUco no coincide.")
        print("     Verifica COLS/ROWS del tablero impreso.")

    cv2.imshow(f"Detección con {best_name} | {n_ch} esquinas ChArUco", display)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
