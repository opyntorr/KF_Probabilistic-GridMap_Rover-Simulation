#!/usr/bin/env python3
"""
Captura imágenes de calibración desde el DJI Tello.

ANTES DE EJECUTAR:
  - Conecta el PC al WiFi del Tello (TELLO-XXXXXX)
  - Mantén el dron en el PISO (no necesita despegar)
  - Hay un espejo frente a la cámara — el frame se voltea automáticamente

CÓMO CAPTURAR BIEN:
  - Espera a que el indicador sea VERDE (ChArUco corners detectadas)
  - Mueve el tablero a distintas posiciones y ángulos entre capturas
  - Cubre esquinas del frame, no solo el centro

Controles:
  SPACE  - guardar (solo si indicador VERDE)
  D      - toggle detección en vivo
  Q      - salir
"""
import os

os.environ.setdefault("QT_LOGGING_RULES", "qt.qpa.fonts=false")
os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")

import cv2
import numpy as np
from djitellopy import Tello

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "images")
os.makedirs(OUTPUT_DIR, exist_ok=True)

COLS = 9
ROWS = 6
SQUARE_SIZE = 0.025
MARKER_SIZE = 0.018
DICT_ID = cv2.aruco.DICT_4X4_50

DETECTION_EVERY_N = 5
MIN_CHARUCO = 6    # esquinas ChArUco mínimas para habilitar guardado
WINDOW = "Tello - Captura Calibración"


def interpolate_charuco(m_corners, m_ids, gray, board):
    """
    Interpolate ChArUco corners via global homography.
    Replacement for the removed cv2.aruco.interpolateCornersCharuco.
    Uses a single homography over all detected markers (robust to flat boards).
    """
    ids_flat = m_ids.ravel()
    marker_obj_pts = board.getObjPoints()   # list[N_markers] of (4,3) in board coords

    src_pts, dst_pts = [], []
    for i, mid in enumerate(ids_flat):
        if mid < len(marker_obj_pts):
            src_pts.append(marker_obj_pts[mid][:, :2])
            dst_pts.append(m_corners[i].reshape(4, 2))

    if len(src_pts) < 1:
        return 0, None, None

    src = np.vstack(src_pts).astype(np.float32)
    dst = np.vstack(dst_pts).astype(np.float32)

    if len(src) < 4:
        return 0, None, None

    H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    if H is None:
        return 0, None, None

    ch_board = board.getChessboardCorners()[:, :2].reshape(-1, 1, 2).astype(np.float32)
    ch_img = cv2.perspectiveTransform(ch_board, H).reshape(-1, 2)

    h, w = gray.shape[:2]
    in_bounds = ((ch_img[:, 0] >= 0) & (ch_img[:, 0] < w) &
                 (ch_img[:, 1] >= 0) & (ch_img[:, 1] < h))
    valid = np.where(in_bounds)[0]

    if len(valid) == 0:
        return 0, None, None

    pts = ch_img[valid].astype(np.float32).reshape(-1, 1, 2)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    pts = cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), criteria)

    return len(valid), pts, valid.reshape(-1, 1).astype(np.int32)


def sharpen(gray):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    blurred = cv2.GaussianBlur(enhanced, (0, 0), sigmaX=2)
    return cv2.addWeighted(enhanced, 1.8, blurred, -0.8, 0)


def build_detectors():
    dictionary = cv2.aruco.getPredefinedDictionary(DICT_ID)
    p = cv2.aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 53
    p.adaptiveThreshWinSizeStep = 4
    p.minMarkerPerimeterRate = 0.02
    p.maxMarkerPerimeterRate = 4.0
    p.polygonalApproxAccuracyRate = 0.08
    p.minOtsuStdDev = 3.0
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    p.cornerRefinementMaxIterations = 30
    p.cornerRefinementMinAccuracy = 0.1

    board = cv2.aruco.CharucoBoard((COLS, ROWS), SQUARE_SIZE, MARKER_SIZE, dictionary)

    aruco_det = cv2.aruco.ArucoDetector(dictionary, p)

    return aruco_det, board


def main():
    tello = Tello()
    tello.connect()
    print(f"Batería: {tello.get_battery()}%")

    tello.streamon()
    frame_read = tello.get_frame_read()

    count = len([f for f in os.listdir(OUTPUT_DIR) if f.endswith(".png")])
    print(f"Imágenes existentes: {count}")
    print(f"Espera a que ChArUco >= {MIN_CHARUCO} (indicador VERDE) antes de guardar")
    print("SPACE=guardar  D=toggle detección  Q=salir")

    aruco_det, board = build_detectors()
    show_detection = True
    n_markers = 0
    n_charuco = 0
    frame_idx = 0

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

    while True:
        frame = frame_read.frame
        if frame is None:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        frame = cv2.flip(frame, 1)
        display = frame.copy()

        if show_detection and frame_idx % DETECTION_EVERY_N == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sharp = sharpen(gray)

            m_corners, m_ids, _ = aruco_det.detectMarkers(sharp)
            n_markers = len(m_ids) if m_ids is not None else 0

            if m_corners and m_ids is not None:
                n_charuco, ch_corners, ch_ids = interpolate_charuco(
                    m_corners, m_ids, gray, board)
            else:
                n_charuco, ch_corners, ch_ids = 0, None, None

            if m_corners:
                cv2.aruco.drawDetectedMarkers(display, m_corners, m_ids)
            if ch_ids is not None and n_charuco > 0:
                cv2.aruco.drawDetectedCornersCharuco(display, ch_corners, ch_ids)

        if show_detection:
            if n_charuco >= MIN_CHARUCO:
                color = (0, 220, 0)
                status = "LISTO"
            elif n_markers > 0:
                color = (0, 200, 220)
                status = f"ajusta posicion (ChArUco={n_charuco})"
            else:
                color = (0, 60, 220)
                status = "tablero no detectado"

            cv2.putText(display,
                        f"ArUco: {n_markers}  ChArUco: {n_charuco}  {status}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        can_save = n_charuco >= MIN_CHARUCO
        hint = "SPACE=guardar" if can_save else f"ChArUco={n_charuco}<{MIN_CHARUCO}"
        cv2.putText(display, f"Guardadas: {count} | {hint} | D=det Q=salir",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)

        cv2.imshow(WINDOW, display)
        frame_idx += 1

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('d'):
            show_detection = not show_detection
            n_markers = 0
            n_charuco = 0
        elif key == ord(' '):
            if can_save:
                path = os.path.join(OUTPUT_DIR, f"calib_{count:04d}.png")
                cv2.imwrite(path, frame)
                count += 1
                print(f"  Guardada: {os.path.basename(path)}  "
                      f"(ArUco={n_markers}, ChArUco={n_charuco})")
            else:
                print(f"  IGNORADA: ChArUco={n_charuco} < {MIN_CHARUCO} — ajusta posición del tablero")

    tello.streamoff()
    tello.end()
    cv2.destroyAllWindows()
    print(f"\nTotal imágenes capturadas: {count}")


if __name__ == "__main__":
    main()
