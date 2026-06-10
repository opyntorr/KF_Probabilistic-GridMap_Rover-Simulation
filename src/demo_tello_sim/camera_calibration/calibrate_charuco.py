#!/usr/bin/env python3
"""
Calibración de cámara del DJI Tello con tablero ChArUco 9x6.
Genera la matriz intrínseca K y el vector de distorsión D.

Board: 9 cols × 6 rows | 25 mm checker | 18 mm marker | DICT_4X4_50

Salidas en calibration_output/:
  tello_calibration.npz   — arrays numpy (K, D, image_size, rms)
  tello_calibration.yaml  — formato compatible con ROS camera_info
"""
import cv2
import numpy as np
import os
import glob
import yaml

IMAGES_DIR = os.path.join(os.path.dirname(__file__), "images")
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "calibration_output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

COLS = 9
ROWS = 6
SQUARE_SIZE = 0.025   # metros
MARKER_SIZE = 0.018   # metros
DICT_ID = cv2.aruco.DICT_4X4_50

MIN_VALID_IMAGES = 10
MIN_CORNERS_PER_IMAGE = 6


def sharpen(gray):
    """CLAHE + unsharp mask para recuperar bordes suavizados por H264 y el lente del Tello."""
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    blurred = cv2.GaussianBlur(enhanced, (0, 0), sigmaX=2)
    return cv2.addWeighted(enhanced, 1.8, blurred, -0.8, 0)


def make_aruco_detector():
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
    dictionary = cv2.aruco.getPredefinedDictionary(DICT_ID)
    return cv2.aruco.ArucoDetector(dictionary, p)


def interpolate_charuco(m_corners, m_ids, gray, board):
    """Global-homography ChArUco interpolation. Avoids the per-marker local
    homography bug in CharucoDetector that returns 0 corners on flat boards."""
    ids_flat = m_ids.ravel()
    marker_obj_pts = board.getObjPoints()

    src_pts, dst_pts = [], []
    for i, mid in enumerate(ids_flat):
        if mid < len(marker_obj_pts):
            src_pts.append(marker_obj_pts[mid][:, :2])
            dst_pts.append(m_corners[i].reshape(4, 2))

    if len(src_pts) < 1:
        return None, None

    src = np.vstack(src_pts).astype(np.float32)
    dst = np.vstack(dst_pts).astype(np.float32)

    if len(src) < 4:
        return None, None

    H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    if H is None:
        return None, None

    ch_board = board.getChessboardCorners()[:, :2].reshape(-1, 1, 2).astype(np.float32)
    ch_img = cv2.perspectiveTransform(ch_board, H).reshape(-1, 2)

    h, w = gray.shape[:2]
    in_bounds = ((ch_img[:, 0] >= 0) & (ch_img[:, 0] < w) &
                 (ch_img[:, 1] >= 0) & (ch_img[:, 1] < h))
    valid = np.where(in_bounds)[0]

    if len(valid) == 0:
        return None, None

    pts = ch_img[valid].astype(np.float32).reshape(-1, 1, 2)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    pts = cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), criteria)

    return pts, valid.reshape(-1, 1).astype(np.int32)


def detect_charuco(image_paths):
    dictionary = cv2.aruco.getPredefinedDictionary(DICT_ID)
    board = cv2.aruco.CharucoBoard((COLS, ROWS), SQUARE_SIZE, MARKER_SIZE, dictionary)
    aruco_det = make_aruco_detector()

    all_corners = []
    all_ids = []
    image_size = None
    skipped = []

    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            print(f"  ERROR  No se pudo leer: {path}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = gray.shape[::-1]  # (width, height)

        sharp = sharpen(gray)
        m_corners, m_ids, _ = aruco_det.detectMarkers(sharp)

        charuco_corners, charuco_ids = None, None
        if m_corners and m_ids is not None:
            charuco_corners, charuco_ids = interpolate_charuco(m_corners, m_ids, gray, board)

        if charuco_ids is not None and len(charuco_ids) >= MIN_CORNERS_PER_IMAGE:
            all_corners.append(charuco_corners)
            all_ids.append(charuco_ids)
            print(f"  OK   {os.path.basename(path):25s}  {len(charuco_ids):2d} esquinas")
        else:
            n = len(charuco_ids) if charuco_ids is not None else 0
            skipped.append(os.path.basename(path))
            print(f"  SKIP {os.path.basename(path):25s}  {n:2d} esquinas (< {MIN_CORNERS_PER_IMAGE})")

    return all_corners, all_ids, image_size, board, skipped


def run_calibration(all_corners, all_ids, board, image_size):
    obj_pts_all = []
    img_pts_all = []
    for corners, ids in zip(all_corners, all_ids):
        obj_pts, img_pts = board.matchImagePoints(corners, ids)
        if obj_pts is not None and len(obj_pts) >= 4:
            obj_pts_all.append(obj_pts)
            img_pts_all.append(img_pts)

    # CORRECCIÓN: Se utiliza flags=0 para forzar el modelo clásico de 5 parámetros.
    # Esto evita el sobreajuste matemático en las esquinas, permitiendo que 
    # las imágenes se puedan usar correctamente para stitching.
    flags = 0
    
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts_all, img_pts_all, image_size, None, None, flags=flags
    )
    return rms, K, D, rvecs, tvecs


def save_results(K, D, image_size, rms):
    npz_path = os.path.join(OUTPUT_DIR, "tello_calibration.npz")
    np.savez(npz_path, K=K, D=D, image_size=np.array(image_size), rms=np.float64(rms))

    yaml_data = {
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "camera_name": "tello",
        "camera_matrix": {
            "rows": 3, "cols": 3,
            "data": K.flatten().tolist(),
        },
        # CORRECCIÓN: 'plumb_bob' es el nombre estándar para el modelo de 5 parámetros
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1, "cols": int(D.size),
            "data": D.ravel().tolist(),
        },
        "rms_reprojection_error_px": float(rms),
    }
    yaml_path = os.path.join(OUTPUT_DIR, "tello_calibration.yaml")
    with open(yaml_path, "w") as f:
        yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)

    return npz_path, yaml_path


def main():
    image_paths = sorted(glob.glob(os.path.join(IMAGES_DIR, "*.png")))
    print(f"Imágenes encontradas: {len(image_paths)}\n")

    if len(image_paths) < MIN_VALID_IMAGES:
        print(f"ERROR: se necesitan al menos {MIN_VALID_IMAGES} imágenes. "
              f"Captura más con capture_images.py")
        return

    all_corners, all_ids, image_size, board, skipped = detect_charuco(image_paths)

    print(f"\nImágenes válidas: {len(all_corners)} / {len(image_paths)}")
    if skipped:
        print(f"Descartadas ({len(skipped)}): {', '.join(skipped)}")

    if len(all_corners) < MIN_VALID_IMAGES:
        print(f"\nERROR: sólo {len(all_corners)} imágenes válidas. "
              f"Necesitas al menos {MIN_VALID_IMAGES}.")
        return

    print("\nEjecutando calibración...")
    rms, K, D, _, _ = run_calibration(all_corners, all_ids, board, image_size)

    print(f"\n{'='*50}")
    print(f"  RMS (error de reproyección): {rms:.4f} px")
    print(f"  (bueno < 1.0 px, excelente < 0.5 px)")
    print(f"{'='*50}")
    print(f"\nMatriz K (intrínseca):\n{K}")
    print(f"\nVector D (distorsión): {D.ravel()}")
    # CORRECCIÓN: Actualizado para reflejar que ahora son 5 parámetros
    print(f"  [k1, k2, p1, p2, k3]")
    print(f"\nResolución calibrada: {image_size[0]}×{image_size[1]} px")

    npz_path, yaml_path = save_results(K, D, image_size, rms)
    print(f"\nGuardado:")
    print(f"  {npz_path}")
    print(f"  {yaml_path}")

    if rms > 1.5:
        print("\nADVERTENCIA: error de reproyección alto. "
              "Considera re-capturar imágenes con mejor cobertura y sin motion blur.")


if __name__ == "__main__":
    main()