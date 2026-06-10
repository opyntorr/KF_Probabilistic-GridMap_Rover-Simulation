#!/usr/bin/env python3
"""
Stitching robusto de imagenes aereas del DJI Tello.

Pipeline manual: undistort -> SIFT -> FLANN matching -> RANSAC homografia
-> gain compensation -> multiband blending.

Uso:
  # Stitching de todas las imagenes en un directorio
  python3 stitch_images.py --input ./flight_images/

  # Solo un subconjunto (cada 3ra imagen)
  python3 stitch_images.py --input ./flight_images/ --step 3

  # Ajustar parametros de matching
  python3 stitch_images.py --input ./flight_images/ --ratio 0.65 --min-matches 15

  # Desactivar undistort (si las imagenes ya estan corregidas)
  python3 stitch_images.py --input ./flight_images/ --no-undistort

  # Modo verbose con visualizacion de matches intermedios
  python3 stitch_images.py --input ./flight_images/ --debug
"""
import os
import sys
import argparse
import glob
import time

os.environ.setdefault("QT_LOGGING_RULES", "qt.qpa.fonts=false")
os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CALIB = os.path.join(SCRIPT_DIR, "calibration_output", "tello_calibration.npz")
DEFAULT_OUTPUT_DIR = os.path.join(SCRIPT_DIR, "stitching_output")

# SIFT / matching
DEFAULT_RATIO = 0.85        # Más permisivo para simulación
DEFAULT_MIN_MATCHES = 4     # Mínimo matemático para homografía
DEFAULT_RANSAC_THRESH = 5.0 # px de reproyeccion para RANSAC


# ---------------------------------------------------------------------------
# 1. Calibracion
# ---------------------------------------------------------------------------
def load_calibration(path):
    """Carga K, D desde el .npz de calibracion."""
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"Calibracion no encontrada: {path}\n"
            "Ejecuta calibrate_charuco.py primero, o usa --no-undistort."
        )
    data = np.load(path)
    return data["K"], data["D"], tuple(data["image_size"])


def undistort(img, K, D, alpha=0.0):
    """Corrige distorsion radial/tangencial usando la calibracion."""
    h, w = img.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=alpha)
    return cv2.undistort(img, K, D, None, new_K), new_K


# ---------------------------------------------------------------------------
# 2. Deteccion de features
# ---------------------------------------------------------------------------
def detect_features(img, mask=None):
    """Detecta keypoints y descriptores SIFT. Umbral de contraste muy bajo para simulación."""
    sift = cv2.SIFT_create(nfeatures=0, contrastThreshold=0.005, edgeThreshold=20)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    kps, descs = sift.detectAndCompute(gray, mask)
    return kps, descs


# ---------------------------------------------------------------------------
# 3. Matching
# ---------------------------------------------------------------------------
def match_pair(descs_a, descs_b, ratio_thresh):
    """FLANN matching con Lowe's ratio test. Retorna lista de DMatch buenos."""
    if descs_a is None or descs_b is None:
        return []
    if len(descs_a) < 2 or len(descs_b) < 2:
        return []

    index_params = dict(algorithm=1, trees=5)   # FLANN_INDEX_KDTREE
    search_params = dict(checks=100)
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    raw = flann.knnMatch(descs_a, descs_b, k=2)
    good = []
    for pair in raw:
        if len(pair) == 2:
            m, n = pair
            if m.distance < ratio_thresh * n.distance:
                good.append(m)
    return good


def compute_homography(kps_a, kps_b, matches, ransac_thresh):
    """Calcula homografia H tal que pts_b = H @ pts_a, con filtrado RANSAC.
    Retorna (H, inlier_mask, n_inliers) o (None, None, 0) si falla."""
    if len(matches) < 4:
        return None, None, 0

    pts_a = np.float32([kps_a[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    pts_b = np.float32([kps_b[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

    H, mask = cv2.findHomography(pts_a, pts_b, cv2.RANSAC, ransac_thresh)
    if H is None:
        return None, None, 0

    n_inliers = int(mask.sum())
    return H, mask, n_inliers


# ---------------------------------------------------------------------------
# 4. Gain compensation (ecualizar brillo entre imagenes)
# ---------------------------------------------------------------------------
def compute_gains(images, homographies, ref_idx):
    """Estima un factor de ganancia por imagen para ecualizar brillo global.
    Usa el promedio de intensidad de cada imagen ponderado por area de overlap."""
    n = len(images)
    gains = np.ones(n, dtype=np.float64)

    ref_mean = np.mean(cv2.cvtColor(images[ref_idx], cv2.COLOR_BGR2GRAY))
    if ref_mean < 1.0:
        ref_mean = 1.0

    for i in range(n):
        if i == ref_idx:
            continue
        img_mean = np.mean(cv2.cvtColor(images[i], cv2.COLOR_BGR2GRAY))
        if img_mean < 1.0:
            img_mean = 1.0
        gains[i] = ref_mean / img_mean

    return gains


def apply_gain(img, gain):
    """Aplica factor de ganancia a una imagen BGR."""
    return np.clip(img.astype(np.float64) * gain, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# 5. Warping y canvas
# ---------------------------------------------------------------------------
def compute_canvas_bounds(images, abs_homographies):
    """Calcula los limites del canvas final dado las homografias absolutas."""
    all_corners = []
    for i, img in enumerate(images):
        h, w = img.shape[:2]
        corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
        warped = cv2.perspectiveTransform(corners, abs_homographies[i])
        all_corners.append(warped)

    all_corners = np.vstack(all_corners)
    x_min, y_min = all_corners.min(axis=0).ravel()
    x_max, y_max = all_corners.max(axis=0).ravel()
    return int(np.floor(x_min)), int(np.floor(y_min)), \
           int(np.ceil(x_max)),  int(np.ceil(y_max))


def warp_image(img, H, canvas_size, offset):
    """Aplica warp perspectivo y traslada al canvas."""
    ox, oy = offset
    T = np.array([[1, 0, -ox],
                   [0, 1, -oy],
                   [0, 0,  1]], dtype=np.float64)
    H_shifted = T @ H
    warped = cv2.warpPerspective(img, H_shifted, canvas_size,
                                  flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT,
                                  borderValue=0)
    # Mascara binaria del area valida
    mask = cv2.warpPerspective(np.ones(img.shape[:2], dtype=np.uint8) * 255,
                                H_shifted, canvas_size,
                                flags=cv2.INTER_NEAREST,
                                borderMode=cv2.BORDER_CONSTANT,
                                borderValue=0)
    return warped, mask


# ---------------------------------------------------------------------------
# 6. Blending
# ---------------------------------------------------------------------------
def feather_mask(mask, radius=15):
    """Crea una mascara con bordes suavizados (feathering) para blending."""
    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    dist = np.clip(dist / radius, 0, 1).astype(np.float32)
    return dist


def multiband_blend(warped_images, masks, levels=4):
    """Multiband blending usando piramides Laplacianas.
    Produce transiciones suaves incluso con diferencias de exposicion."""
    if not warped_images:
        return None

    h, w = warped_images[0].shape[:2]
    # Ajustar dimensiones para que sean divisibles por 2^levels
    factor = 2 ** levels
    h_pad = int(np.ceil(h / factor) * factor)
    w_pad = int(np.ceil(w / factor) * factor)

    def pad(img):
        if len(img.shape) == 3:
            return np.pad(img, ((0, h_pad - h), (0, w_pad - w), (0, 0)),
                         mode='constant')
        return np.pad(img, ((0, h_pad - h), (0, w_pad - w)),
                     mode='constant')

    # Construir piramides Gaussianas de las mascaras de peso
    weight_pyramids = []
    for mask in masks:
        weight = feather_mask(mask).astype(np.float32)
        weight = pad(weight)
        gp = [weight]
        for _ in range(levels):
            weight = cv2.pyrDown(weight)
            gp.append(weight)
        weight_pyramids.append(gp)

    # Construir piramides Laplacianas de las imagenes
    lap_pyramids = []
    for img in warped_images:
        img_f = pad(img).astype(np.float32)
        gp = [img_f]
        for _ in range(levels):
            img_f = cv2.pyrDown(img_f)
            gp.append(img_f)
        # Laplaciana
        lp = []
        for j in range(levels):
            expanded = cv2.pyrUp(gp[j + 1],
                                  dstsize=(gp[j].shape[1], gp[j].shape[0]))
            lp.append(gp[j] - expanded)
        lp.append(gp[levels])
        lap_pyramids.append(lp)

    # Mezclar en cada nivel de la piramide
    blended_pyr = []
    for level_idx in range(levels + 1):
        blended = np.zeros_like(lap_pyramids[0][level_idx])
        weight_sum = np.zeros(blended.shape[:2], dtype=np.float32)

        for img_idx in range(len(warped_images)):
            w_level = weight_pyramids[img_idx][level_idx]
            if len(blended.shape) == 3:
                w3 = w_level[:, :, np.newaxis]
            else:
                w3 = w_level
            blended += lap_pyramids[img_idx][level_idx] * w3
            weight_sum += w_level

        # Evitar division por cero
        weight_sum = np.maximum(weight_sum, 1e-6)
        if len(blended.shape) == 3:
            blended /= weight_sum[:, :, np.newaxis]
        else:
            blended /= weight_sum
        blended_pyr.append(blended)

    # Reconstruir desde la piramide
    result = blended_pyr[levels]
    for j in range(levels - 1, -1, -1):
        result = cv2.pyrUp(result,
                           dstsize=(blended_pyr[j].shape[1], blended_pyr[j].shape[0]))
        result += blended_pyr[j]

    result = np.clip(result, 0, 255).astype(np.uint8)
    return result[:h, :w]


def linear_blend(warped_images, masks):
    """Blending lineal simple con feathering. Alternativa ligera al multiband."""
    if not warped_images:
        return None

    h, w = warped_images[0].shape[:2]
    accumulator = np.zeros((h, w, 3), dtype=np.float64)
    weight_sum = np.zeros((h, w), dtype=np.float64)

    for img, mask in zip(warped_images, masks):
        weight = feather_mask(mask, radius=30).astype(np.float64)
        accumulator += img.astype(np.float64) * weight[:, :, np.newaxis]
        weight_sum += weight

    weight_sum = np.maximum(weight_sum, 1e-6)
    result = accumulator / weight_sum[:, :, np.newaxis]
    return np.clip(result, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# 7. Crop de bordes negros
# ---------------------------------------------------------------------------
def auto_crop(panorama, border_thresh=5):
    """Recorta bordes negros del panorama final."""
    gray = cv2.cvtColor(panorama, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, border_thresh, 255, cv2.THRESH_BINARY)

    # Encontrar el rectangulo mas grande contenido
    coords = cv2.findNonZero(thresh)
    if coords is None:
        return panorama
    x, y, w, h = cv2.boundingRect(coords)
    return panorama[y:y+h, x:x+w]


# ---------------------------------------------------------------------------
# 8. Pipeline principal
# ---------------------------------------------------------------------------
def build_chain_homographies(images, kps_list, descs_list, args):
    """Estima homografias encadenadas imagen-a-imagen.
    Retorna lista de homografias absolutas (relativas a la imagen de referencia)."""
    n = len(images)
    ref = n // 2  # imagen central como referencia

    # Homografias relativas: H_rel[i] transforma imagen i a imagen i+1
    H_rel = [None] * (n - 1)

    for i in range(n - 1):
        matches = match_pair(descs_list[i], descs_list[i + 1], args.ratio)
        H, mask, n_inliers = compute_homography(
            kps_list[i], kps_list[i + 1], matches, args.ransac
        )

        if H is None or n_inliers < args.min_matches:
            print(f"  FALLO: par ({i}, {i+1}) -- "
                  f"{len(matches)} matches, {n_inliers} inliers "
                  f"(minimo: {args.min_matches})")
            print(f"         Intenta reducir --step o aumentar overlap entre imagenes.")
            return None, ref

        H_rel[i] = H
        if args.debug:
            print(f"  Par ({i:3d}, {i+1:3d}): "
                  f"{len(matches):4d} matches, {n_inliers:4d} inliers")

    # Acumular homografias absolutas respecto a ref
    H_abs = [None] * n
    H_abs[ref] = np.eye(3, dtype=np.float64)

    # Hacia la derecha: ref+1, ref+2, ...
    for i in range(ref, n - 1):
        # H_rel[i] transforma i -> i+1, necesitamos i+1 -> ref
        # H_abs[i+1] = H_abs[i] @ inv(H_rel[i])
        H_abs[i + 1] = H_abs[i] @ np.linalg.inv(H_rel[i])

    # Hacia la izquierda: ref-1, ref-2, ...
    for i in range(ref, 0, -1):
        # H_rel[i-1] transforma i-1 -> i
        # H_abs[i-1] = H_abs[i] @ H_rel[i-1]
        H_abs[i - 1] = H_abs[i] @ H_rel[i - 1]

    return H_abs, ref


def stitch(args):
    """Pipeline completo de stitching."""
    t_start = time.time()

    # --- Cargar imagenes ---
    patterns = ["*.png", "*.jpg", "*.jpeg", "*.bmp"]
    paths = []
    for pat in patterns:
        paths.extend(glob.glob(os.path.join(args.input, pat)))
    paths = sorted(paths)

    if args.step > 1:
        paths = paths[::args.step]

    print(f"Imagenes a procesar: {len(paths)}")
    if len(paths) < 2:
        print("ERROR: se necesitan al menos 2 imagenes para stitching.")
        return

    # --- Cargar calibracion ---
    K, D, new_K = None, None, None
    if not args.no_undistort:
        K, D, _ = load_calibration(args.calib)
        print(f"Calibracion cargada: {args.calib}")

    # --- Cargar y preprocesar ---
    print("\n[1/5] Cargando y preprocesando imagenes...")
    images = []
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print(f"  WARN: no se pudo leer {p}, saltando.")
            continue
        if not args.no_undistort:
            img, new_K = undistort(img, K, D, alpha=0.0)
        images.append(img)

    if len(images) < 2:
        print("ERROR: insuficientes imagenes validas.")
        return

    print(f"  {len(images)} imagenes cargadas "
          f"({images[0].shape[1]}x{images[0].shape[0]} px)")

    # --- Features ---
    print("\n[2/5] Detectando features (SIFT)...")
    kps_list = []
    descs_list = []
    for i, img in enumerate(images):
        kps, descs = detect_features(img)
        kps_list.append(kps)
        descs_list.append(descs)
        if args.debug:
            n = len(kps) if kps else 0
            print(f"  Imagen {i:3d}: {n:5d} keypoints")

    # --- Homografias ---
    print("\n[3/5] Calculando homografias (RANSAC)...")
    H_abs, ref_idx = build_chain_homographies(images, kps_list, descs_list, args)
    if H_abs is None:
        print("\nERROR: no se pudo construir la cadena de homografias.")
        print("Posibles soluciones:")
        print("  - Aumentar overlap entre imagenes (reducir --step)")
        print("  - Reducir --ratio (mas estricto en matching)")
        print("  - Verificar que las imagenes tengan textura suficiente")
        return

    # --- Gain compensation ---
    gains = compute_gains(images, H_abs, ref_idx)
    images_comp = [apply_gain(img, g) for img, g in zip(images, gains)]

    # --- Warping ---
    print("\n[4/5] Warping y proyeccion al canvas...")
    x_min, y_min, x_max, y_max = compute_canvas_bounds(images_comp, H_abs)
    canvas_w = x_max - x_min
    canvas_h = y_max - y_min

    # Limitar tamano del canvas para evitar OOM
    MAX_DIM = 20000
    if canvas_w > MAX_DIM or canvas_h > MAX_DIM:
        print(f"  WARN: canvas demasiado grande ({canvas_w}x{canvas_h}). "
              f"Limitando a {MAX_DIM}px.")
        scale = min(MAX_DIM / canvas_w, MAX_DIM / canvas_h)
        for i in range(len(H_abs)):
            S = np.diag([scale, scale, 1.0])
            H_abs[i] = S @ H_abs[i]
        x_min, y_min, x_max, y_max = compute_canvas_bounds(images_comp, H_abs)
        canvas_w = x_max - x_min
        canvas_h = y_max - y_min

    print(f"  Canvas: {canvas_w}x{canvas_h} px")
    canvas_size = (canvas_w, canvas_h)
    offset = (x_min, y_min)

    warped_images = []
    warped_masks = []
    for i, img in enumerate(images_comp):
        w_img, w_mask = warp_image(img, H_abs[i], canvas_size, offset)
        warped_images.append(w_img)
        warped_masks.append(w_mask)

    # --- Blending ---
    print("\n[5/5] Blending...")
    if args.blend == "multiband":
        print("  Metodo: multiband (piramides Laplacianas)")
        panorama = multiband_blend(warped_images, warped_masks, levels=args.levels)
    else:
        print("  Metodo: linear (feathering)")
        panorama = linear_blend(warped_images, warped_masks)

    if panorama is None:
        print("ERROR: blending fallo.")
        return

    # --- Crop ---
    if not args.no_crop:
        panorama = auto_crop(panorama)

    # --- Guardar ---
    os.makedirs(args.output, exist_ok=True)
    out_path = os.path.join(args.output, "panorama.png")
    cv2.imwrite(out_path, panorama)

    elapsed = time.time() - t_start
    print(f"\nPanorama guardado: {out_path}")
    print(f"  Tamano: {panorama.shape[1]}x{panorama.shape[0]} px")
    print(f"  Tiempo: {elapsed:.1f}s")

    # Debug: guardar tambien imagenes warpeadas individuales
    if args.debug:
        debug_dir = os.path.join(args.output, "debug")
        os.makedirs(debug_dir, exist_ok=True)
        for i, (w_img, w_mask) in enumerate(zip(warped_images, warped_masks)):
            cv2.imwrite(os.path.join(debug_dir, f"warped_{i:04d}.png"), w_img)
            cv2.imwrite(os.path.join(debug_dir, f"mask_{i:04d}.png"), w_mask)
        print(f"  Debug: {len(warped_images)} warped images en {debug_dir}/")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="Stitching robusto de imagenes aereas del Tello.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    p.add_argument("--input", "-i", required=True,
                   help="Directorio con imagenes a unir (ordenadas por nombre).")
    p.add_argument("--output", "-o", default=DEFAULT_OUTPUT_DIR,
                   help=f"Directorio de salida (default: {DEFAULT_OUTPUT_DIR})")
    p.add_argument("--calib", default=DEFAULT_CALIB,
                   help="Ruta al .npz de calibracion.")
    p.add_argument("--no-undistort", action="store_true",
                   help="No aplicar correccion de distorsion.")
    p.add_argument("--step", type=int, default=1,
                   help="Usar cada N-esima imagen (default: 1 = todas).")
    p.add_argument("--ratio", type=float, default=DEFAULT_RATIO,
                   help=f"Lowe's ratio threshold (default: {DEFAULT_RATIO}). "
                        "Menor = mas estricto.")
    p.add_argument("--min-matches", type=int, default=DEFAULT_MIN_MATCHES,
                   help=f"Minimo de inliers RANSAC por par (default: {DEFAULT_MIN_MATCHES}).")
    p.add_argument("--ransac", type=float, default=DEFAULT_RANSAC_THRESH,
                   help=f"Threshold RANSAC en px (default: {DEFAULT_RANSAC_THRESH}).")
    p.add_argument("--blend", choices=["multiband", "linear"], default="multiband",
                   help="Metodo de blending (default: multiband).")
    p.add_argument("--levels", type=int, default=4,
                   help="Niveles de piramide para multiband (default: 4).")
    p.add_argument("--no-crop", action="store_true",
                   help="No recortar bordes negros del panorama.")
    p.add_argument("--debug", action="store_true",
                   help="Guardar imagenes intermedias y mostrar info extra.")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    stitch(args)
