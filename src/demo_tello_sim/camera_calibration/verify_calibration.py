#!/usr/bin/env python3
"""
Verifica la calibración aplicando undistort a todas las imágenes
y mostrando/guardando comparaciones original vs corregida.

Uso:
  python3 verify_calibration.py           # revisa primera imagen
  python3 verify_calibration.py --all     # guarda comparación de todas
"""
import cv2
import numpy as np
import os
import glob
import sys

IMAGES_DIR = os.path.join(os.path.dirname(__file__), "images")
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "calibration_output")
VERIFY_DIR = os.path.join(OUTPUT_DIR, "verify")


def load_calibration():
    npz_path = os.path.join(OUTPUT_DIR, "tello_calibration.npz")
    if not os.path.exists(npz_path):
        raise FileNotFoundError(f"No encontrado: {npz_path}\n"
                                "Ejecuta primero calibrate_charuco.py")
    data = np.load(npz_path)
    return data["K"], data["D"], tuple(data["image_size"]), float(data["rms"])


def undistort_image(img, K, D, alpha=0.0):
    h, w = img.shape[:2]
    # alpha=0: recorta píxeles negros | alpha=1: mantiene todos los píxeles
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=alpha)
    undistorted = cv2.undistort(img, K, D, None, new_K)
    return undistorted, new_K


def make_comparison(original, undistorted):
    h = max(original.shape[0], undistorted.shape[0])

    def pad(img):
        ph = h - img.shape[0]
        return np.pad(img, ((0, ph), (0, 0), (0, 0)))

    orig_p = pad(original)
    undi_p = pad(undistorted)
    combined = np.hstack([orig_p, undi_p])
    w = original.shape[1]
    cv2.putText(combined, "Original",     (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 80, 255), 2)
    cv2.putText(combined, "Undistorted",  (w + 10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 0), 2)
    return combined


def main():
    K, D, image_size, rms = load_calibration()
    print(f"Calibración cargada — RMS: {rms:.4f} px")
    print(f"K:\n{K}")
    print(f"D: {D.ravel()}")

    images = sorted(glob.glob(os.path.join(IMAGES_DIR, "*.png")))
    if not images:
        print("No hay imágenes en images/")
        return

    mode_all = "--all" in sys.argv
    targets = images if mode_all else [images[0]]

    if mode_all:
        os.makedirs(VERIFY_DIR, exist_ok=True)

    for path in targets:
        img = cv2.imread(path)
        undistorted, new_K = undistort_image(img, K, D, alpha=0.0)
        comparison = make_comparison(img, undistorted)

        if mode_all:
            out = os.path.join(VERIFY_DIR, "cmp_" + os.path.basename(path))
            cv2.imwrite(out, comparison)
            print(f"  Guardada: {out}")
        else:
            out = os.path.join(OUTPUT_DIR, "undistort_comparison.png")
            cv2.imwrite(out, comparison)
            print(f"Comparación guardada: {out}")
            cv2.imshow("Original | Undistorted  (cualquier tecla para salir)", comparison)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    if mode_all:
        print(f"\n{len(targets)} comparaciones guardadas en: {VERIFY_DIR}")


if __name__ == "__main__":
    main()
