#!/usr/bin/env python3
"""
Helper de una línea para cargar la calibración desde otros scripts.

Uso desde otro módulo:
    from camera_calibration.load_calibration import load_tello_calibration
    K, D = load_tello_calibration()
"""
import numpy as np
import os

_NPZ = os.path.join(os.path.dirname(__file__), "calibration_output", "tello_calibration.npz")


def load_tello_calibration(path: str = _NPZ):
    """Devuelve (K, D, image_size, rms) desde el archivo .npz."""
    data = np.load(path)
    return data["K"], data["D"], tuple(data["image_size"]), float(data["rms"])


if __name__ == "__main__":
    K, D, size, rms = load_tello_calibration()
    print(f"Resolución: {size[0]}×{size[1]}")
    print(f"RMS: {rms:.4f} px")
    print(f"K:\n{K}")
    print(f"D: {D.ravel()}")
    print(f"  [k1={D[0,0]:.6f}, k2={D[0,1]:.6f}, "
          f"p1={D[0,2]:.6f}, p2={D[0,3]:.6f}, k3={D[0,4]:.6f}]")
