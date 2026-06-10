#!/usr/bin/env python3
"""
Captura imagenes para stitching desde el DJI Tello.

Dos modos de operacion:
  MANUAL  - presiona SPACE para guardar cada foto (default)
  AUTO    - guarda automaticamente cada N segundos (--auto --interval 2)

ANTES DE EJECUTAR:
  - Conecta el PC al WiFi del Tello (TELLO-XXXXXX)
  - El dron puede estar en la mano o volando

Controles:
  SPACE  - guardar foto (modo manual)
  A      - toggle modo auto
  Q      - salir

Uso:
  python3 capture_stitching.py
  python3 capture_stitching.py --auto --interval 1.5
  python3 capture_stitching.py --output ./mis_fotos/
"""
import os
import sys
import time
import argparse

os.environ.setdefault("QT_LOGGING_RULES", "qt.qpa.fonts=false")
os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")

import cv2
from djitellopy import Tello

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUTPUT = os.path.join(SCRIPT_DIR, "stitching_input")
WINDOW = "Tello - Captura Stitching"


def main():
    parser = argparse.ArgumentParser(description="Captura imagenes para stitching.")
    parser.add_argument("--output", "-o", default=DEFAULT_OUTPUT,
                        help=f"Directorio de salida (default: {DEFAULT_OUTPUT})")
    parser.add_argument("--auto", action="store_true",
                        help="Captura automatica por intervalo de tiempo.")
    parser.add_argument("--interval", type=float, default=2.0,
                        help="Segundos entre capturas automaticas (default: 2.0)")
    parser.add_argument("--flip", action="store_true",
                        help="Voltear imagen horizontalmente (si usas espejo).")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    tello = Tello()
    tello.connect()
    battery = tello.get_battery()
    print(f"Bateria: {battery}%")
    if battery < 15:
        print("ADVERTENCIA: bateria baja, considera cargar antes de volar.")

    tello.streamon()
    frame_read = tello.get_frame_read()

    # Contar imagenes existentes para continuar la numeracion
    existing = [f for f in os.listdir(args.output)
                if f.startswith("stitch_") and f.endswith(".png")]
    count = len(existing)
    print(f"Imagenes existentes en {args.output}: {count}")

    auto_mode = args.auto
    last_capture = 0.0

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    print(f"\nModo: {'AUTO (cada {:.1f}s)'.format(args.interval) if auto_mode else 'MANUAL (SPACE)'}")
    print("SPACE=guardar  A=toggle auto  Q=salir\n")

    while True:
        frame = frame_read.frame
        if frame is None:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        if args.flip:
            frame = cv2.flip(frame, 1)

        display = frame.copy()
        now = time.time()

        # Info overlay
        mode_str = f"AUTO ({args.interval:.1f}s)" if auto_mode else "MANUAL"
        cv2.putText(display, f"Capturadas: {count} | Modo: {mode_str} | Q=salir",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)

        # Guia visual: retícula central para alinear overlap
        h, w = display.shape[:2]
        # Lineas de tercios (ayudan a mantener ~30% overlap)
        color_guide = (80, 80, 80)
        cv2.line(display, (w // 3, 0), (w // 3, h), color_guide, 1)
        cv2.line(display, (2 * w // 3, 0), (2 * w // 3, h), color_guide, 1)
        cv2.line(display, (0, h // 3), (w, h // 3), color_guide, 1)
        cv2.line(display, (0, 2 * h // 3), (w, 2 * h // 3), color_guide, 1)

        # Indicador de overlap: franjas laterales
        overlap_w = w // 4  # 25% del ancho como zona de overlap sugerida
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (overlap_w, h), (0, 180, 0), -1)
        cv2.rectangle(overlay, (w - overlap_w, 0), (w, h), (0, 180, 0), -1)
        cv2.addWeighted(overlay, 0.12, display, 0.88, 0, display)
        cv2.putText(display, "overlap", (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 150, 0), 1)
        cv2.putText(display, "overlap", (w - overlap_w + 5, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 150, 0), 1)

        # Captura automatica
        saved_this_frame = False
        if auto_mode and (now - last_capture) >= args.interval:
            path = os.path.join(args.output, f"stitch_{count:04d}.png")
            cv2.imwrite(path, frame)
            count += 1
            last_capture = now
            saved_this_frame = True
            print(f"  AUTO: {os.path.basename(path)}")

        if saved_this_frame:
            cv2.putText(display, "GUARDADA", (w // 2 - 80, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

        cv2.imshow(WINDOW, display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('a'):
            auto_mode = not auto_mode
            mode_str = f"AUTO ({args.interval:.1f}s)" if auto_mode else "MANUAL"
            print(f"  Modo cambiado a: {mode_str}")
            last_capture = now  # resetear timer
        elif key == ord(' '):
            path = os.path.join(args.output, f"stitch_{count:04d}.png")
            cv2.imwrite(path, frame)
            count += 1
            print(f"  MANUAL: {os.path.basename(path)}")

    tello.streamoff()
    tello.end()
    cv2.destroyAllWindows()
    print(f"\nTotal imagenes capturadas: {count}")
    print(f"Directorio: {args.output}")
    print(f"\nPara hacer stitching:")
    print(f"  python3 camera_calibration/stitch_images.py --input {args.output}")


if __name__ == "__main__":
    main()
