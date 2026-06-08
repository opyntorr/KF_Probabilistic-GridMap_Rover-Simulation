#!/usr/bin/env python3
# perf_hud.py — métricas de rendimiento para scene_mecanum.py (Isaac Sim 4.5).
#
# En escenas SCRIPTED (SimulationApp standalone) el HUD de estadísticas/FPS que sí
# aparece en el editor de Isaac NO se muestra por defecto: el lanzador arranca con
# display_options=3094 (oculta cosas del viewport para no contaminar synthetic data);
# 3286 es el "buen default del editor" que SÍ muestra las estadísticas.
#   Fuente: exts/isaacsim.simulation_app/isaacsim/simulation_app/simulation_app.py
#           DEFAULT_LAUNCHER_CONFIG["display_options"] = 3094  (línea ~69) y el docstring
#           de display_options (línea ~99): "3286 is another good default, used for the
#           regular isaac-sim editor experience". El valor se aplica como arg de arranque
#           --/persistent/app/viewport/displayOptions=<n> (línea ~311).
#
# Este módulo aporta:
#   - enable_stats_overlay(): activa el HUD de viewport (3286) por carb.settings en CALIENTE.
#   - PerfMonitor: mide FPS (cronómetro del update_event_stream, igual que constant_fps.py),
#     RTF = Δsim_time/Δwall, GPU util+mem (pynvml con fallback a nvidia-smi) y RAM (psutil),
#     e imprime "[PERF] fps=.. rtf=.. gpu=..% gmem=..MB ram=..%" cada ~1 s. Opcional: publica
#     std_msgs/String en /isaac/perf y/o dibuja texto en el viewport.
#
# CONTRATO DE INTEGRACIÓN: sin efectos colaterales al importar; todo dentro de funciones/clase.
# El orchestrator decide cuándo llamar enable_stats_overlay() y cuándo instanciar PerfMonitor.
#
# APIs citadas (todas leídas del install local):
#   - update_event_stream + perf_counter_ns:
#       standalone_examples/api/isaacsim.simulation_app/constant_fps.py (líneas 31-37):
#         omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(cb)
#         timestamp_ns = time.perf_counter_ns()
#   - sim time:
#       omni.timeline.get_timeline_interface().get_current_time()  (ya usado en scene_mecanum.py
#       línea ~502: timeline.get_current_time()). Aceptamos también un sim_time externo (p.ej.
#       de IsaacReadSimulationTime / world.current_time) vía PerfMonitor.tick(sim_time=...).
#   - pynvml (bundled: extscache/omni.services.pip_archive-*/pip_prebundle/pynvml/nvml.py):
#       nvmlInit(); nvmlDeviceGetHandleByIndex(i); nvmlDeviceGetUtilizationRates(h).gpu (%);
#       nvmlDeviceGetMemoryInfo(h).used/.total (bytes)  [c_nvmlUtilization_t.gpu (línea 990),
#       c_nvmlMemory_t.total/free/used (líneas 933-936)].
#   - psutil (bundled: extscache/omni.kit.pip_archive-*/pip_prebundle/psutil):
#       psutil.virtual_memory().percent  (patrón de datarecorders/memory.py y cpu.py).

import shutil
import subprocess
import time


# --------------------------------------------------------------------------
# Overlay de estadísticas del viewport (FPS/HUD del editor)
# --------------------------------------------------------------------------
# display_options es un bitmask; los bits extra de 3286 respecto a 3094 encienden
# la capa de estadísticas/FPS del viewport. Ver simulation_app.py (arriba).
DISPLAY_OPTIONS_STATS = 3286     # editor-like: muestra HUD de estadísticas/FPS
DISPLAY_OPTIONS_HIDDEN = 3094    # default del lanzador: oculta el HUD


def enable_stats_overlay(display_options: int = DISPLAY_OPTIONS_STATS) -> bool:
    """Enciende el HUD de estadísticas/FPS del viewport en caliente (no headless).

    Equivale a lanzar SimulationApp con {"display_options": 3286}. En una escena ya
    arrancada, escribe el mismo setting persistente que usa el lanzador:
        --/persistent/app/viewport/displayOptions=<n>   (simulation_app.py línea ~311)

    Devuelve True si pudo escribir el setting. No-fatal: en headless no hay viewport,
    así que solo deja el setting puesto (sin error).
    """
    try:
        import carb

        settings = carb.settings.get_settings()
        # clave persistente exacta usada por el lanzador de SimulationApp
        settings.set("/persistent/app/viewport/displayOptions", int(display_options))
        # algunos builds leen también la clave no-persistente del viewport activo
        settings.set("/app/viewport/displayOptions", int(display_options))
        # forzar que la barra de stats del viewport esté visible (omni.kit.viewport.window)
        settings.set("/persistent/app/viewport/Viewport/Viewport0/fillViewport", False)
        return True
    except Exception as e:  # noqa: BLE001
        print(f"[PERF] no pude activar el HUD de stats ({e})", flush=True)
        return False


# --------------------------------------------------------------------------
# Lectura de GPU: pynvml con fallback a nvidia-smi
# --------------------------------------------------------------------------
class _GpuReader:
    """Lee util % y memoria de la GPU. Prefiere pynvml; si falta, parsea nvidia-smi."""

    def __init__(self, gpu_index: int = 0):
        self._idx = gpu_index
        self._mode = "none"
        self._handle = None
        self._nvml = None
        self._smi = shutil.which("nvidia-smi")
        self._warned = False
        # 1) intento pynvml (viene bundleado con Isaac; ver cabecera)
        try:
            import pynvml

            pynvml.nvmlInit()
            self._handle = pynvml.nvmlDeviceGetHandleByIndex(gpu_index)
            self._nvml = pynvml
            self._mode = "pynvml"
        except Exception:  # noqa: BLE001 — pynvml ausente o sin driver
            self._nvml = None
            self._handle = None
            if self._smi:
                self._mode = "nvidia-smi"

    def read(self):
        """Devuelve (util_pct, mem_used_MB, mem_total_MB); None en los que no se puedan leer."""
        if self._mode == "pynvml":
            try:
                util = self._nvml.nvmlDeviceGetUtilizationRates(self._handle)
                mem = self._nvml.nvmlDeviceGetMemoryInfo(self._handle)
                # c_nvmlUtilization_t.gpu (%) ; c_nvmlMemory_t.used/.total (bytes)
                return (
                    float(util.gpu),
                    mem.used / (1024.0 ** 2),
                    mem.total / (1024.0 ** 2),
                )
            except Exception:  # noqa: BLE001 — cae al fallback en caliente
                self._mode = "nvidia-smi" if self._smi else "none"

        if self._mode == "nvidia-smi":
            try:
                out = subprocess.check_output(
                    [
                        "nvidia-smi",
                        "--query-gpu=utilization.gpu,memory.used,memory.total",
                        "--format=csv,noheader,nounits",
                        f"--id={self._idx}",
                    ],
                    stderr=subprocess.DEVNULL,
                    timeout=2.0,
                ).decode("utf-8", "ignore")
                line = out.strip().splitlines()[0]
                util_s, used_s, total_s = (p.strip() for p in line.split(","))
                return (float(util_s), float(used_s), float(total_s))
            except Exception:  # noqa: BLE001
                if not self._warned:
                    print("[PERF] nvidia-smi no disponible; GPU stats off", flush=True)
                    self._warned = True
                return (None, None, None)

        return (None, None, None)

    def shutdown(self):
        if self._mode == "pynvml" and self._nvml is not None:
            try:
                self._nvml.nvmlShutdown()
            except Exception:  # noqa: BLE001
                pass


# --------------------------------------------------------------------------
# Monitor principal
# --------------------------------------------------------------------------
class PerfMonitor:
    """Mide y reporta FPS / RTF / GPU / RAM en una escena standalone de Isaac.

    Uso (en scene_mecanum.py, después de crear `simulation_app` y de timeline.play()):
        from perf_hud import PerfMonitor, enable_stats_overlay
        enable_stats_overlay()                 # opcional, solo con ventana
        perf = PerfMonitor(simulation_app, timeline=timeline, publish_ros=True)
        ...
        while simulation_app.is_running():
            ...
            simulation_app.update()
            perf.tick()                        # 1 vez por iteración del bucle
        perf.shutdown()                        # en el finally

    El FPS se cronometra con el update_event_stream de omni.kit.app (mismo patrón que
    constant_fps.py): cada update() dispara un evento y medimos Δ time.perf_counter_ns.
    Así el FPS refleja el ritmo REAL de la app, no el del bucle de Python.
    """

    def __init__(
        self,
        simulation_app,
        timeline=None,
        gpu_index: int = 0,
        report_period_s: float = 1.0,
        publish_ros: bool = False,
        ros_topic: str = "/isaac/perf",
        ros_node=None,
        viewport_overlay: bool = False,
        prefix: str = "[PERF]",
    ):
        self._app = simulation_app
        self._timeline = timeline
        self._period = float(report_period_s)
        self._prefix = prefix
        self._gpu = _GpuReader(gpu_index)

        # --- cronómetro de FPS por el update_event_stream (constant_fps.py) ---
        self._frame_count = 0          # frames de app desde el último reporte
        self._sub = None
        try:
            import omni.kit.app

            self._sub = (
                omni.kit.app.get_app()
                .get_update_event_stream()
                .create_subscription_to_pop(self._on_update, name="perf_hud_fps")
            )
        except Exception as e:  # noqa: BLE001
            print(f"[PERF] sin update_event_stream ({e}); FPS por bucle", flush=True)

        # --- ventanas de tiempo para FPS (wall) y RTF (sim vs wall) ---
        now_ns = time.perf_counter_ns()
        self._t_report_ns = now_ns          # inicio de la ventana de reporte (wall)
        self._wall_last_ns = now_ns         # wall al inicio de la ventana de RTF
        self._sim_last = self._read_sim_time()  # sim_time al inicio de la ventana de RTF

        # --- ROS2 opcional (std_msgs/String en /isaac/perf) ---
        self._pub = None
        self._owns_node = False
        self._ros_node = ros_node
        self._last_line = ""
        if publish_ros:
            self._setup_ros(ros_topic, ros_node)

        # --- overlay de texto en viewport opcional ---
        self._overlay_ctx = None
        if viewport_overlay:
            self._setup_overlay()

    # ---- callbacks / helpers internos ------------------------------------
    def _on_update(self, _event):
        # se llama una vez por cada simulation_app.update() (= frame de la app)
        self._frame_count += 1

    def _read_sim_time(self):
        """sim_time actual: usa el timeline si se pasó (get_current_time, ya usado en la
        escena), si no intenta crear uno. Devuelve None si no se puede leer."""
        tl = self._timeline
        if tl is None:
            try:
                import omni.timeline

                tl = omni.timeline.get_timeline_interface()
                self._timeline = tl
            except Exception:  # noqa: BLE001
                return None
        try:
            return float(tl.get_current_time())
        except Exception:  # noqa: BLE001
            return None

    def _setup_ros(self, topic, node):
        try:
            from std_msgs.msg import String

            if node is None:
                import rclpy

                if not rclpy.ok():
                    rclpy.init()
                node = rclpy.create_node("isaac_perf_hud")
                self._owns_node = True
            self._ros_node = node
            self._pub = node.create_publisher(String, topic, 10)
            self._String = String
            print(f"[PERF] publicando std_msgs/String en {topic}", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[PERF] ROS2 perf off ({e})", flush=True)
            self._pub = None

    def _setup_overlay(self):
        # Texto en viewport vía omni.ui (solo con ventana). No-fatal en headless.
        try:
            import omni.ui as ui

            self._ui = ui
            self._overlay_ctx = {"label": None}
        except Exception as e:  # noqa: BLE001
            print(f"[PERF] overlay de viewport off ({e})", flush=True)
            self._overlay_ctx = None

    # ---- API pública ------------------------------------------------------
    def tick(self, sim_time: float = None):
        """Llamar 1 vez por iteración del bucle principal (tras simulation_app.update()).

        Si se pasa `sim_time` (p.ej. world.current_time o salida de IsaacReadSimulationTime)
        se usa ese para el RTF; si no, se lee del timeline. Cada ~report_period_s imprime
        (y opcionalmente publica/dibuja) una línea de métricas.
        """
        now_ns = time.perf_counter_ns()
        dt_report = (now_ns - self._t_report_ns) / 1e9
        if dt_report < self._period:
            return  # aún no toca reportar

        # --- FPS: frames de app / segundos de pared en esta ventana ---
        if self._sub is not None and self._frame_count > 0:
            fps = self._frame_count / dt_report
        else:
            # fallback: 1 frame por tick() (el bucle llama tick una vez por update)
            fps = 1.0 / dt_report if dt_report > 0 else 0.0

        # --- RTF: avance de sim_time / avance de wall en esta ventana ---
        sim_now = sim_time if sim_time is not None else self._read_sim_time()
        dt_wall = (now_ns - self._wall_last_ns) / 1e9
        rtf = None
        if sim_now is not None and self._sim_last is not None and dt_wall > 0:
            rtf = (sim_now - self._sim_last) / dt_wall

        # --- GPU + RAM ---
        gpu_util, gmem_used, gmem_total = self._gpu.read()
        ram_pct = None
        try:
            import psutil

            ram_pct = psutil.virtual_memory().percent
        except Exception:  # noqa: BLE001
            ram_pct = None

        # --- formato de la línea ---
        gpu_s = f"{gpu_util:.0f}" if gpu_util is not None else "na"
        gmem_s = f"{gmem_used:.0f}" if gmem_used is not None else "na"
        ram_s = f"{ram_pct:.0f}" if ram_pct is not None else "na"
        rtf_s = f"{rtf:.2f}" if rtf is not None else "na"
        line = (
            f"{self._prefix} fps={fps:.1f} rtf={rtf_s} "
            f"gpu={gpu_s}% gmem={gmem_s}MB ram={ram_s}%"
        )
        print(line, flush=True)
        self._last_line = line

        # --- publicación ROS2 opcional ---
        if self._pub is not None:
            try:
                msg = self._String()
                msg.data = line
                self._pub.publish(msg)
            except Exception:  # noqa: BLE001
                pass

        # --- overlay de viewport opcional ---
        self._draw_overlay(line)

        # --- reset de ventanas ---
        self._t_report_ns = now_ns
        self._wall_last_ns = now_ns
        self._sim_last = sim_now
        self._frame_count = 0

    def _draw_overlay(self, text):
        ctx = self._overlay_ctx
        if ctx is None:
            return
        try:
            ui = self._ui
            if ctx["label"] is None:
                # ventana flotante mínima en la esquina con el texto de métricas
                ctx["window"] = ui.Window(
                    "Perf", width=360, height=64,
                    flags=ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_NO_SCROLLBAR,
                )
                with ctx["window"].frame:
                    ctx["label"] = ui.Label(text, style={"font_size": 16})
            else:
                ctx["label"].text = text
        except Exception:  # noqa: BLE001
            self._overlay_ctx = None  # desactiva si el UI no está disponible

    @property
    def last_line(self):
        """Última línea de métricas formateada (str)."""
        return self._last_line

    def shutdown(self):
        """Libera la suscripción, pynvml y el nodo ROS si lo creó este monitor."""
        if self._sub is not None:
            try:
                self._sub.unsubscribe()
            except Exception:  # noqa: BLE001
                pass
            self._sub = None
        self._gpu.shutdown()
        if self._owns_node and self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:  # noqa: BLE001
                pass


# Conveniencia: documentado en el docstring del módulo.
def recommended_simulation_app_config(headless: bool = False) -> dict:
    """Config sugerida para SimulationApp que muestra el HUD de stats desde el arranque.

    Pasar esto a SimulationApp(...) equivale a llamar enable_stats_overlay() pero ya
    desde el primer frame:
        SimulationApp(recommended_simulation_app_config())
    Solo afecta al modo con ventana (en headless no hay viewport que mostrar).
    """
    return {
        "renderer": "RaytracedLighting",
        "headless": headless,
        "display_options": DISPLAY_OPTIONS_STATS,  # 3286: HUD de stats visible
    }
