import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import csv
import os
import numpy as np
import signal
from rclpy.signals import SignalHandlerOptions

class TelloPlotter(Node):
    def __init__(self):
        super().__init__('tello_plotter')
        
        # Suscriptor a la odometría y a la meta
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.target_sub = self.create_subscription(Point, '/drone1/target_position', self.target_callback, 10)
        
        # Posición objetivo inicial
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_received = False
        
        # Suscriptor al drift
        self.drift_sub = self.create_subscription(Point, '/drone1/drift_info', self.drift_callback, 10)
        self.drift_x = 0.0
        self.drift_y = 0.0
        self.drift_magnitude = 0.0
        self.drift_received = False
        
        # Historial de datos
        self.time_history = []
        self.x_history = []
        self.y_history = []
        self.z_history = []
        self.tx_history = []
        self.ty_history = []
        self.tz_history = []
        self.start_time = None
        
        # Historial de ruido (jitter = diferencia entre muestras consecutivas)
        # Preparar datos estadísticos
        self.jitter_x = []
        self.jitter_y = []
        self.jitter_z = []
        self.jitter_time = []
        
        self.get_logger().info("Plotter iniciado. Esperando datos...")
        
        # Varianza en ventana deslizante
        self.WINDOW_SIZE = 25  # Ventana de ~2.5s a 10Hz
        self.var_x = []
        self.var_y = []
        self.var_z = []
        self.var_time = []
        
        # Configuración de gráfica en tiempo real
        plt.ion()
        self.fig = plt.figure(figsize=(16, 9))
        gs = gridspec.GridSpec(3, 3, figure=self.fig, wspace=0.35, hspace=0.4)
        
        # Columna 0: Subplot 3D
        self.ax3d = self.fig.add_subplot(gs[:, 0], projection='3d')
        self.ax3d.set_title('Trayectoria 3D')
        self.ax3d.set_xlabel('X [m]'); self.ax3d.set_ylabel('Y [m]'); self.ax3d.set_zlabel('Z [m]')
        self.ax3d.plot([0], [0], [0], 'go', label='Inicio')
        self.target_line3d, = self.ax3d.plot([], [], [], 'r--', label='Meta')
        self.line3d, = self.ax3d.plot([], [], [], 'b-', label='Trayecto')
        self.ax3d.set_xlim([-0.5, max(3.0, self.target_x + 1)])
        self.ax3d.set_ylim([-0.5, max(3.0, self.target_y + 1)])
        self.ax3d.set_zlim([0, max(2.0, self.target_z + 1)])
        self.ax3d.legend(fontsize=7)
        
        # Columna 1: Posición X, Y, Z vs Tiempo
        self.ax_x = self.fig.add_subplot(gs[0, 1])
        self.line_x, = self.ax_x.plot([], [], 'b-')
        self.target_line_x, = self.ax_x.plot([], [], 'r--', label='Meta X')
        self.ax_x.set_ylabel('X [m]'); self.ax_x.grid(True)
        
        self.ax_y = self.fig.add_subplot(gs[1, 1])
        self.line_y, = self.ax_y.plot([], [], 'g-')
        self.target_line_y, = self.ax_y.plot([], [], 'r--', label='Meta Y')
        self.ax_y.set_ylabel('Y [m]'); self.ax_y.grid(True)
        
        self.ax_z = self.fig.add_subplot(gs[2, 1])
        self.line_z, = self.ax_z.plot([], [], 'm-')
        self.target_line_z, = self.ax_z.plot([], [], 'r--', label='Meta Z')
        self.ax_z.set_ylabel('Z [m]'); self.ax_z.set_xlabel('Tiempo [s]'); self.ax_z.grid(True)
        
        # Columna 2: Análisis de Ruido
        self.ax_jitter = self.fig.add_subplot(gs[0, 2])
        self.jline_x, = self.ax_jitter.plot([], [], 'b-', alpha=0.7, label='ΔX', linewidth=0.8)
        self.jline_y, = self.ax_jitter.plot([], [], 'g-', alpha=0.7, label='ΔY', linewidth=0.8)
        self.jline_z, = self.ax_jitter.plot([], [], 'm-', alpha=0.7, label='ΔZ', linewidth=0.8)
        self.ax_jitter.set_ylabel('Jitter [m]'); self.ax_jitter.set_title('Ruido (Jitter)', fontsize=9)
        self.ax_jitter.grid(True); self.ax_jitter.legend(fontsize=7)
        
        self.ax_var = self.fig.add_subplot(gs[1, 2])
        self.vline_x, = self.ax_var.plot([], [], 'b-', label='σ² X', linewidth=1.0)
        self.vline_y, = self.ax_var.plot([], [], 'g-', label='σ² Y', linewidth=1.0)
        self.vline_z, = self.ax_var.plot([], [], 'm-', label='σ² Z', linewidth=1.0)
        self.ax_var.set_ylabel('Varianza [m²]'); self.ax_var.set_title(f'Varianza (ventana {self.WINDOW_SIZE})', fontsize=9)
        self.ax_var.grid(True); self.ax_var.legend(fontsize=7)
        
        self.ax_stats = self.fig.add_subplot(gs[2, 2])
        self.ax_stats.axis('off')
        self.ax_stats.set_title('Estadísticas de Ruido', fontsize=9)
        self.stats_text = self.ax_stats.text(
            0.05, 0.95, 'Esperando datos...',
            transform=self.ax_stats.transAxes, fontsize=8,
            verticalalignment='top', family='monospace')
        
        plt.show()

        # Timer para actualizar la gráfica (10 Hz)
        self.timer = self.create_timer(0.1, self.update_plot)
        self.messages_received = 0

    def drift_callback(self, msg):
        self.drift_x = msg.x
        self.drift_y = msg.y
        self.drift_magnitude = np.sqrt(msg.x**2 + msg.y**2)
        self.drift_received = True

    def odom_callback(self, msg):
        # Usar el timestamp del mensaje (cuando el integrador lo publicó),
        # NO self.get_clock().now() (cuando el plotter lo procesó).
        # Esto evita que matplotlib distorsione los timestamps al bloquear el executor.
        msg_sec = msg.header.stamp.sec
        msg_nsec = msg.header.stamp.nanosec
        msg_time_ns = msg_sec * 1_000_000_000 + msg_nsec
        
        if self.start_time is None:
            self.start_time = msg_time_ns
            
        t = (msg_time_ns - self.start_time) / 1e9
        pose = msg.pose.pose.position
        
        if not self.target_received:
            self.target_x = pose.x
            self.target_y = pose.y
            self.target_z = pose.z
        
        # Calcular jitter (diferencia entre muestras consecutivas)
        if len(self.x_history) > 0:
            self.jitter_x.append(pose.x - self.x_history[-1])
            self.jitter_y.append(pose.y - self.y_history[-1])
            self.jitter_z.append(pose.z - self.z_history[-1])
            self.jitter_time.append(t)
        
        self.time_history.append(t)
        self.x_history.append(pose.x)
        self.y_history.append(pose.y)
        self.z_history.append(pose.z)
        self.tx_history.append(self.target_x)
        self.ty_history.append(self.target_y)
        self.tz_history.append(self.target_z)
        
        # Calcular varianza en ventana deslizante
        n = len(self.x_history)
        if n >= self.WINDOW_SIZE:
            wx = self.x_history[-self.WINDOW_SIZE:]
            wy = self.y_history[-self.WINDOW_SIZE:]
            wz = self.z_history[-self.WINDOW_SIZE:]
            self.var_x.append(np.var(wx))
            self.var_y.append(np.var(wy))
            self.var_z.append(np.var(wz))
            self.var_time.append(t)
        
        self.messages_received += 1

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_z = msg.z
        self.target_received = True
        
        # Ajustar límites 3D si la meta está fuera de la vista
        curr_xlim = self.ax3d.get_xlim()
        curr_ylim = self.ax3d.get_ylim()
        curr_zlim = self.ax3d.get_zlim()
        self.ax3d.set_xlim([min(curr_xlim[0], -0.5), max(curr_xlim[1], self.target_x + 1)])
        self.ax3d.set_ylim([min(curr_ylim[0], -0.5), max(curr_ylim[1], self.target_y + 1)])
        self.ax3d.set_zlim([min(curr_zlim[0], 0), max(curr_zlim[1], self.target_z + 1)])

    def update_plot(self):
        if not self.time_history:
            return
            
        # Actualizar gráfica en tiempo real (cada 2 mensajes recibidos para no saturar)
        if self.messages_received % 2 == 0:
            try:
                self.line3d.set_data(self.x_history, self.y_history)
                self.line3d.set_3d_properties(self.z_history)
                
                self.target_line3d.set_data(self.tx_history, self.ty_history)
                self.target_line3d.set_3d_properties(self.tz_history)
                
                self.line_x.set_data(self.time_history, self.x_history)
                self.line_y.set_data(self.time_history, self.y_history)
                self.line_z.set_data(self.time_history, self.z_history)
                
                self.target_line_x.set_data(self.time_history, self.tx_history)
                self.target_line_y.set_data(self.time_history, self.ty_history)
                self.target_line_z.set_data(self.time_history, self.tz_history)
                
                max_t = max(5.0, self.time_history[-1])
                self.ax_x.set_xlim([0, max_t])
                self.ax_y.set_xlim([0, max_t])
                self.ax_z.set_xlim([0, max_t])
                
                min_x, max_x = min(self.x_history + [0, self.target_x]), max(self.x_history + [0, self.target_x])
                self.ax_x.set_ylim([min_x - 0.5, max_x + 0.5])
                
                min_y, max_y = min(self.y_history + [0, self.target_y]), max(self.y_history + [0, self.target_y])
                self.ax_y.set_ylim([min_y - 0.5, max_y + 0.5])
                
                min_z, max_z = min(self.z_history + [0, self.target_z]), max(self.z_history + [0, self.target_z])
                self.ax_z.set_ylim([min_z - 0.5, max_z + 0.5])
                
                # ── Actualizar gráficas de ruido ──
                if len(self.jitter_time) > 1:
                    self.jline_x.set_data(self.jitter_time, self.jitter_x)
                    self.jline_y.set_data(self.jitter_time, self.jitter_y)
                    self.jline_z.set_data(self.jitter_time, self.jitter_z)
                    self.ax_jitter.set_xlim([0, max_t])
                    all_j = self.jitter_x + self.jitter_y + self.jitter_z
                    jmin, jmax = min(all_j), max(all_j)
                    margin = max(0.01, (jmax - jmin) * 0.2)
                    self.ax_jitter.set_ylim([jmin - margin, jmax + margin])
                
                if len(self.var_time) > 1:
                    self.vline_x.set_data(self.var_time, self.var_x)
                    self.vline_y.set_data(self.var_time, self.var_y)
                    self.vline_z.set_data(self.var_time, self.var_z)
                    self.ax_var.set_xlim([0, max_t])
                    all_v = self.var_x + self.var_y + self.var_z
                    vmax = max(all_v) if all_v else 0.01
                    self.ax_var.set_ylim([0, vmax * 1.3])
                
                # Actualizar estadísticas cada 10 muestras
                if len(self.jitter_x) > 10 and self.messages_received % 10 == 0:
                    jx_std = np.std(self.jitter_x)
                    jy_std = np.std(self.jitter_y)
                    jz_std = np.std(self.jitter_z)
                    x_range = max(self.x_history) - min(self.x_history) if self.x_history else 1.0
                    y_range = max(self.y_history) - min(self.y_history) if self.y_history else 1.0
                    z_range = max(self.z_history) - min(self.z_history) if self.z_history else 1.0
                    snr_x = x_range / jx_std if jx_std > 1e-9 else float('inf')
                    snr_y = y_range / jy_std if jy_std > 1e-9 else float('inf')
                    snr_z = z_range / jz_std if jz_std > 1e-9 else float('inf')
                    
                    stats = (
                        f"Std Jitter:\n"
                        f"  X: {jx_std*100:.3f} cm\n"
                        f"  Y: {jy_std*100:.3f} cm\n"
                        f"  Z: {jz_std*100:.3f} cm\n\n"
                        f"SNR (rango/σ):\n"
                        f"  X: {snr_x:.1f}\n"
                        f"  Y: {snr_y:.1f}\n"
                        f"  Z: {snr_z:.1f}\n\n"
                        f"Muestras: {len(self.x_history)}"
                    )
                    
                    if self.drift_received:
                        stats += (
                            f"\n\nViento Simulado (Drift):\n"
                            f"  Magnitud: {self.drift_magnitude:.3f} m/s\n"
                            f"  Dir X: {self.drift_x:.3f} m/s\n"
                            f"  Dir Y: {self.drift_y:.3f} m/s"
                        )
                        
                    self.stats_text.set_text(stats)
                
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
            except Exception:
                pass

    def exportar_datos(self):
        dir_path = "/ros2_ws/src/tello_control_pos/tello_control_pos/"
        if not os.path.exists(dir_path):
            os.makedirs(dir_path, exist_ok=True)
            
        # 1. Guardar CSV fijo
        csv_file = os.path.join(dir_path, "ultimo_reporte_tello.csv")
        with open(csv_file, mode='w') as f:
            writer = csv.writer(f)
            writer.writerow(["Time", "X", "Y", "Z", "Target_X", "Target_Y", "Target_Z"])
            for row in zip(self.time_history, self.x_history, self.y_history, self.z_history, self.tx_history, self.ty_history, self.tz_history): 
                writer.writerow(row)

        # 2. Guardar PNG de Posiciones X, Y, Z vs Tiempo
        plt.figure(figsize=(10, 8))
        plt.subplot(3, 1, 1)
        plt.plot(self.time_history, self.x_history, 'b-', label='X')
        plt.plot(self.time_history, self.tx_history, 'r--', label='Meta X')
        plt.title('Posición en el tiempo')
        plt.ylabel('X [m]'); plt.grid(True); plt.legend()
        
        plt.subplot(3, 1, 2)
        plt.plot(self.time_history, self.y_history, 'g-', label='Y')
        plt.plot(self.time_history, self.ty_history, 'r--', label='Meta Y')
        plt.ylabel('Y [m]'); plt.grid(True); plt.legend()
        
        plt.subplot(3, 1, 3)
        plt.plot(self.time_history, self.z_history, 'm-', label='Z')
        plt.plot(self.time_history, self.tz_history, 'r--', label='Meta Z')
        plt.xlabel('Tiempo [s]'); plt.ylabel('Z [m]'); plt.grid(True); plt.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(dir_path, "ultima_grafica_xyz_tello.png"))
        plt.close()

        # 3. Guardar PNG de Trayectoria 3D
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot([0], [0], [0], 'go', label='Inicio')
        ax.plot(self.tx_history, self.ty_history, self.tz_history, 'r--', label='Meta')
        ax.plot(self.x_history, self.y_history, self.z_history, 'b-', label='Trayecto 3D')
        ax.set_title('Trayectoria 3D - Tello')
        ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
        ax.legend()
        plt.savefig(os.path.join(dir_path, "ultima_trayectoria_3d_tello.png"))
        plt.close()

        # 4. Guardar PNG de Análisis de Ruido
        if len(self.jitter_x) > 2:
            fig, axes = plt.subplots(3, 1, figsize=(12, 10))
            
            # Jitter por eje
            axes[0].plot(self.jitter_time, self.jitter_x, 'b-', alpha=0.6, linewidth=0.8, label='ΔX')
            axes[0].plot(self.jitter_time, self.jitter_y, 'g-', alpha=0.6, linewidth=0.8, label='ΔY')
            axes[0].plot(self.jitter_time, self.jitter_z, 'm-', alpha=0.6, linewidth=0.8, label='ΔZ')
            axes[0].set_title('Jitter de Odometría (cambio entre muestras)')
            axes[0].set_ylabel('Δ posición [m]'); axes[0].grid(True); axes[0].legend()
            axes[0].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
            
            # Varianza en ventana deslizante
            if len(self.var_time) > 1:
                axes[1].plot(self.var_time, self.var_x, 'b-', label='σ² X')
                axes[1].plot(self.var_time, self.var_y, 'g-', label='σ² Y')
                axes[1].plot(self.var_time, self.var_z, 'm-', label='σ² Z')
            axes[1].set_title(f'Varianza en Ventana Deslizante ({self.WINDOW_SIZE} muestras)')
            axes[1].set_ylabel('Varianza [m²]'); axes[1].set_xlabel('Tiempo [s]')
            axes[1].grid(True); axes[1].legend()
            
            # Histograma de jitter
            axes[2].hist(self.jitter_x, bins=50, alpha=0.5, color='b', label=f'X (σ={np.std(self.jitter_x)*100:.2f}cm)')
            axes[2].hist(self.jitter_y, bins=50, alpha=0.5, color='g', label=f'Y (σ={np.std(self.jitter_y)*100:.2f}cm)')
            axes[2].hist(self.jitter_z, bins=50, alpha=0.5, color='m', label=f'Z (σ={np.std(self.jitter_z)*100:.2f}cm)')
            axes[2].set_title('Distribución del Jitter')
            axes[2].set_xlabel('Δ posición [m]'); axes[2].set_ylabel('Frecuencia')
            axes[2].grid(True); axes[2].legend()
            
            plt.tight_layout()
            plt.savefig(os.path.join(dir_path, "analisis_ruido_odom.png"), dpi=150)
            plt.close()
        
        # 5. Guardar CSV de ruido
        if len(self.jitter_x) > 0:
            csv_noise = os.path.join(dir_path, "ruido_odometria.csv")
            with open(csv_noise, mode='w') as f:
                writer = csv.writer(f)
                writer.writerow(["Time", "Jitter_X", "Jitter_Y", "Jitter_Z"])
                for row in zip(self.jitter_time, self.jitter_x, self.jitter_y, self.jitter_z):
                    writer.writerow(row)
            
            self.get_logger().info(
                f"\n{'='*50}\n"
                f"  REPORTE DE RUIDO DE ODOMETRÍA\n"
                f"{'='*50}\n"
                f"  Std Jitter X: {np.std(self.jitter_x)*100:.3f} cm\n"
                f"  Std Jitter Y: {np.std(self.jitter_y)*100:.3f} cm\n"
                f"  Std Jitter Z: {np.std(self.jitter_z)*100:.3f} cm\n"
                f"  Max Jitter X: {max(abs(j) for j in self.jitter_x)*100:.3f} cm\n"
                f"  Max Jitter Y: {max(abs(j) for j in self.jitter_y)*100:.3f} cm\n"
                f"  Max Jitter Z: {max(abs(j) for j in self.jitter_z)*100:.3f} cm\n"
                f"{'='*50}"
            )
        
        self.get_logger().info(f"Reportes de gráficas actualizados en: {dir_path}")

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TelloPlotter()
    
    # Flag para garantizar que la exportación se ejecute exactamente una vez
    exported = False
    
    def shutdown_handler(sig, frame):
        nonlocal exported
        if not exported:
            exported = True
            node.get_logger().info("Señal recibida. Generando archivos finales...")
            try:
                node.exportar_datos()
            except Exception as e:
                node.get_logger().error(f"Error al exportar: {e}")
        raise SystemExit(0)
    
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        if not exported:
            exported = True
            node.get_logger().info("Deteniendo graficador y generando archivos finales...")
            try:
                node.exportar_datos()
            except Exception as e:
                node.get_logger().error(f"Error al exportar: {e}")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
