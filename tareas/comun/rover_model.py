"""
Modelo cinematico del Robot Diferencial Generalizado.

Referencia: D. Diaz, R. Kelly, "On Modeling and Position Tracking Control of the
Generalized Differential Driven Wheeled Mobile Robot", IEEE 2016.

Se usa el "Modelo 2" (entradas = velocidades angulares de las ruedas) porque es
el que admite la ley de control por inversion de matriz (ec. 33 del paper).

    xi = [x, y, theta]^T   (postura del punto de interes p)
    u  = [w_r, w_l]^T      (velocidad angular rueda derecha / izquierda)

    xi_dot = [ D(theta) ; phi^T ] u            (ec. 27)

con:
    D(theta) = [ r/2 cos - k sin ,  r/2 cos + k sin ;
                 r/2 sin + k cos ,  r/2 sin - k cos ]     k = h*r/d   (ec. 28)
    phi^T    = [ r/d , -r/d ]                              (ec. 29)

D(theta) es no singular siempre que h != 0 (det = -h r^2 / d), lo que permite
controlar el punto p sin las restricciones del modelo clasico (h = 0).

Este modulo es COMPARTIDO por las dos tareas (filtro de Kalman y mapa de
ocupacion), de modo que ambas viven en el mismo entorno.
"""

import numpy as np


class RoverParams:
    """Parametros geometricos del rover (en metros). Valores del paper."""

    def __init__(self, r=0.04, h=0.30, d=0.10):
        self.r = r  # radio de rueda            [m]
        self.h = h  # offset del punto p (h!=0) [m]
        self.d = d  # separacion entre ruedas   [m]


def D_matrix(theta, p):
    """Matriz D(theta) (2x2), ec. 28."""
    r, h, d = p.r, p.h, p.d
    k = h * r / d
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [r / 2 * c - k * s, r / 2 * c + k * s],
        [r / 2 * s + k * c, r / 2 * s - k * c],
    ])


def phi_vec(p):
    """Vector phi (ec. 29): theta_dot = phi^T u."""
    return np.array([p.r / p.d, -p.r / p.d])


def f_kinematics(xi, u, p):
    """Campo vectorial xi_dot = f(xi, u) del Modelo 2 (ec. 27)."""
    theta = xi[2]
    q_dot = D_matrix(theta, p) @ u
    th_dot = phi_vec(p) @ u
    return np.array([q_dot[0], q_dot[1], th_dot])


def jacobian_f(xi, u, p):
    """
    Jacobiano F = d f / d xi  (3x3), necesario para linealizar el filtro de
    Kalman continuo (forma de Kalman-Bucy extendida). f solo depende de theta
    y de u, por lo que las columnas de x e y son cero.
    """
    theta = xi[2]
    r, h, d = p.r, p.h, p.d
    k = h * r / d
    c, s = np.cos(theta), np.sin(theta)
    wr, wl = u
    df1_dth = (-r / 2 * s - k * c) * wr + (-r / 2 * s + k * c) * wl
    df2_dth = (r / 2 * c - k * s) * wr + (r / 2 * c + k * s) * wl
    F = np.zeros((3, 3))
    F[0, 2] = df1_dth
    F[1, 2] = df2_dth
    return F


def controller(xi_hat, qd, qd_dot, Kp, p):
    """
    Ley de control de seguimiento de posicion (ec. 33):

        u = D^-1(theta) [ qd_dot + Kp (qd - q) ]

    IMPORTANTE: se evalua con el estado ESTIMADO xi_hat (salida del filtro de
    Kalman), no con el estado real. En lazo cerrado ideal se obtiene
    q_tilde_dot = -Kp q_tilde, asintoticamente estable.
    """
    theta = xi_hat[2]
    q = xi_hat[0:2]
    D = D_matrix(theta, p)
    q_tilde = qd - q
    return np.linalg.solve(D, qd_dot + Kp @ q_tilde)


# --------------------------------------------------------------------------- #
# Variante en TWIST (v_a, omega) — util cuando se comanda el chasis con un Twist
# (p.ej. el JetAuto en Gazebo). Equivale a Model 1 (ec. 7): q_dot_p = A(theta)[v,w].
# --------------------------------------------------------------------------- #
def point_p(xi, h):
    """Postura del punto de interes p (offset h) desde el estado del chasis."""
    x, y, th = xi
    return np.array([x + h * np.cos(th), y + h * np.sin(th)])


def A_matrix(theta, h):
    """A(theta) tal que q_dot_p = A [v_a, omega]^T ; det A = h (invertible si h!=0)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -h * s],
                     [s,  h * c]])


def controller_twist(q_p, theta, qd, qd_dot, Kp, h):
    """Ley de control (ec. 33) en twist: devuelve (v_a, omega) con el estado estimado."""
    vw = np.linalg.solve(A_matrix(theta, h), qd_dot + Kp @ (qd - q_p))
    return float(vw[0]), float(vw[1])


def f_unicycle(xi, v, w):
    """Modelo del chasis (punto a) accionado por el twist (v, w): xi_dot."""
    th = xi[2]
    return np.array([v * np.cos(th), v * np.sin(th), w])


def jacobian_unicycle(xi, v, w):
    """F = d f_unicycle / d xi (3x3)."""
    th = xi[2]
    F = np.zeros((3, 3))
    F[0, 2] = -v * np.sin(th)
    F[1, 2] = v * np.cos(th)
    return F
