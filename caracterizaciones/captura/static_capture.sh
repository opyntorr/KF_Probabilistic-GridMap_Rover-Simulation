#!/usr/bin/env bash
# static_capture.sh — controla la grabacion estatica (IMU+LiDAR) en el Orin, desde la laptop.
#
#   ~/static_capture.sh start <nombre> [horas]   # lanza la grabacion DETACHED en el Orin
#   ~/static_capture.sh stop                      # la para LIMPIO (SIGINT -> cierra el bag)
#   ~/static_capture.sh status                    # ve si esta grabando + tamano de los bags
#   ~/static_capture.sh fetch <nombre>            # trae el bag del Orin a la laptop (~/static_runs)
#
# Precondiciones (fisicas): A1 desconectado, MS200 girando, robot QUIETO, escena estatica.
set -u
ORIN="${ORIN_HOST:-jetson@10.42.1.1}"
SSHC=(ssh -o ConnectTimeout=12 -o StrictHostKeyChecking=no "$ORIN")
CMD="${1:-}"

case "$CMD" in
  start)
    NAME="${2:-static_$(date +%Y%m%d_%H%M%S)}"; HOURS="${3:-12}"
    "${SSHC[@]}" "test -f ~/orin_record_static.sh" || { echo "ERROR: falta ~/orin_record_static.sh en el Orin (despliegalo primero)"; exit 1; }
    if "${SSHC[@]}" "pgrep -f '[r]os2 bag record' >/dev/null"; then
      echo "ERROR: ya hay una grabacion corriendo. Para con: $0 stop"; exit 1; fi
    echo "Lanzando grabacion '$NAME' (tope ${HOURS}h) en el Orin, detached..."
    "${SSHC[@]}" "mkdir -p ~/static_runs"
    "${SSHC[@]}" "setsid bash ~/orin_record_static.sh '$NAME' '$HOURS' > ~/static_runs/$NAME.log 2>&1 < /dev/null & disown; sleep 1; echo lanzado"
    sleep 4
    echo "--- log inicial ---"
    "${SSHC[@]}" "tail -4 ~/static_runs/$NAME.log 2>/dev/null; echo '--- proceso ---'; pgrep -af '[r]os2 bag record' | head -1"
    echo
    echo "Puedes cerrar la laptop. En la manana:  $0 stop   y luego:  $0 fetch $NAME"
    ;;
  stop)
    echo "Parando grabacion (SIGINT, cierre limpio del bag)..."
    "${SSHC[@]}" "pkill -INT -f '[r]os2 bag record' && sleep 4 && echo 'parado OK' || echo 'no habia grabacion activa'"
    ;;
  status)
    "${SSHC[@]}" "echo '--- grabacion activa? ---'; pgrep -af '[r]os2 bag record' | head -1 || echo '(ninguna)'; echo '--- bags en ~/static_runs ---'; du -sh ~/static_runs/*/ 2>/dev/null | tail -8; echo '--- disco ---'; df -h /home | tail -1"
    ;;
  fetch)
    NAME="${2:?uso: $0 fetch <nombre>}"
    mkdir -p "$HOME/static_runs"
    echo "Trayendo ~/static_runs/$NAME del Orin a la laptop..."
    scp -r -o ConnectTimeout=20 "$ORIN:static_runs/$NAME" "$HOME/static_runs/" && echo "OK -> ~/static_runs/$NAME"
    ;;
  *)
    echo "uso: $0 {start <nombre> [horas] | stop | status | fetch <nombre>}"
    ;;
esac
