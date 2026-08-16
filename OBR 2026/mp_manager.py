"""
Gerenciador de variáveis e estados compartilhados entre processos (multiprocessing).
"""

import multiprocessing as mp
from constants import (
    FRAME_WIDTH, FRAME_HEIGHT, CENTER_X,
    LINE_LOST, GREEN_NONE
)

# Sincronização e controle de encerramento
terminate = mp.Event()

# Status da câmera
camera_ok = mp.Value('b', False)

# Status de percepção da linha
line_status = mp.Value('i', LINE_LOST)

# Coordenadas do ponto central da linha e desvios
center_x = mp.Value('d', float(CENTER_X))
center_y = mp.Value('d', float(FRAME_HEIGHT))
error_x = mp.Value('d', 0.0)          # Normalizado ou em pixels (center_x - CENTER_X)

# Direção angular da linha
line_angle = mp.Value('d', 90.0)      # 90° = reta alinhada com o robô
error_angle = mp.Value('d', 0.0)     # line_angle - 90.0

# Marcadores de cores
green_signal = mp.Value('i', GREEN_NONE)
red_detected = mp.Value('b', False)

# Diagnóstico
fps = mp.Value('d', 0.0)


def reset_estado():
    """Restaura os valores padrão de percepção."""
    camera_ok.value = False
    line_status.value = LINE_LOST
    center_x.value = float(CENTER_X)
    center_y.value = float(FRAME_HEIGHT)
    error_x.value = 0.0
    line_angle.value = 90.0
    error_angle.value = 0.0
    green_signal.value = GREEN_NONE
    red_detected.value = False
    fps.value = 0.0
