"""
Módulo LineFollower para o robô OBR 2026.
Re-exporta e fornece a interface do seguidor de linha e visão computacional.
"""

import time
import numpy as np
import cv2

from line_cam import LineFollowerVision, capturar_e_processar, inicializar_camera, capturar_frame
import mp_manager as mgr
from constants import (
    FRAME_WIDTH, FRAME_HEIGHT, CENTER_X, CENTER_Y,
    LINE_LOST, LINE_TRACKING, LINE_GAP, RED_STOP,
    GREEN_NONE, GREEN_LEFT, GREEN_RIGHT, GREEN_DOUBLE, GREEN_APPROACH
)


class LineFollower:
    """Classe de alto nível do seguidor de linha para integração e testes diretos."""

    def __init__(self, width=FRAME_WIDTH, height=FRAME_HEIGHT):
        self.vision = LineFollowerVision(width=width, height=height)
        self.width = width
        self.height = height

    def process(self, frame):
        """Processa um frame e retorna os dados de percepção."""
        return self.vision.process_frame(frame)


__all__ = [
    "LineFollower",
    "LineFollowerVision",
    "capturar_e_processar",
    "inicializar_camera",
    "capturar_frame",
]