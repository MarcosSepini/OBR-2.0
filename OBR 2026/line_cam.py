"""
Módulo de captura de câmera e visão computacional (Line Follower) para OBR 2026.
Processa o feed de vídeo, detecta a linha preta (ponto central e direção/ângulo),
marcadores verdes (esquerda, direita, duplo verde/180) e fita vermelha de parada.
"""

import time
import numpy as np
import cv2

import mp_manager as mgr
from constants import (
    FRAME_WIDTH, FRAME_HEIGHT, CENTER_X, CENTER_Y,
    LINE_LOST, LINE_TRACKING, LINE_GAP, RED_STOP,
    GREEN_NONE, GREEN_LEFT, GREEN_RIGHT, GREEN_DOUBLE, GREEN_APPROACH,
    GREEN_STR_TO_INT,
    LIMIAR_PRETO_BASE, LIMIAR_PRETO_BRILHO,
    HSV_VERDE_MIN, HSV_VERDE_MAX,
    HSV_VERMELHO_MIN1, HSV_VERMELHO_MAX1, HSV_VERMELHO_MIN2, HSV_VERMELHO_MAX2,
    FRACAO_MIN_VERMELHO,
    AREA_MIN_LINHA_PRETA, AREA_MIN_MARCADOR_VERDE, LIMIAR_ADJACENCIA_VERDE,
    BLUR_KSIZE_PRETO, MORPH_KSIZE, MORPH_OPEN_ITERS, MORPH_CLOSE_ITERS
)


class LineFollowerVision:
    """Processador de visão computacional para detecção de linha e marcadores."""

    def __init__(self, width=FRAME_WIDTH, height=FRAME_HEIGHT):
        self.width = width
        self.height = height

        # Limiares de área escalonados para a resolução
        scale = (width * height) / (320 * 240)
        self.min_black_area = max(200, int(AREA_MIN_LINHA_PRETA * scale))
        self.min_green_area = max(150, int(AREA_MIN_MARCADOR_VERDE * scale))
        self.limiar_adjacencia = LIMIAR_ADJACENCIA_VERDE

        # Limiarização Preto / Iluminação
        self.base_black = LIMIAR_PRETO_BASE
        self.bright_black = LIMIAR_PRETO_BRILHO
        self.black_blur_ksize = BLUR_KSIZE_PRETO
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, MORPH_KSIZE)
        self.morph_open_iterations = MORPH_OPEN_ITERS
        self.morph_close_iterations = MORPH_CLOSE_ITERS

        # Limiares de cor Verde (HSV)
        self.green_hsv_lower = np.array(HSV_VERDE_MIN, dtype=np.uint8)
        self.green_hsv_upper = np.array(HSV_VERDE_MAX, dtype=np.uint8)

        # Limiares de cor Vermelha (HSV)
        self.red_hsv_lower_1 = np.array(HSV_VERMELHO_MIN1, dtype=np.uint8)
        self.red_hsv_upper_1 = np.array(HSV_VERMELHO_MAX1, dtype=np.uint8)
        self.red_hsv_lower_2 = np.array(HSV_VERMELHO_MIN2, dtype=np.uint8)
        self.red_hsv_upper_2 = np.array(HSV_VERMELHO_MAX2, dtype=np.uint8)
        self.red_min_fraction = FRACAO_MIN_VERMELHO

        # Máscaras de brilho nas bordas para compensação de reflexos
        self._init_bright_masks()

        # Estados internos
        self.last_angle = 90.0
        self.angle = 90.0
        self.point_angle = 90.0
        self.box_angle = 90.0
        self.combined_angle = 90.0
        self.prev_side = None
        self.edge_following = None

        self.green_signal = "NONE"
        self.prev_green_signal = "NONE"
        self.last_seen_green = 0.0
        self.green_action_cooldown_until = 0.0

        self.image = None
        self.gray_image = None
        self.hsv_image = None
        self.black_mask = None
        self.black_contour = None
        self.green_mask = None
        self.green_contours = []
        self.red_detected = False

    def _init_bright_masks(self):
        """Cria máscaras para regiões periféricas onde há mais reflexo de luz."""
        h, w = self.height, self.width
        self.bright_mask = np.zeros((h, w), dtype=np.uint8)

        pts_top = np.array([[0, 0], [w, 0], [int(w * 0.8), int(h * 0.25)], [int(w * 0.2), int(h * 0.25)]])
        pts_bottom = np.array([[int(w * 0.2), int(h * 0.75)], [int(w * 0.8), int(h * 0.75)], [w, h], [0, h]])
        pts_left = np.array([[0, 0], [int(w * 0.15), int(h * 0.2)], [int(w * 0.15), int(h * 0.8)], [0, h]])
        pts_right = np.array([[w, 0], [w, h], [int(w * 0.85), int(h * 0.8)], [int(w * 0.85), int(h * 0.2)]])

        cv2.fillPoly(self.bright_mask, [pts_top, pts_bottom, pts_left, pts_right], 255)
        self.inv_bright_mask = cv2.bitwise_not(self.bright_mask)

    def process_frame(self, frame):
        """
        Processa um frame BGR e retorna (status, center_pt, angle, error_x, error_angle, green_sig, red_flag).
        """
        if frame is None:
            return LINE_LOST, (CENTER_X, self.height), 90.0, 0.0, 0.0, "NONE", False

        self.image = frame
        self.green_mask = None
        self.green_contours = []

        # 1. Segmentação de cores
        self.find_green()
        self.find_black()
        self.find_red()

        # 2. Verificação de linha preta
        if self.black_contour is not None:
            self.green_check()
            ref_point, angle = self.calculate_angle_and_center(self.black_contour)
            status = LINE_TRACKING

            error_x = float(ref_point[0] - CENTER_X)
            error_angle = float(angle - 90.0)

            return status, ref_point, angle, error_x, error_angle, self.green_signal, self.red_detected
        else:
            self.green_signal = "NONE"
            ref_point = (CENTER_X, self.height)
            status = RED_STOP if self.red_detected else LINE_GAP
            return status, ref_point, 90.0, 0.0, 0.0, "NONE", self.red_detected

    def find_green(self):
        """Detecta regiões verdes (marcadores de curva)."""
        self.hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(self.hsv_image, self.green_hsv_lower, self.green_hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel, iterations=self.morph_open_iterations)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel, iterations=self.morph_close_iterations)
        self.green_mask = mask

        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.green_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_green_area]

    def find_black(self):
        """Segmenta a fita preta, subtraindo áreas verdes e aplicando compensação de luz."""
        self.black_contour = None
        self.black_mask = None

        if self.image is None:
            return

        self.gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        _, mask_base = cv2.threshold(self.gray_image, self.base_black, 255, cv2.THRESH_BINARY_INV)
        _, mask_bright = cv2.threshold(self.gray_image, self.bright_black, 255, cv2.THRESH_BINARY_INV)

        mask = cv2.bitwise_or(
            cv2.bitwise_and(mask_base, self.inv_bright_mask),
            cv2.bitwise_and(mask_bright, self.bright_mask)
        )

        # Remove verde para não poluir o contorno preto
        if self.green_mask is not None:
            mask = cv2.bitwise_and(mask, cv2.bitwise_not(self.green_mask))

        mask = cv2.medianBlur(mask, self.black_blur_ksize)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel, iterations=self.morph_open_iterations)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel, iterations=self.morph_close_iterations)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_black_area]

        # Prioriza contornos que tocam ou estão na metade inferior da tela (próximo ao robô)
        valid_contours = [c for c in valid_contours if any(p[0][1] > int(self.height * 0.4) for p in c)]

        if not valid_contours:
            return

        self.black_contour = self._pick_black_contour(valid_contours)
        if self.black_contour is not None:
            contour_mask = np.zeros_like(self.gray_image, dtype=np.uint8)
            cv2.drawContours(contour_mask, [self.black_contour], -1, 255, thickness=cv2.FILLED)
            self.black_mask = contour_mask

    def _pick_black_contour(self, contours):
        """Seleciona o contorno mais provável da linha que o robô está seguindo."""
        if not contours:
            return None
        if len(contours) == 1:
            return contours[0]

        bottom_y = self.height - max(4, int(self.height * 0.08))
        x0 = int(self.width * 0.15)
        x1 = int(self.width * 0.85)

        # Candidatos que tocam a base central
        bottom_hit = [c for c in contours if any((p[0][1] >= bottom_y) and (x0 <= p[0][0] <= x1) for p in c)]
        candidates = bottom_hit if bottom_hit else contours

        # Retorna o de maior área entre os candidatos válidos
        return max(candidates, key=cv2.contourArea)

    def find_red(self):
        """Detecta faixa vermelha de fim de pista."""
        if self.image is None:
            self.red_detected = False
            return

        if self.hsv_image is None:
            self.hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(self.hsv_image, self.red_hsv_lower_1, self.red_hsv_upper_1)
        mask2 = cv2.inRange(self.hsv_image, self.red_hsv_lower_2, self.red_hsv_upper_2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        red_pixels = cv2.countNonZero(red_mask)
        threshold = int(self.red_min_fraction * self.width * self.height)
        self.red_detected = red_pixels > threshold

    def green_check(self):
        """Valida e classifica os marcadores verdes em relação à linha preta."""
        self.green_signal = "NONE"

        if not self.green_contours:
            self._green_hold()
            return

        valid_boxes = []
        y_avgs = []

        for contour in self.green_contours:
            valid_box, y_avg = self.validate_green_contour(contour)
            if valid_box is not None:
                valid_boxes.append(valid_box)
                y_avgs.append(y_avg)

        if len(valid_boxes) >= 2:
            self.green_signal = "DOUBLE"
        elif len(valid_boxes) == 1:
            valid_box = valid_boxes[0]
            y_avg = y_avgs[0]

            if y_avg < self.height // 3:
                self.green_signal = "APPROACH"
            else:
                left_points = sorted(valid_box, key=lambda p: p[0])[:2]
                left_x = int(sum(p[0] for p in left_points) / 2)
                left_y = int(sum(p[1] for p in left_points) / 2)

                if self.black_check_box((left_x, left_y), threshold=0.20, anchor="right"):
                    self.green_signal = "RIGHT"
                else:
                    right_points = sorted(valid_box, key=lambda p: p[0])[-2:]
                    right_x = int(sum(p[0] for p in right_points) / 2)
                    right_y = int(sum(p[1] for p in right_points) / 2)

                    if self.black_check_box((right_x, right_y), threshold=0.20, anchor="left"):
                        self.green_signal = "LEFT"

        self._green_hold()
        self.prev_green_signal = self.green_signal

    def _green_hold(self):
        """Mantém a persistência do sinal verde para evitar oscilações por jitter."""
        if self.green_signal not in ("NONE", "DOUBLE", "APPROACH") and self.prev_green_signal != self.green_signal:
            self.last_seen_green = time.perf_counter()
        elif self.green_signal == "DOUBLE":
            self.last_seen_green = 0.0

        if (time.perf_counter() - self.last_seen_green) < 0.4 and self.prev_green_signal not in ("NONE", "DOUBLE", "APPROACH"):
            self.green_signal = self.prev_green_signal

    def validate_green_contour(self, contour):
        """Verifica se o contorno verde é adjacente à linha preta."""
        if contour is None or len(contour) < 4 or self.black_mask is None:
            return None, None

        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect).astype(np.int32)
        points = [tuple(pt) for pt in box]

        top_points = sorted(points, key=lambda p: p[1])[:2]
        x_avg = int(sum(p[0] for p in top_points) / 2)
        y_avg = int(sum(p[1] for p in top_points) / 2)

        # Checa se há linha preta adjacente ao bloco verde (acima ou nas laterais)
        toca_cima = self.black_check_box((x_avg, y_avg), threshold=self.limiar_adjacencia, anchor="top")
        toca_esq = self.black_check_box((x_avg, y_avg), threshold=self.limiar_adjacencia, anchor="right")   # olha para a esquerda
        toca_dir = self.black_check_box((x_avg, y_avg), threshold=self.limiar_adjacencia, anchor="left")    # olha para a direita

        if not (toca_cima or toca_esq or toca_dir):
            return None, None

        return points, y_avg

    def black_check_box(self, check_point, threshold=None, anchor="center"):
        """Checa a presença de pixels pretos em uma janela vizinha ao ponto de teste."""
        if self.black_mask is None:
            return False

        if threshold is None:
            threshold = self.limiar_adjacencia

        half_size = max(5, int(self.width / 20))
        x, y = int(check_point[0]), int(check_point[1])

        if not (0 <= x < self.width and 0 <= y < self.height):
            return False

        if anchor == "top":
            x_start, x_end = x - (2 * half_size), x + (2 * half_size) + 1
            y_start, y_end = y - (2 * half_size), y + 1
        elif anchor == "left":
            # Ponto à esquerda do teste -> olha para a direita
            x_start, x_end = x, x + (2 * half_size) + 1
            y_start, y_end = y - half_size, y + half_size + 1
        elif anchor == "right":
            # Ponto à direita do teste -> olha para a esquerda
            x_start, x_end = x - (2 * half_size), x + 1
            y_start, y_end = y - half_size, y + half_size + 1
        else:
            x_start, x_end = x - half_size, x + half_size + 1
            y_start, y_end = y - half_size, y + half_size + 1

        x_start = max(0, x_start)
        x_end = min(self.width, x_end)
        y_start = max(0, y_start)
        y_end = min(self.height, y_end)

        if (x_end - x_start) <= 1 or (y_end - y_start) <= 1:
            return False

        region = self.black_mask[y_start:y_end, x_start:x_end]
        if region.size == 0:
            return False

        black_pixels = cv2.countNonZero(region)
        return (black_pixels / region.size) >= threshold

    def calculate_angle_and_center(self, contour):
        """
        Calcula o ponto central de referência (ref_point) e a direção angular (angle)
        do vetor entre a base central do robô e o ponto de interesse da linha.
        """
        if contour is None:
            return (CENTER_X, self.height), 90.0

        ref_point = self.calculate_top_contour(contour)

        # Se houver marcador verde ativo (LEFT ou RIGHT), ajusta o ponto para direcionar a curva
        if self.green_signal in ("LEFT", "RIGHT"):
            top_half_points = [p for p in contour if p[0][1] < (self.height // 2)]
            candidate_points = top_half_points if top_half_points else contour
            extreme_point = min(candidate_points, key=lambda p: p[0][0]) if self.green_signal == "LEFT" else max(candidate_points, key=lambda p: p[0][0])

            shift_pixels = int(self.width * 0.1)
            if self.green_signal == "LEFT":
                x = min(extreme_point[0][0] + shift_pixels, CENTER_X)
            else:
                x = max(extreme_point[0][0] - shift_pixels, CENTER_X)
            ref_point = (x, 0)
            angle = self._angle_from_ref_point(ref_point)
            return ref_point, angle

        # Checa pontos de borda se a linha sair pelas laterais (curva acentuada)
        left_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] <= self.width // 16]
        right_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] >= (15 * self.width // 16)]

        ref_threshold = 8
        if ref_point[1] >= ref_threshold and self.green_signal != "APPROACH" and (left_edge_points or right_edge_points):
            y_left = min(p[1] for p in left_edge_points) if left_edge_points else None
            y_right = min(p[1] for p in right_edge_points) if right_edge_points else None
            prev_side = self.prev_side

            if prev_side is None:
                if y_left is None and y_right is not None:
                    prev_side = "RIGHT"
                elif y_right is None and y_left is not None:
                    prev_side = "LEFT"
                elif y_left is not None and y_right is not None:
                    prev_side = "LEFT" if y_left < y_right else "RIGHT"

            if y_left is not None and (y_right is None or prev_side == "LEFT"):
                ref_point = (0, y_left)
                self.prev_side = "LEFT"
            elif y_right is not None:
                ref_point = (self.width - 1, y_right)
                self.prev_side = "RIGHT"
        else:
            self.prev_side = None

        angle = self._angle_from_ref_point(ref_point)
        self.angle = angle
        self.last_angle = angle
        return ref_point, angle

    def calculate_top_contour(self, contour):
        """Encontra o centróide da fatia superior do contorno da linha."""
        if contour is None:
            return (CENTER_X, self.height)

        h, w = self.height, self.width
        pts = contour[:, 0, :]

        min_y = 5
        max_y = min(h, min_y + max(10, int(h / 16)))

        x, y, bw, bh = cv2.boundingRect(contour)
        x0, y0 = max(0, x), max(0, y)
        x1, y1 = min(w, x + bw), min(h, y + bh)

        band_y0 = max(min_y, y0)
        band_y1 = min(max_y, y1)

        if band_y0 >= band_y1:
            return (CENTER_X, h)

        local_w = x1 - x0
        local_h = y1 - y0
        local_mask = np.zeros((local_h, local_w), dtype=np.uint8)

        shifted = pts.copy()
        shifted[:, 0] -= x0
        shifted[:, 1] -= y0
        shifted = shifted.reshape((-1, 1, 2))

        cv2.drawContours(local_mask, [shifted], -1, 255, thickness=cv2.FILLED)
        band = local_mask[band_y0 - y0: band_y1 - y0, :]

        nz = cv2.findNonZero(band)
        if nz is None:
            return (CENTER_X, h)

        band_pts = nz[:, 0, :].astype(np.int32)
        band_pts[:, 0] += x0
        band_pts[:, 1] += band_y0

        x_avg = int(np.mean(band_pts[:, 0]))
        y_avg = int(np.mean(band_pts[:, 1]))
        return (x_avg, y_avg)

    def _angle_from_ref_point(self, ref_point):
        """Calcula o ângulo do vetor da base central do robô até o ponto de referência."""
        bottom_center = (self.width // 2, self.height)
        dx = bottom_center[0] - ref_point[0]
        dy = bottom_center[1] - ref_point[1]

        angle = float(np.degrees(np.arctan2(dy, dx)))
        if angle < 0:
            angle += 180.0
        return angle


def inicializar_camera(width=FRAME_WIDTH, height=FRAME_HEIGHT):
    """
    Inicializa a câmera tentando primeiro Picamera2 (Raspberry Pi OS)
    e com fallback para OpenCV VideoCapture.
    """
    # 1. Tenta Picamera2
    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (width, height)}
        )
        picam2.configure(config)
        picam2.start()
        print("[line_cam] Picamera2 inicializada com sucesso.")
        return ("picamera2", picam2)
    except Exception as e:
        print(f"[line_cam] Picamera2 não disponível ({e}), tentando OpenCV VideoCapture...")

    # 2. Fallback OpenCV
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 60)

    if cap.isOpened():
        print("[line_cam] OpenCV VideoCapture(0) inicializado com sucesso.")
        return ("opencv", cap)

    print("[line_cam] AVISO: Nenhuma câmera física encontrada. Entrando em modo sintético.")
    return ("none", None)


def capturar_frame(tipo_cam, cam_obj, width=FRAME_WIDTH, height=FRAME_HEIGHT):
    """Captura um frame BGR da câmera configurada."""
    if tipo_cam == "picamera2":
        # Picamera2 entrega RGB, converte para BGR para OpenCV
        rgb = cam_obj.capture_array()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if rgb is not None else None
    elif tipo_cam == "opencv":
        ret, frame = cam_obj.read()
        return frame if ret else None
    else:
        # Frame mock para ambiente sem câmera
        time.sleep(0.03)
        mock = np.ones((height, width, 3), dtype=np.uint8) * 255
        cv2.line(mock, (width // 2, height), (width // 2, 0), (0, 0, 0), 15)
        return mock


def capturar_e_processar():
    """
    Loop principal do processo de visão computacional.
    Lê a câmera, processa a linha e atualiza as variáveis compartilhadas em mp_manager.
    """
    print("[line_cam] Processo de câmera iniciado.")
    tipo_cam, cam_obj = inicializar_camera(FRAME_WIDTH, FRAME_HEIGHT)
    vision = LineFollowerVision(FRAME_WIDTH, FRAME_HEIGHT)

    mgr.camera_ok.value = True
    quadros = 0
    t_inicio = time.perf_counter()

    try:
        while not mgr.terminate.is_set():
            frame = capturar_frame(tipo_cam, cam_obj, FRAME_WIDTH, FRAME_HEIGHT)
            if frame is None:
                mgr.camera_ok.value = False
                time.sleep(0.01)
                continue

            mgr.camera_ok.value = True

            # Processamento de visão
            status, center_pt, angle, err_x, err_ang, green_sig, red_flag = vision.process_frame(frame)

            # Atualização dos dados compartilhados para o control.py
            mgr.line_status.value = status
            mgr.center_x.value = float(center_pt[0])
            mgr.center_y.value = float(center_pt[1])
            mgr.error_x.value = float(err_x)
            mgr.line_angle.value = float(angle)
            mgr.error_angle.value = float(err_ang)
            mgr.green_signal.value = GREEN_STR_TO_INT.get(green_sig, GREEN_NONE)
            mgr.red_detected.value = bool(red_flag)

            quadros += 1
            if quadros % 60 == 0:
                elapsed = time.perf_counter() - t_inicio
                mgr.fps.value = quadros / elapsed if elapsed > 0 else 0.0

    except KeyboardInterrupt:
        print("[line_cam] Interrompido pelo usuário.")
    except Exception as e:
        print(f"[line_cam] ERRO inesperado: {e}")
    finally:
        mgr.camera_ok.value = False
        if tipo_cam == "picamera2" and cam_obj is not None:
            try:
                cam_obj.stop()
            except Exception:
                pass
        elif tipo_cam == "opencv" and cam_obj is not None:
            cam_obj.release()
        print("[line_cam] Processo de câmera finalizado com sucesso.")


if __name__ == "__main__":
    # Teste isolado do módulo de câmera
    capturar_e_processar()
