import time

import cv2
import numpy as np

import mp_manager as mgr
from constants import FRAME_WIDTH, FRAME_HEIGHT, LINE_LOST, LINE_FOUND

# ==================================================================================================================================================================================================================
# CALIBRAÇÃO -- ajuste tudo isso na prática, olhando a linha/pista real.
# Nada aqui é "mágico", são só chutes iniciais coerentes com FRAME_WIDTH/
# FRAME_HEIGHT (320x200) definidos em constants.py.
# ==================================================================================================================================================================================================================

CENTER_X = FRAME_WIDTH // 2

# --- Preto (linha) ---
LIMIAR_PRETO = 90          # abaixo disso em escala de cinza (0-255) é "linha"
AREA_MIN_LINHA = 500       # pixels pretos mínimos pra considerar a linha válida

# ROI principal de seguimento: faixa horizontal perto da base da imagem.
# É dela que sai o "erro" (mgr.line_angle) usado no controle P dos motores.
Y0_ROI_LINHA, Y1_ROI_LINHA = int(FRAME_HEIGHT * 0.65), FRAME_HEIGHT

# ROI de retorno (canto inferior esquerdo) -- usado só depois da virada
# cega (executar_virada, em control.py) pra saber se a linha foi
# reencontrada (mgr.retorno_linha_ok).
X0_ROI_RETORNO, X1_ROI_RETORNO = 0, int(FRAME_WIDTH * 0.25)
Y0_ROI_RETORNO, Y1_ROI_RETORNO = int(FRAME_HEIGHT * 0.75), FRAME_HEIGHT
AREA_MIN_RETORNO = 200

# ROI_TOPO_CENTRO (topo, meio) -- usado pra saber quando o giro de erro
# grande (executar_correcao_erro_grande, em control.py) pode parar. Não
# basta "aparecer" preto na borda de baixo do ROI: o ponto mais alto
# achado (menor y) precisa ter chegado pelo menos no meio do ROI.
X0_ROI_TOPO_CENTRO, X1_ROI_TOPO_CENTRO = int(FRAME_WIDTH * 0.35), int(FRAME_WIDTH * 0.65)
Y0_ROI_TOPO_CENTRO, Y1_ROI_TOPO_CENTRO = 0, int(FRAME_HEIGHT * 0.30)
Y_MEIO_ROI_TOPO_CENTRO = Y0_ROI_TOPO_CENTRO + (Y1_ROI_TOPO_CENTRO - Y0_ROI_TOPO_CENTRO) // 2
AREA_MIN_TOPO_CENTRO = 150

# --- Verde (gatilho de virada) ---
# Dois ROIs, um em cada canto/lado da imagem. Quando os DOIS têm verde ao
# mesmo tempo, dispara mgr.virar_flag (control.py reseta pra 0 depois de
# executar a virada).
X0_VERDE_ESQ, X1_VERDE_ESQ = 0, int(FRAME_WIDTH * 0.18)
X0_VERDE_DIR, X1_VERDE_DIR = int(FRAME_WIDTH * 0.82), FRAME_WIDTH
Y0_VERDE, Y1_VERDE = int(FRAME_HEIGHT * 0.25), int(FRAME_HEIGHT * 0.75)
VERDE_HSV_MIN = np.array([40, 70, 60])
VERDE_HSV_MAX = np.array([90, 255, 255])
AREA_MIN_VERDE = 120

INTERVALO_CICLO = 0.01  # folga entre capturas, pra não fritar a CPU


# ==================================================================================================================================================================================================================
# CÂMERA
# ==================================================================================================================================================================================================================

def _iniciar_camera():
    """Inicializa a Picamera2 no formato BGR (compatível com OpenCV) e no
    tamanho definido em constants.py. Precisa acontecer dentro do processo
    filho (depois do fork), igual ao pin_factory do GPIO em motores.py."""
    from picamera2 import Picamera2

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "BGR888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.5)  # tempo pra câmera estabilizar exposição/branco
    return picam2


# ==================================================================================================================================================================================================================
# HELPERS DE VISÃO
# ==================================================================================================================================================================================================================

def _centro_de_massa(mask, x0, y0, x1, y1, area_min):
    """Calcula o centro de massa (cx, cy) dos pixels não-zero de `mask`
    dentro do retângulo [x0:x1, y0:y1]. Retorna (achou, cx, cy, area) já
    com cx/cy nas coordenadas do frame inteiro (não do recorte)."""
    roi = mask[y0:y1, x0:x1]
    m = cv2.moments(roi, binaryImage=True)
    area = m["m00"]

    if area < area_min:
        return False, 0, 0, area

    cx = int(m["m10"] / area) + x0
    cy = int(m["m01"] / area) + y0
    return True, cx, cy, area


def _area_verde(mask_verde, x0, y0, x1, y1):
    return cv2.countNonZero(mask_verde[y0:y1, x0:x1])


# ==================================================================================================================================================================================================================
# LOOP PRINCIPAL (roda no processo "line_cam", ver main.py)
# ==================================================================================================================================================================================================================

def capturar_e_processar():
    try:
        picam2 = _iniciar_camera()
    except Exception as e:
        print(f"[line_cam] ERRO ao iniciar a câmera: {e}")
        mgr.camera_ok.value = 0
        return

    print("[line_cam] Câmera iniciada, loop de captura/processamento começou.")

    # view direta pra memória compartilhada, pra publicar o frame sem
    # cópia extra (usado só pra debug/visualização externa; o control.py
    # não depende disso)
    shm_buf = np.ndarray((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8, buffer=mgr.shm.buf)

    try:
        while not mgr.terminate.is_set():
            frame = picam2.capture_array()
            if frame is None:
                mgr.camera_ok.value = 0
                time.sleep(0.02)
                continue

            mgr.camera_ok.value = 1

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            _, mask_preto = cv2.threshold(gray, LIMIAR_PRETO, 255, cv2.THRESH_BINARY_INV)

            # --- linha principal -> erro pro controle P (mgr.line_angle) ---
            achou_linha, cx, _, _ = _centro_de_massa(
                mask_preto, 0, Y0_ROI_LINHA, FRAME_WIDTH, Y1_ROI_LINHA, AREA_MIN_LINHA
            )
            if achou_linha:
                mgr.line_status.value = LINE_FOUND
                mgr.line_angle.value = float(cx - CENTER_X)
                mgr.cx_alvo_v.value = cx
            else:
                mgr.line_status.value = LINE_LOST

            # --- ROI de retorno (checado só depois da virada cega) ---
            achou_retorno, *_ = _centro_de_massa(
                mask_preto, X0_ROI_RETORNO, Y0_ROI_RETORNO, X1_ROI_RETORNO, Y1_ROI_RETORNO, AREA_MIN_RETORNO
            )
            mgr.retorno_linha_ok.value = 1 if achou_retorno else 0

            # --- ROI_TOPO_CENTRO (fim do giro de erro grande) ---
            achou_topo, _, cy_topo, _ = _centro_de_massa(
                mask_preto, X0_ROI_TOPO_CENTRO, Y0_ROI_TOPO_CENTRO, X1_ROI_TOPO_CENTRO, Y1_ROI_TOPO_CENTRO, AREA_MIN_TOPO_CENTRO
            )
            mgr.centro_topo_ok.value = 1 if (achou_topo and cy_topo >= Y_MEIO_ROI_TOPO_CENTRO) else 0

            # --- verde esquerda/direita -> gatilho de virada ---
            mask_verde = cv2.inRange(hsv, VERDE_HSV_MIN, VERDE_HSV_MAX)
            area_verde_esq = _area_verde(mask_verde, X0_VERDE_ESQ, Y0_VERDE, X1_VERDE_ESQ, Y1_VERDE)
            area_verde_dir = _area_verde(mask_verde, X0_VERDE_DIR, Y0_VERDE, X1_VERDE_DIR, Y1_VERDE)
            if area_verde_esq > AREA_MIN_VERDE and area_verde_dir > AREA_MIN_VERDE:
                mgr.virar_flag.value = 1
            # não zera aqui de propósito: quem consome o gatilho (control.py,
            # em executar_virada) é quem reseta virar_flag pra 0 depois de virar

            # --- publica o frame na memória compartilhada (debug/visualização) ---
            with mgr.frame_lock:
                shm_buf[:] = frame
                mgr.novo_frame_flag.value = 1

            time.sleep(INTERVALO_CICLO)
    finally:
        picam2.stop()
        mgr.camera_ok.value = 0
        print("[line_cam] Câmera parada.")
