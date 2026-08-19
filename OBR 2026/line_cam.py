"""
Módulo de captura de câmera e visão computacional (Line Follower) para OBR 2026.
Processa o feed de vídeo, detecta a linha preta (ponto central e direção/ângulo),
marcadores verdes (esquerda, direita, duplo verde/180) e fita vermelha de parada.
"""

import time
import numpy as np
import cv2
import os

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

import constants

# dá pra desligar o debug sem editar o código: LINE_CAM_DEBUG=0 python3 main.py
#
# IMPORTANTE PRA PERFORMANCE: o modo debug (imshow + waitKey + overlay
# desenhado no frame inteiro) é de longe a coisa mais cara do loop. Em uso
# real no robô, rode SEMPRE com LINE_CAM_DEBUG=0 -- só ligue pra calibrar
# olhando a imagem.
DEBUG = os.environ.get("LINE_CAM_DEBUG", "0") == "0"
CAMERA_NUM = 0

# Câmera montada de cabeça para baixo (ou virada) no robô: gira o frame
# 180 graus logo após capturar, antes de qualquer processamento. Se a
# câmera for remontada na orientação normal no futuro, é só trocar para
# ROTACIONAR_CAMERA = False.
ROTACIONAR_CAMERA = True

BLACK_THRESH = 87
MIN_CONTOUR_AREA = 2500

# a maior ROI de verde (ROI_ESQ) tem 60x200 = 12000 px no total -- o valor
# antigo (8000000) era maior que isso, ou seja, nunca validava contorno
# nenhum. Comece com isto e calibre olhando o debug (LINE_CAM_DEBUG=1) com
# a área real das manchas de ruído vs. da fita/marcação verde de verdade.
MIN_CONTOUR_AREA_VERDE = 15000000000

# verde em HSV (H: 0-179 no OpenCV) -- bem mais robusto à luz que threshold em RGB.
# S e V mínimos foram subidos (eram 40/40): em ambiente escuro o ganho
# automático da câmera amplifica ruído, e ruído de sensor tende a ter
# saturação BAIXA (fica meio acinzentado) mesmo quando o matiz (H) cai por
# acaso na faixa do verde -- então exigir mais saturação/brilho filtra boa
# parte desse ruído sem precisar mexer no matiz. Se a fita verde real
# também for escura, pode precisar abaixar o V de novo -- teste na prática.
VERDE_HSV_BAIXO = np.array([35, 90, 60])
VERDE_HSV_ALTO = np.array([85, 255, 255])

# kernel usado pra limpar ruído (abre) e fechar buracos pequenos (fecha) nas máscaras.
# Kernel menor (era 5x5) custa bem menos por pixel e ainda limpa ruído de
# sensor sem perder pedaço da linha/fita -- se voltar a aparecer ruído,
# suba de novo pra (5,5).
KERNEL_MORFOLOGICO = np.ones((3, 3), np.uint8)

ROI_ESQ = (0, 155, 0, 200)
ROI_DIR = (165, 320, 0, 200)

# ROIs de "topo" (mais longe do robô) usados para o seguimento normal de
# linha (sem verde). Formato: (x1, x2, y1, y2)
ROI_TOPO_ESQ = (0, 67, 0, 77)
ROI_TOPO_CENTRO = (67, 253, 0, 77)
ROI_TOPO_DIR = (253, 320, 0, 77)

# ROI no canto inferior esquerdo usado só depois da virada de 90 graus,
# para saber quando a linha foi reencontrada
ROI_RETORNO_ESQ_INF = (0, 60, 150, 200)
FRACAO_PRETO_RETORNO = 0.2 # % do ROI que precisa estar preta p/ considerar "achou a linha"

# usado por mgr.centro_topo_ok (ver capturar_e_processar): em vez de
# considerar "achou" assim que QUALQUER preto aparece no ROI_TOPO_CENTRO,
# só conta quando o ponto MAIS ALTO do contorno (y_min, retornado por
# detectar_ponto_extremo_preto(..., "cima")) já chegou pelo menos até o
# MEIO da altura do ROI_TOPO_CENTRO (ou passou dele, y_min menor ainda)
# -- ou seja, precisa de uma fatia razoável de preto visível ali dentro,
# não só a pontinha entrando pela borda de baixo do ROI.
#
# ANTES exigia y_min DENTRO de uma janela estreita (30 +/- 8, ou seja só
# 22-38): durante o giro no próprio eixo a linha entra rápido e o y_min
# pula de frame pra frame (ex: 50 -> 12), pulando por cima dessa janela
# estreita sem nunca cair dentro dela -- então centro_topo_ok quase nunca
# ficava 1, o giro nunca confirmava e só parava no TIMEOUT_GIRO, entrando
# de novo no giro em seguida (parecia girar pra sempre). Agora é uma
# condição de "cheguei até o meio ou além", que não depende de acertar
# um frame bem no meio do intervalo.
Y_MEIO_ROI_TOPO_CENTRO = (ROI_TOPO_CENTRO[2] + ROI_TOPO_CENTRO[3]) / 2  # (0+60)/2 = 30
TOLERANCIA_Y_MEIO_TOPO_CENTRO = 8  # px de folga -- calibre na prática

cx_anterior = None  # último centro conhecido da linha preta (usado quando só há verde)
soma_erro_debug = 0.0  # acumulador só para exibir "SUM" no overlay de debug

# FPS mostrado no overlay de debug -- suavizado (média móvel exponencial)
# pra não ficar piscando número a cada frame; só é atualizado/lido quando
# debug_ativo (ver capturar_e_processar), então não custa nada com debug
# desligado.
_fps_debug = 0.0
_tempo_ultimo_frame_debug = None
FPS_SUAVIZACAO = 0.1  # 0 = trava no valor antigo, 1 = sem suavização nenhuma

_contagem_verde_180 = 0  # frames seguidos com verde válido nos dois lados
CICLOS_CONFIRMACAO_VERDE_180 = 5  # quantos frames seguidos pra confirmar o giro de 180 (filtra ruído de 1 frame só)

# Cache de máscara (preto/verde) por ROI, válido só dentro do MESMO frame.
# Antes, a mesma ROI (ex: ROI_TOPO_CENTRO, ROI_ESQ, ROI_DIR) tinha sua
# máscara recalculada do zero -- com GaussianBlur + 2x morphologyEx, as
# partes mais caras do pipeline -- toda vez que uma função diferente
# pedia preto/verde nela, às vezes 2-3x por frame. Com o cache, cada ROI
# só processa a máscara uma vez por frame; _limpar_cache_mascaras() é
# chamado no começo de cada iteração do loop principal.
_mascara_cache = {}

# Cache do recorte já borrado (GaussianBlur) por ROI, independente do
# tipo (preto/verde) -- ROI_ESQ e ROI_DIR, por exemplo, precisam de
# máscara preta E verde no mesmo frame; antes disso o blur (a parte mais
# cara do cálculo da máscara) era feito 2x em cima do mesmo recorte, uma
# vez pra cada tipo. Agora é feito 1x só e reaproveitado.
_recorte_blur_cache = {}

# Cache do maior contorno (findContours + max + contourArea) por (roi, tipo),
# válido só dentro do MESMO frame -- essa é a parte mais cara de todo o
# pipeline depois do blur. Ex: no seguimento normal, TODO frame chama
# detectar_centro_preto(ROI_TOPO_CENTRO) e, na sequência, também
# detectar_ponto_extremo_preto(ROI_TOPO_CENTRO, "cima") -- as duas rodavam
# findContours do zero em cima da MESMA máscara. Com esse cache, o
# findContours + achar o maior contorno roda 1x por (roi, tipo) por frame,
# não importa quantas funções diferentes precisem dele.
_contorno_cache = {}


def _limpar_cache_mascaras():
    _mascara_cache.clear()
    _recorte_blur_cache.clear()
    _contorno_cache.clear()


def _obter_recorte_borrado(frame_rgb, roi):
    """Recorta a ROI do frame e aplica o GaussianBlur uma única vez por
    frame, cacheado só por roi (sem depender do tipo preto/verde)."""
    cache_hit = _recorte_blur_cache.get(roi)
    if cache_hit is not None:
        return cache_hit

    x1, x2, y1, y2 = roi
    recorte = frame_rgb[y1:y2, x1:x2]
    borrado = cv.GaussianBlur(recorte, (5, 5), 0)
    resultado = (borrado, x1, y1)
    _recorte_blur_cache[roi] = resultado
    return resultado


def _obter_mascara_roi(frame_rgb, roi, tipo):
    """Retorna (mask, x1, y1) da ROI, usando cache por (roi, tipo) dentro
    do frame atual. tipo: 'preto' ou 'verde'."""
    chave = (roi, tipo)
    cache_hit = _mascara_cache.get(chave)
    if cache_hit is not None:
        return cache_hit

    borrado, x1, y1 = _obter_recorte_borrado(frame_rgb, roi)
    mask = _mascara_preto_de_borrado(borrado) if tipo == "preto" else _mascara_verde_de_borrado(borrado)
    resultado = (mask, x1, y1)
    _mascara_cache[chave] = resultado
    return resultado


def _obter_maior_contorno(frame_rgb, roi, tipo):
    """Retorna (maior, area_maior, mask, x1, y1) do maior contorno da
    máscara (roi, tipo), cacheado por frame. 'maior' vem None quando não
    há contorno nenhum na máscara (area_maior fica 0.0 nesse caso).

    Antes, cada função de detecção (detectar_centro_preto,
    detectar_ponto_extremo_preto, etc.) chamava cv.findContours + procurava
    o maior contorno do zero -- mesmo quando duas funções diferentes
    pediam isso para a MESMA (roi, tipo) no MESMO frame (ex:
    ROI_TOPO_CENTRO é consultado 2x todo frame no seguimento normal).
    findContours é uma das chamadas mais caras do pipeline depois do
    blur, então esse cache evita repetir esse trabalho."""
    chave = (roi, tipo)
    cache_hit = _contorno_cache.get(chave)
    if cache_hit is not None:
        return cache_hit

    mask, x1, y1 = _obter_mascara_roi(frame_rgb, roi, tipo)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        resultado = (None, 0.0, mask, x1, y1)
    else:
        maior = max(contours, key=cv.contourArea)
        area_maior = cv.contourArea(maior)
        resultado = (maior, area_maior, mask, x1, y1)
    _contorno_cache[chave] = resultado
    return resultado


def _limpar_mascara(mask):
    """Tira ruído solto (opening) e fecha buraquinhos/serrilhado (closing)."""
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, KERNEL_MORFOLOGICO)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, KERNEL_MORFOLOGICO)
    return mask


def _mascara_preto_de_borrado(borrado):
    # Mesma condição de antes ("todos os canais < BLACK_THRESH"), mas via
    # cv.inRange (roda dentro do OpenCV) em vez de np.all pixel a pixel em
    # Python/NumPy -- mesmo resultado, bem mais leve pra CPU.
    limite_superior = BLACK_THRESH - 1
    mask = cv.inRange(borrado,
                       (0, 0, 0),
                       (limite_superior, limite_superior, limite_superior))
    return _limpar_mascara(mask)


def _mascara_verde_de_borrado(borrado):
    hsv = cv.cvtColor(borrado, cv.COLOR_RGB2HSV)
    mask = cv.inRange(hsv, VERDE_HSV_BAIXO, VERDE_HSV_ALTO)
    return _limpar_mascara(mask)


def mascara_preto(frame_rgb):
    """Mantida para o overlay de debug (opera no frame/recorte inteiro,
    fora do sistema de cache por ROI)."""
    borrado = cv.GaussianBlur(frame_rgb, (5, 5), 0)
    return _mascara_preto_de_borrado(borrado)


def mascara_verde(frame_rgb):
    """Mantida para o overlay de debug (opera no frame/recorte inteiro,
    fora do sistema de cache por ROI)."""
    borrado = cv.GaussianBlur(frame_rgb, (5, 5), 0)
    return _mascara_verde_de_borrado(borrado)


def detectar_centro_preto(frame_rgb, roi):
    maior, area_maior, mask, x1, y1 = _obter_maior_contorno(frame_rgb, roi, "preto")
    if maior is None or area_maior < MIN_CONTOUR_AREA:
        return None

    M = cv.moments(maior)
    if M["m00"] == 0:
        return None

    cx_local = int(M["m10"] / M["m00"])
    cy_local = int(M["m01"] / M["m00"])
    if mask[0, cx_local] == 255:
        cy_local = 0

    return (cx_local + x1, cy_local + y1)


def detectar_centro_verde(frame_rgb, roi):
    maior, area_maior, mask, x1, y1 = _obter_maior_contorno(frame_rgb, roi, "verde")
    if maior is None or area_maior < MIN_CONTOUR_AREA_VERDE:
        return None

    M = cv.moments(maior)
    if M["m00"] == 0:
        return None

    cx_local = int(M["m10"] / M["m00"])
    cy_local = int(M["m01"] / M["m00"])
    if mask[0, cx_local] == 255:
        cy_local = 0

    return (cx_local + x1, cy_local + y1)


def _mascara_contorno_recortada(maior):
    """Preenche o contorno 'maior' num canvas do tamanho do SEU bounding
    box, não da ROI inteira -- resultado idêntico ao de desenhar na ROI
    inteira e depois recortar, mas bem mais barato: normalmente a linha
    ocupa só uma fração da ROI (ex: ROI_ESQ/ROI_DIR tem 200x60/66 px, a
    linha em si costuma ser bem menor que isso), então tanto o
    cv.drawContours quanto o np.where que vem depois (em _ponto_extremo /
    _ponto_x_topo_y_max) trabalham numa área bem menor.

    Retorna (mask_recortada, dx, dy): dx/dy são o deslocamento do canto
    do bounding box em relação à ROI, para somar junto com x1/y1."""
    dx, dy, w, h = cv.boundingRect(maior)
    mask_recortada = np.zeros((h, w), dtype=np.uint8)
    contorno_deslocado = maior - (dx, dy)
    cv.drawContours(mask_recortada, [contorno_deslocado], -1, 255, -1)
    return mask_recortada, dx, dy


def _ponto_extremo(mask, x1, y1, direcao):
    ys, xs = np.where(mask == 255)
    if len(ys) == 0:
        return None
    if direcao == "esquerda":
        idx = np.argmin(xs)
        return (int(xs[idx]) + x1, int(ys[idx]) + y1)
    elif direcao == "direita":
        idx = np.argmax(xs)
        return (int(xs[idx]) + x1, int(ys[idx]) + y1)
    else:  # "cima"
        # bug antigo: pegava só o primeiro pixel do menor y (argmin), que no
        # np.where sempre vem do lado esquerdo -> ponto ficava puxado pra
        # esquerda mesmo com a linha centralizada. Correção: média de x de
        # TODOS os pixels que estão na borda de cima (mesmo y mínimo).
        y_min = ys.min()
        xs_no_topo = xs[ys == y_min]
        cx_topo = int(round(xs_no_topo.mean()))
        return (cx_topo + x1, int(y_min) + y1)


def _ponto_x_topo_y_max(mask, x1, y1):
    """Combina x da linha mais de cima do contorno (mesma média usada na
    direção 'cima') com o y mais baixo (maior valor) de TODO o contorno.
    Ou seja: x de onde a linha entra no ROI por cima, y do ponto mais
    perto do robô (mais embaixo) que esse mesmo contorno alcança."""
    ys, xs = np.where(mask == 255)
    if len(ys) == 0:
        return None

    y_min = ys.min()
    xs_no_topo = xs[ys == y_min]
    cx_topo = int(round(xs_no_topo.mean()))

    y_max = ys.max()

    return (cx_topo + x1, int(y_max) + y1)


def detectar_ponto_extremo_preto(frame_rgb, roi, direcao):
    """Ponto extremo do maior contorno preto dentro do ROI.
    direcao: 'esquerda' (menor x), 'direita' (maior x) ou 'cima' (menor y)."""
    maior, area_maior, mask, x1, y1 = _obter_maior_contorno(frame_rgb, roi, "preto")
    if maior is None or area_maior < MIN_CONTOUR_AREA:
        return None

    mask_maior, dx, dy = _mascara_contorno_recortada(maior)
    return _ponto_extremo(mask_maior, x1 + dx, y1 + dy, direcao)


def detectar_ponto_x_topo_y_max_preto(frame_rgb, roi):
    """Igual detectar_ponto_extremo_preto, mas o ponto retornado usa
    _ponto_x_topo_y_max (x da linha mais de cima + y mais embaixo do
    contorno), em vez de um extremo único (esquerda/direita/cima)."""
    maior, area_maior, mask, x1, y1 = _obter_maior_contorno(frame_rgb, roi, "preto")
    if maior is None or area_maior < MIN_CONTOUR_AREA:
        return None

    mask_maior, dx, dy = _mascara_contorno_recortada(maior)
    return _ponto_x_topo_y_max(mask_maior, x1 + dx, y1 + dy)


def detectar_ponto_extremo_verde(frame_rgb, roi, direcao):
    """Igual detectar_ponto_extremo_preto, mas para o verde."""
    maior, area_maior, mask, x1, y1 = _obter_maior_contorno(frame_rgb, roi, "verde")
    if maior is None or area_maior < MIN_CONTOUR_AREA_VERDE:
        return None

    mask_maior, dx, dy = _mascara_contorno_recortada(maior)
    return _ponto_extremo(mask_maior, x1 + dx, y1 + dy, direcao)


def roi_majoritariamente_preto(frame_rgb, roi, fracao=FRACAO_PRETO_RETORNO):
    """True se o ROI estiver quase todo preto (usado para achar a linha de volta)."""
    mask, _, _ = _obter_mascara_roi(frame_rgb, roi, "preto")
    return (np.count_nonzero(mask) / mask.size) >= fracao


def processar_linha_topo(frame_rgb):
    """Seguimento normal de linha (sem verde), usando 3 ROIs mais acima:
    esquerda, centro e direita.

    Só considera algo se houver preto no ROI CENTRAL:
      - preto na esquerda + centro  -> vira para a esquerda (usa o CENTRO do preto na esquerda)
      - preto na direita  + centro  -> vira para a direita  (usa o CENTRO do preto na direita)
      - preto só no centro          -> segue reto (usa o ponto mais alto do centro)
      - preto nos três (cruzamento) -> segue reto (mesmo caso acima)

    Se não achar preto nenhum no ROI central de cima (linha "sumiu" lá em
    cima), tenta um fallback usando os ROIs INTEIROS de esquerda e direita
    (ROI_ESQ / ROI_DIR, os mesmos usados na lógica do verde). Em vez do
    centroide puro, cada ponto usa _ponto_x_topo_y_max: x = média das
    colunas da linha mais de cima do contorno (mesma lógica da direção
    "cima"), y = y MÁXIMO (ponto mais baixo/mais perto do robô) de todo
    o contorno:
      - preto nos dois (esq + dir)  -> x = média dos dois x's, y = o maior
                                        dos dois y's máximos
      - preto só na esquerda        -> usa o ponto da esquerda
      - preto só na direita         -> usa o ponto da direita
      - preto em nenhum dos dois    -> linha perdida (LINE_LOST)

    Retorna também, como último valor, o 'centro' bruto (resultado de
    detectar_centro_preto no ROI_TOPO_CENTRO): None quando veio do
    fallback dos ROIs inteiros, ou o ponto quando achou de verdade no
    ROI_TOPO_CENTRO. Usado por quem chama pra publicar mgr.centro_topo_ok.
    """
    centro = detectar_centro_preto(frame_rgb, ROI_TOPO_CENTRO)

    if centro is None:
        centro_esq_full = detectar_ponto_x_topo_y_max_preto(frame_rgb, ROI_ESQ)
        centro_dir_full = detectar_ponto_x_topo_y_max_preto(frame_rgb, ROI_DIR)

        if centro_esq_full is not None and centro_dir_full is not None:
            cx_fallback = (centro_esq_full[0] + centro_dir_full[0]) // 2
            cy_fallback = max(centro_esq_full[1], centro_dir_full[1])
            ponto_alvo = (cx_fallback, cy_fallback)
        elif centro_esq_full is not None:
            ponto_alvo = centro_esq_full
        elif centro_dir_full is not None:
            ponto_alvo = centro_dir_full
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
