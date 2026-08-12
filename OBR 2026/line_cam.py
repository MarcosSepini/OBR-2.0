import os
import cv2 as cv
import numpy as np
import time
from picamera2 import Picamera2
from multiprocessing.shared_memory import SharedMemory

import mp_manager as mgr
from constants import FRAME_WIDTH, FRAME_HEIGHT, FRAME_SHAPE, LINE_LOST, LINE_FOUND

# dá pra desligar o debug sem editar o código: LINE_CAM_DEBUG=0 python3 main.py
#
# IMPORTANTE PRA PERFORMANCE: o modo debug (imshow + waitKey + overlay
# desenhado no frame inteiro) é de longe a coisa mais cara do loop. Em uso
# real no robô, rode SEMPRE com LINE_CAM_DEBUG=0 -- só ligue pra calibrar
# olhando a imagem.
DEBUG = os.environ.get("LINE_CAM_DEBUG", "0") == "1"
CAMERA_NUM = 0

# Câmera montada de cabeça para baixo (ou virada) no robô: gira o frame
# 180 graus logo após capturar, antes de qualquer processamento. Se a
# câmera for remontada na orientação normal no futuro, é só trocar para
# ROTACIONAR_CAMERA = False.
ROTACIONAR_CAMERA = True

BLACK_THRESH = 20
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
            return None, LINE_LOST, None, None, None, None, None

        cx_alvo = ponto_alvo[0]
        return cx_alvo, LINE_FOUND, centro_esq_full, centro_dir_full, None, ponto_alvo, None

    # antes usava detectar_ponto_extremo_preto(..., "esquerda"/"direita"),
    # agora usa o CENTRO do preto em cada ROI de topo esquerda/direita
    esq = detectar_centro_preto(frame_rgb, ROI_TOPO_ESQ)
    dir_ = detectar_centro_preto(frame_rgb, ROI_TOPO_DIR)
    topo_centro = detectar_ponto_extremo_preto(frame_rgb, ROI_TOPO_CENTRO, "cima")

    if esq is not None and dir_ is None:
        ponto_alvo = esq
    elif dir_ is not None and esq is None:
        ponto_alvo = dir_
    else:
        ponto_alvo = topo_centro if topo_centro is not None else centro

    cx_alvo = ponto_alvo[0]
    return cx_alvo, LINE_FOUND, esq, dir_, topo_centro, ponto_alvo, centro


def processar_linha_com_verde(frame_rgb):
    """Lógica do verde: só serve pra 2 coisas — disparar a virada (acao) e,
    quando fizer sentido, sugerir um ponto_verde que sobrescreve o cx_alvo
    normal (processar_linha_topo). Quando não há verde nenhum, ponto_verde
    volta None e quem chama usa o cx_alvo normal (só preto).

    1) verde só é válido se estiver ABAIXO do preto (mais perto do robô);
       se estiver acima (mais longe), é descartado
    2) 2 verdes válidos (esq + dir)   -> acao = "180" (vira com o giroscópio)
    3) só o esquerdo válido           -> ponto_verde = centro do preto na esquerda
    4) só o direito válido            -> ponto_verde = ponto mais alto do preto na direita
    5) tem verde, mas nenhum válido   -> ponto_verde = ponto mais alto do ROI central de topo
    6) não tem verde nenhum           -> ponto_verde = None (segue normal, só preto)
    """
    global cx_anterior, _contagem_verde_180

    centro_verde_esq = detectar_ponto_extremo_verde(frame_rgb, ROI_ESQ, "esquerda")
    centro_verde_dir = detectar_ponto_extremo_verde(frame_rgb, ROI_DIR, "direita")

    centro_preto_esq = detectar_centro_preto(frame_rgb, ROI_ESQ)
    centro_preto_dir = detectar_centro_preto(frame_rgb, ROI_DIR)

    tem_preto_esq = centro_preto_esq is not None
    tem_preto_dir = centro_preto_dir is not None

    if tem_preto_esq and tem_preto_dir:
        cx_preto = (centro_preto_esq[0] + centro_preto_dir[0]) // 2
        cy_preto = min(centro_preto_esq[1], centro_preto_dir[1])
    elif tem_preto_esq:
        cx_preto, cy_preto = centro_preto_esq
    elif tem_preto_dir:
        cx_preto, cy_preto = centro_preto_dir
    else:
        cx_preto, cy_preto = None, None

    if cx_preto is not None:
        cx_anterior = cx_preto
    elif centro_verde_esq is not None or centro_verde_dir is not None:
        cx_preto = cx_anterior

    # verde só é válido se estiver ABAIXO do preto (mais perto do robô);
    # se estiver acima (mais longe), descarta
    if cy_preto is not None:
        verde_esq_valido = centro_verde_esq is not None and centro_verde_esq[1] > cy_preto
        verde_dir_valido = centro_verde_dir is not None and centro_verde_dir[1] > cy_preto
    else:
        verde_esq_valido = centro_verde_esq is not None
        verde_dir_valido = centro_verde_dir is not None

    tem_verde_qualquer = centro_verde_esq is not None or centro_verde_dir is not None

    acao = None
    ponto_verde = None

    if verde_esq_valido and verde_dir_valido:
        _contagem_verde_180 += 1
        if _contagem_verde_180 >= CICLOS_CONFIRMACAO_VERDE_180:
            acao = "180"
    else:
        _contagem_verde_180 = 0
        if verde_esq_valido:
            ponto_verde = centro_preto_esq
        elif verde_dir_valido:
            ponto_verde = detectar_ponto_extremo_preto(frame_rgb, ROI_DIR, "cima")
        elif tem_verde_qualquer:
            ponto_verde = detectar_ponto_extremo_preto(frame_rgb, ROI_TOPO_CENTRO, "cima")

    return acao, ponto_verde, cx_preto, centro_preto_esq, centro_preto_dir, centro_verde_esq, centro_verde_dir


def _desenhar_contorno_linha(frame_bgr, frame_rgb):
    """Traça o contorno da linha preta (ciano) direto no frame de debug (BGR)."""
    mask = mascara_preto(frame_rgb)
    contornos, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(frame_bgr, contornos, -1, (255, 255, 0), 2)  # ciano, em BGR


def _desenhar_contorno_verde(frame_bgr, frame_rgb):
    """Traça o contorno do verde detectado (verde) direto no frame de debug (BGR)."""
    mask = mascara_verde(frame_rgb)
    contornos, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(frame_bgr, contornos, -1, (0, 255, 0), 2)  # verde, em BGR


def _desenhar_ponto_alvo(frame_bgr, ponto_alvo):
    """Ponto usado para calcular o erro — mesma cor azul-clara/ciano do contorno."""
    if ponto_alvo is not None:
        cv.circle(frame_bgr, ponto_alvo, 6, (255, 255, 0), -1)
        cv.circle(frame_bgr, ponto_alvo, 8, (255, 255, 255), 1)


def _desenhar_centros_verde(frame_bgr, centro_verde_esq, centro_verde_dir):
    """Marca o centro do verde detectado de cada lado."""
    for centro in (centro_verde_esq, centro_verde_dir):
        if centro is not None:
            cv.circle(frame_bgr, centro, 6, (0, 255, 0), -1)
            cv.circle(frame_bgr, centro, 8, (0, 0, 0), 1)


def _desenhar_centroide_verde(frame_bgr, centroide_verde_esq, centroide_verde_dir):
    """Marca o CENTRÓIDE (moments, meio de verdade da mancha) do verde de
    cada lado -- diferente dos pontos extremos já marcados em
    _desenhar_centros_verde (que são o mais à esquerda/direita, usados na
    lógica de decisão). Desenhado como uma cruz verde pra não confundir
    com a bolinha cheia dos pontos extremos."""
    for centroide in (centroide_verde_esq, centroide_verde_dir):
        if centroide is not None:
            cv.drawMarker(frame_bgr, centroide, (0, 255, 0),
                           markerType=cv.MARKER_CROSS, markerSize=16, thickness=2)
            cv.drawMarker(frame_bgr, centroide, (0, 0, 0),
                           markerType=cv.MARKER_CROSS, markerSize=10, thickness=1)


def _texto_verde(centro_verde_esq, centro_verde_dir):
    if centro_verde_esq is not None and centro_verde_dir is not None:
        return "AMBOS"
    if centro_verde_esq is not None:
        return "ESQ"
    if centro_verde_dir is not None:
        return "DIR"
    return "None"


def _iniciar_debug_window():
    """Tenta abrir a janela de debug. Se não houver display (modo headless),
    desativa o debug.

    Importante: sem a variável DISPLAY (ou WAYLAND_DISPLAY), o backend Qt do
    OpenCV NÃO lança cv.error — ele aborta o processo inteiro (SIGABRT),
    ignorando totalmente o try/except. Por isso o check de ambiente vem
    ANTES de qualquer chamada gráfica; o try/except abaixo só cobre outras
    falhas (ex: display presente mas com problema)."""
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("[line_cam] DEBUG desativado (sem DISPLAY/WAYLAND_DISPLAY — modo headless).")
        return False
    try:
        cv.namedWindow("line_cam debug", cv.WINDOW_NORMAL)
        return True
    except cv.error as e:
        print(f"[line_cam] DEBUG desativado (erro ao abrir janela): {e}")
        return False


def capturar_e_processar():
    global soma_erro_debug, _fps_debug, _tempo_ultimo_frame_debug

    shm = SharedMemory(name=mgr.shm.name)
    frame_buf = np.ndarray(FRAME_SHAPE, dtype=np.uint8, buffer=shm.buf)

    debug_ativo = DEBUG and _iniciar_debug_window()

    try:
        picam2 = Picamera2(camera_num=CAMERA_NUM)
        config = picam2.create_video_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
            buffer_count=4,  # mais buffers em voo = menos frames perdidos/travados esperando o consumidor
        )
        picam2.configure(config)
        # Sobe o teto de FPS que o sensor pode entregar (em microssegundos:
        # duração mínima/máxima de frame). 8000us -> até ~125 fps; ajuste o
        # limite inferior conforme o sensor/iluminação permitirem sem ficar
        # escuro/ruidoso demais. Sem isso o picamera2 costuma ficar preso
        # numa taxa bem mais conservadora por padrão.
        try:
            picam2.set_controls({"FrameDurationLimits": (8000, 33333)})
        except Exception as e:
            print(f"[line_cam] aviso: não deu pra ajustar FrameDurationLimits: {e}")
        picam2.start()
        time.sleep(0.01)
    except Exception as e:
        print(f"ERRO ao iniciar a Picamera2: {e}")
        mgr.camera_ok.value = 0
        shm.close()
        return

    mgr.camera_ok.value = 1
    center_x = FRAME_WIDTH // 2

    try:
        while not mgr.terminate.is_set():
            # 1) CAPTURAR FRAME
            try:
                frame_rgb = picam2.capture_array()
            except Exception as e:
                print(f"ERRO ao capturar frame da Picamera2: {e}")
                mgr.camera_ok.value = 0
                mgr.line_status.value = LINE_LOST
                break

            # Câmera montada invertida no robô: gira 180 graus antes de
            # qualquer processamento.
            if ROTACIONAR_CAMERA:
                frame_rgb = cv.rotate(frame_rgb, cv.ROTATE_180)

            h, w = frame_rgb.shape[:2]
            if (w, h) != (FRAME_WIDTH, FRAME_HEIGHT):
                frame_rgb = cv.resize(frame_rgb, (FRAME_WIDTH, FRAME_HEIGHT))

            # Cache de máscaras válido só para ESTE frame -- zera antes de
            # qualquer detecção rodar (ver _obter_mascara_roi).
            _limpar_cache_mascaras()

            with mgr.frame_lock:
                frame_buf[:] = frame_rgb
                mgr.novo_frame_flag.value = 1

            # 2) Seguimento normal de linha (sem verde), com os 3 ROIs de topo
            (cx_alvo_topo, status_linha, topo_esq, topo_dir,
             topo_centro, ponto_alvo_topo, centro_topo) = processar_linha_topo(frame_rgb)

            # sinal usado pelo control: True só quando o preto foi achado de
            # verdade no ROI_TOPO_CENTRO (não veio do fallback esq/dir) E o
            # ponto mais alto do contorno (topo_centro, y_min) já chegou
            # perto do MEIO da altura desse ROI -- não basta só "apareceu"
            centro_topo_ok = (
                topo_centro is not None
                and topo_centro[1] <= (Y_MEIO_ROI_TOPO_CENTRO + TOLERANCIA_Y_MEIO_TOPO_CENTRO)
            )
            mgr.centro_topo_ok.value = 1 if centro_topo_ok else 0

            # 3) VERDE nos ROIs de baixo: dispara a virada (acao) e, quando fizer
            # sentido, sugere um ponto_verde que sobrescreve o cx_alvo normal
            (acao, ponto_verde, cx_preto,
             centro_preto_esq, centro_preto_dir,
             centro_verde_esq, centro_verde_dir) = processar_linha_com_verde(frame_rgb)

            mgr.virar_flag.value = 1 if acao == "180" else 0

            if ponto_verde is not None:
                ponto_alvo = ponto_verde
                cx_alvo = ponto_verde[0]
            else:
                ponto_alvo = ponto_alvo_topo
                cx_alvo = cx_alvo_topo

            mgr.line_status.value = status_linha
            if cx_alvo is not None:
                mgr.line_angle.value = cx_alvo - center_x
                mgr.cx_alvo_v.value = cx_alvo

            # 4) ROI de retorno (canto inferior esquerdo), usado só depois da virada
            mgr.retorno_linha_ok.value = 1 if roi_majoritariamente_preto(frame_rgb, ROI_RETORNO_ESQ_INF) else 0

            if debug_ativo:
                erro_debug = (cx_alvo - center_x) if cx_alvo is not None else 0
                soma_erro_debug = soma_erro_debug + erro_debug if status_linha == LINE_FOUND else 0.0

                # FPS suavizado (média móvel exponencial) com base no tempo
                # entre um frame de debug e o próximo -- só calculado aqui
                # dentro, então não pesa nada com debug desligado.
                agora = time.time()
                if _tempo_ultimo_frame_debug is not None:
                    delta = agora - _tempo_ultimo_frame_debug
                    if delta > 0:
                        fps_instantaneo = 1.0 / delta
                        _fps_debug = (_fps_debug + FPS_SUAVIZACAO * (fps_instantaneo - _fps_debug)
                                      if _fps_debug > 0 else fps_instantaneo)
                _tempo_ultimo_frame_debug = agora

                # centróide (moments) do verde de cada lado, só pra debug --
                # usa o mesmo cache de máscaras da ROI (já calculada acima
                # em processar_linha_com_verde), então não recalcula à toa.
                centroide_verde_esq = detectar_centro_verde(frame_rgb, ROI_ESQ)
                centroide_verde_dir = detectar_centro_verde(frame_rgb, ROI_DIR)

                frame_debug = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)
                _desenhar_contorno_linha(frame_debug, frame_rgb)
                _desenhar_contorno_verde(frame_debug, frame_rgb)
                _desenhar_ponto_alvo(frame_debug, ponto_alvo)
                _desenhar_centros_verde(frame_debug, centro_verde_esq, centro_verde_dir)
                _desenhar_centroide_verde(frame_debug, centroide_verde_esq, centroide_verde_dir)

                linhas_texto = [
                    f"FPS: {_fps_debug:.1f}",
                    f"GREEN: {_texto_verde(centro_verde_esq, centro_verde_dir)}",
                    f"TURN: {erro_debug}",
                    f"SUM: {int(soma_erro_debug)}",
                ]
                for i, txt in enumerate(linhas_texto):
                    cv.putText(frame_debug, txt, (10, 25 + i * 25),
                               cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                cv.imshow("line_cam debug", frame_debug)
                if cv.waitKey(1) & 0xFF == ord('q'):
                    debug_ativo = False
                    cv.destroyAllWindows()
    finally:
        mgr.camera_ok.value = 0
        if debug_ativo:
            cv.destroyAllWindows()
        picam2.stop()
        shm.close()