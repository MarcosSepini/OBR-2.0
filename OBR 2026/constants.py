"""
Constantes globais para o robô seguidor de linha - OBR 2026.
Centraliza calibrações de visão/cores, pinos de motores, ganhos de PID e tratamento de falhas/GAP.
"""

# ==========================================
# CONFIGURAÇÕES DE CÂMERA E IMAGEM
# ==========================================
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
CENTER_X = FRAME_WIDTH // 2
CENTER_Y = FRAME_HEIGHT // 2

# ==========================================
# LIMIARES DE VISÃO E CORES (CALIBRAÇÃO DE PISTA)
# ==========================================
# Limiares de preto (grayscale: 0 a 255)
LIMIAR_PRETO_BASE = 100         # Limiar em iluminação normal
LIMIAR_PRETO_BRILHO = 150       # Limiar em regiões periféricas com reflexo

# Faixas de cor Verde (HSV: H=0-180, S=0-255, V=0-255)
HSV_VERDE_MIN = (35, 60, 50)
HSV_VERDE_MAX = (90, 255, 255)

# Faixas de cor Vermelha (HSV: dois intervalos para cobrir o vermelho no OpenCV)
HSV_VERMELHO_MIN1 = (0, 90, 60)
HSV_VERMELHO_MAX1 = (8, 255, 255)
HSV_VERMELHO_MIN2 = (165, 90, 60)
HSV_VERMELHO_MAX2 = (180, 255, 255)
FRACAO_MIN_VERMELHO = 0.08      # Fração do frame com pixels vermelhos para disparar parada

# Áreas mínimas de contorno (em pixels para resolução 320x240)
AREA_MIN_LINHA_PRETA = 500
AREA_MIN_MARCADOR_VERDE = 300
LIMIAR_ADJACENCIA_VERDE = 0.10  # Densidade mínima de preto adjacente para validar verde

# ==========================================
# PARÂMETROS DE PROCESSAMENTO MORFOLÓGICO
# ==========================================
BLUR_KSIZE_PRETO = 5            # Tamanho do filtro mediano
MORPH_KSIZE = (5, 5)            # Kernel retangular para abertura e fechamento
MORPH_OPEN_ITERS = 1            # Iterações de abertura (elimina ruídos pontuais)
MORPH_CLOSE_ITERS = 2           # Iterações de fechamento (preenche falhas na linha)

# ==========================================
# PINOS DOS MOTORES (PONTE BTS7960)
# ==========================================
RPWM_ESQ, LPWM_ESQ, REN_ESQ, LEN_ESQ = 18, 19, 20, 21
RPWM_DIR, LPWM_DIR, REN_DIR, LEN_DIR = 12, 13, 5, 6

# ==========================================
# STATUS DA LINHA
# ==========================================
LINE_LOST = 0
LINE_TRACKING = 1
LINE_GAP = 2
RED_STOP = 3

# ==========================================
# SINAIS VERDES
# ==========================================
GREEN_NONE = 0
GREEN_LEFT = 1
GREEN_RIGHT = 2
GREEN_DOUBLE = 3
GREEN_APPROACH = 4

GREEN_STR_TO_INT = {
    "NONE": GREEN_NONE,
    "LEFT": GREEN_LEFT,
    "RIGHT": GREEN_RIGHT,
    "DOUBLE": GREEN_DOUBLE,
    "APPROACH": GREEN_APPROACH,
}


GREEN_INT_TO_STR = {
    GREEN_NONE: "NONE",
    GREEN_LEFT: "LEFT",
    GREEN_RIGHT: "RIGHT",
    GREEN_DOUBLE: "DOUBLE",
    GREEN_APPROACH: "APPROACH",
}

# ==========================================
# PARÂMETROS DE CONTROLE E MOTORES (PID)
# ==========================================
BASE_SPEED = 35.0
BASE_SPEED_APPROACH = 25.0

# Ganhos de correção PID
KP_POS = 1.6                    # Peso do desvio do ponto central (X)
KP_ANG = 1.0                    # Peso do desvio angular (direção)
KP_ALTO = 2.4                   # Ganho amplificado para desvios maiores
KD = 0.22                       # Ganho derivativo (amortecimento de oscilação / zig-zag)
KI = 0.01                       # Ganho integral (correção de erro residual em curvas contínuas)
ALPHA_DERIVADA = 0.60           # Filtro passa-baixas da derivada (suavização de jitter)
LIMITE_INTEGRAL = 0.20          # Trava anti-windup para o acumulador integral
ERRO_LIMITE_KP = 0.35
FATOR_CURVA_CONTRA_ROTACAO = 1.5# Fator para contra-rotação fluida da roda interna

# Limites de giro e curvas fechadas
LIMITE_ERRO_GIRO = 60.0         # Graus de erro angular para disparar recuperação de 90°
ERRO_ALVO_GIRO = 15.0           # Erro angular considerado "zerado" para encerrar pivô
TIMEOUT_GIRO_ERRO = 1.5         # Trava de segurança para giros de pivô
KP_GIRO = 0.45                  # Ganho proporcional de giro (desacelera conforme o erro cai)
MIN_VEL_GIRO = 18.0             # Velocidade mínima de pivô para não travar motores
MAX_VEL_GIRO = 35.0             # Velocidade máxima de pivô para não perder a linha
TEMPO_AVANCO_VERTICE_90 = 0.10  # Avanço de alinhamento de eixo no vértice de 90°

# Manobras de Marcadores Verdes
VEL_AVANCO_VERDE = 30.0
TEMPO_AVANCO_VERDE = 0.20
VEL_GIRO_VERDE = 32.0
TEMPO_GIRO_VERDE = 0.55
TIMEOUT_MANOBRA_VERDE = 2.0
COOLDOWN_VERDE = 0.8
COOLDOWN_DUPLO_VERDE = 0.6

# ==========================================
# PARÂMETROS DE GAP E TRATAMENTO DE FALHAS
# ==========================================
VEL_AVANCO_GAP = 24.5           # Velocidade moderada em linha reta durante falhas na linha (GAP)
TIMEOUT_GAP_BUSCA = 1.6         # Tempo máximo insistindo em linha reta antes de busca ativa
TIMEOUT_PERDA_LINHA_SEGURANCA = 2.5 # Timeout de segurança sem linha antes de parar motores

# quando o erro da linha (graus) ultrapassa este limite, positivo ou
# negativo, a correção usa um KP maior (curva mais fechada)
ERRO_LIMITE_KP = 60.0

