FRAME_WIDTH = 320 #LARGURA
FRAME_HEIGHT = 200 #ALTURA
FRAME_SHAPE = (FRAME_HEIGHT, FRAME_WIDTH, 3)
FRAME_NBYTES = FRAME_HEIGHT * FRAME_WIDTH * 3

LINE_LOST = 0
LINE_FOUND = 1

# tempo (em segundos) da virada "cega" para sair da linha pela esquerda.
# Sem giroscópio pra medir o ângulo, a virada agora é por tempo fixo --
# calibre na prática cronometrando o robô girando ~90 graus na VEL_VIRADA.
TEMPO_VIRADA = 0.6

# tempo (em segundos) do giro no próprio eixo quando o erro fica muito
# grande (abs(erro) >= LIMITE_ERRO_GIRO, ver control.py). Por enquanto
# também é por tempo fixo, igual TEMPO_VIRADA (antes girava até o
# ROI_TOPO_CENTRO reencontrar a linha) -- calibre cronometrando o giro na
# VEL_GIRO até dar a volta desejada.
TEMPO_GIRO_ERRO = 1.2


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

# quando o erro da linha (graus) ultrapassa este limite, positivo ou
# negativo, a correção usa um KP maior (curva mais fechada)
ERRO_LIMITE_KP = 90.0

