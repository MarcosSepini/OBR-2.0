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

# quando o erro da linha (graus) ultrapassa este limite, positivo ou
# negativo, a correção usa um KP maior (curva mais fechada)
ERRO_LIMITE_KP = 90.0
