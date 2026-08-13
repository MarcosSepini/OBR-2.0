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

# o giro no próprio eixo (quando abs(erro) >= LIMITE_ERRO_GIRO, ver
# control.py) agora NÃO é mais por tempo fixo: ele fica girando (pivô
# puro) e reavaliando o erro a cada ciclo, e só para quando o erro cai
# até ERRO_ALVO_GIRO (ou menos). TIMEOUT_GIRO_ERRO é só uma trava de
# segurança -- se por algum motivo o erro nunca cair (linha sumiu, etc.)
# o giro para sozinho depois desse tempo, em vez de girar pra sempre.
ERRO_ALVO_GIRO = 15.0  # abaixo disso (em px de erro), considera "zerado" e para o giro
TIMEOUT_GIRO_ERRO = 1.2  # segundos -- trava de segurança, calibre conforme testar

# quando o erro da linha (graus) ultrapassa este limite, positivo ou
# negativo, a correção usa um KP maior (curva mais fechada)
ERRO_LIMITE_KP = 90.0
