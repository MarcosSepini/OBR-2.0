import time

from motores import PonteHBTS7960
import mp_manager as mgr
from constants import (
    FRAME_WIDTH, LINE_LOST, TEMPO_VIRADA, ERRO_ALVO_GIRO, TIMEOUT_GIRO_ERRO,
    ERRO_LIMITE_KP,
)

RPWM_ESQ, LPWM_ESQ, REN_ESQ, LEN_ESQ = 12, 13, 5, 6
RPWM_DIR, LPWM_DIR, REN_DIR, LEN_DIR = 18, 19, 20, 21

KP = 0.6
KP_ALTO = 1.2  # usado quando abs(erro) > ERRO_LIMITE_KP -- ajuste conforme testar
BASE_SPEED = 60.0
CENTER_X = FRAME_WIDTH // 2
VEL_VIRADA = 60.0

# quando abs(erro) ultrapassa isso (positivo = direita, negativo =
# esquerda), em vez da correção proporcional o robô entra numa manobra de
# giro: pivô no próprio eixo (roda esq/dir com sinais opostos, sem
# avançar nem recuar) pro lado do erro. O giro fica reavaliando o erro a
# cada ciclo e só para quando ele cai até ERRO_ALVO_GIRO (ou no timeout
# de segurança TIMEOUT_GIRO_ERRO, ambos em constants.py) -- ver
# executar_correcao_erro_grande().
LIMITE_ERRO_GIRO = 130.0
VEL_GIRO = 45.0  # intensidade do giro nas rodas (pivô no próprio eixo) -- ajuste conforme testar


#################################
## MOTORES                     ##
#################################

def executar_virada(motor_esq, motor_dir):
    """Virada 'cega' de duração fixa (TEMPO_VIRADA) para a esquerda, sem
    seguir linha e sem giroscópio. Depois checa o ROI de retorno (canto
    inferior esquerdo); sem ré -- se não achar a linha, só para e espera
    ela aparecer (segue no próximo ciclo do loop de controle)."""
    print(f"[control] Iniciando virada cega de {TEMPO_VIRADA}s (saindo da linha p/ esquerda)...")

    motor_esq.set_velocidade(-VEL_VIRADA)
    motor_dir.set_velocidade(VEL_VIRADA)
    t_ini_giro = time.time()
    while (time.time() - t_ini_giro < TEMPO_VIRADA
           and not mgr.terminate.is_set()):
        time.sleep(0.005)
    motor_esq.set_velocidade(0)
    motor_dir.set_velocidade(0)
    print("[control] Virada concluída.")

    if mgr.retorno_linha_ok.value:
        print("[control] Linha encontrada no ROI de retorno, seguindo normalmente.")
    else:
        print("[control] Linha não encontrada no ROI de retorno; parei por segurança "
              "(sem ré), aguardando a linha aparecer.")


def executar_correcao_erro_grande(motor_esq, motor_dir):
    """Erro grande demais (abs(erro) >= LIMITE_ERRO_GIRO): gira NO PRÓPRIO
    EIXO (pivô puro, sem avançar nem recuar nem um pouco) pro lado do erro
    (negativo = esquerda, positivo = direita) -- rodas com mesma
    magnitude e sinais SEMPRE opostos (uma positiva, outra negativa), sem
    nenhum viés pra frente/trás. A DIREÇÃO é fixada UMA VEZ no início da
    manobra (não recalcula a cada ciclo em cima de um erro instável).

    Agora NÃO é mais por tempo fixo: as rodas ficam ligadas em pivô e o
    erro (mgr.line_angle) é reavaliado a cada ciclo -- o giro só termina
    quando abs(erro) cai até ERRO_ALVO_GIRO (erro "zerado"). Um
    TIMEOUT_GIRO_ERRO serve só de trava de segurança, pra não ficar
    girando pra sempre se a linha nunca for reencontrada."""
    erro_inicial = mgr.line_angle.value

    if erro_inicial > 0:
        vel_esq, vel_dir, lado = VEL_GIRO, -VEL_GIRO, "direita"
    else:
        vel_esq, vel_dir, lado = -VEL_GIRO, VEL_GIRO, "esquerda"

    print(f"[control] Erro {erro_inicial:.0f} >= {LIMITE_ERRO_GIRO}, girando (pivô) pra "
          f"{lado} até abs(erro) <= {ERRO_ALVO_GIRO:.0f} "
          f"(timeout de segurança: {TIMEOUT_GIRO_ERRO}s)...")

    t_ini = time.time()
    motivo_parada = "timeout"

    while not mgr.terminate.is_set():
        if time.time() - t_ini >= TIMEOUT_GIRO_ERRO:
            motivo_parada = "timeout"
            break

        # reafirma a velocidade a cada ciclo (garante que o pivô continua
        # de fato girando, e não só "seta uma vez e espera")
        motor_esq.set_velocidade(vel_esq)
        motor_dir.set_velocidade(vel_dir)

        if mgr.line_status.value != LINE_LOST and abs(mgr.line_angle.value) <= ERRO_ALVO_GIRO:
            motivo_parada = "erro_zerado"
            break

        time.sleep(0.005)

    motor_esq.set_velocidade(0)
    motor_dir.set_velocidade(0)

    if mgr.terminate.is_set():
        print("[control] Giro interrompido (encerramento do programa).")
    elif motivo_parada == "erro_zerado":
        print(f"[control] Erro caiu pra {mgr.line_angle.value:.0f} "
              f"(<= {ERRO_ALVO_GIRO:.0f}), giro concluído, retomando seguimento normal.")
    else:
        print(f"[control] Timeout de {TIMEOUT_GIRO_ERRO}s do giro atingido sem o erro cair "
              f"(erro atual: {mgr.line_angle.value:.0f}); parei por segurança e voltei ao "
              f"seguimento normal.")


def calcular_comando_motor(status, erro, center_x=CENTER_X, base=BASE_SPEED):
    if status == LINE_LOST:
        return base, base

    kp = KP_ALTO if abs(erro) > ERRO_LIMITE_KP else KP
    correcao = kp * (erro / center_x)
    vel_esq = base + correcao * base
    vel_dir = base - correcao * base

    vel_esq = max(-100.0, min(100.0, vel_esq))
    vel_dir = max(-100.0, min(100.0, vel_dir))
    return vel_esq, vel_dir


def loop_controle():

    try:
        motor_esq = PonteHBTS7960(RPWM_ESQ, LPWM_ESQ, REN_ESQ, LEN_ESQ)
        motor_dir = PonteHBTS7960(RPWM_DIR, LPWM_DIR, REN_DIR, LEN_DIR)
    except Exception as e:
        print(f"[control] ERRO ao inicializar os motores (GPIO): {e}")
        return

    print("[control] Motores inicializados, loop de controle iniciado.")

    try:
        while not mgr.terminate.is_set():
            if not mgr.camera_ok.value:
                motor_esq.set_velocidade(0)
                motor_dir.set_velocidade(0)
                time.sleep(0.05)
                continue

            if mgr.virar_flag.value:
                executar_virada(motor_esq, motor_dir)
                mgr.virar_flag.value = 0
                continue

            if (mgr.line_status.value != LINE_LOST
                    and abs(mgr.line_angle.value) >= LIMITE_ERRO_GIRO):
                executar_correcao_erro_grande(motor_esq, motor_dir)
                continue

            vel_esq, vel_dir = calcular_comando_motor(
                mgr.line_status.value, mgr.line_angle.value
            )
            motor_esq.set_velocidade(vel_esq)
            motor_dir.set_velocidade(vel_dir)
            time.sleep(0.005)
    finally:
        motor_esq.parar()
        motor_dir.parar()
        motor_esq.fechar()
        motor_dir.fechar()
        print("[control] Motores parados e GPIO liberado.")
