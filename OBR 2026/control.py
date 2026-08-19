<<<<<<< HEAD


=======
>>>>>>> origin/main
import time

from motores import PonteHBTS7960
import mp_manager as mgr
from constants import FRAME_WIDTH, LINE_LOST, TEMPO_VIRADA, TEMPO_GIRO_ERRO, ERRO_LIMITE_KP

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
# avançar nem recuar) pro lado do erro.
# Por enquanto é por TEMPO FIXO (TEMPO_GIRO_ERRO, em constants.py): não
# fica esperando o ROI_TOPO_CENTRO reencontrar a linha (mgr.centro_topo_ok)
# -- só gira por esse tempo e volta a seguir a linha normalmente.
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

<<<<<<< HEAD
        self.ultimo_erro = None
        self.ultimo_tempo = None
        self.derivada_filtrada = 0.0
        self.integral = 0.0

    def reset(self):
        """Reseta histórico temporal e acumuladores para prevenir picos (derivative kick)."""
        self.ultimo_erro = None
        self.ultimo_tempo = None
        self.derivada_filtrada = 0.0
        self.integral = 0.0

    def calcular(
        self,
        line_status,
        error_x,
        error_angle,
        green_signal,
        base_speed=BASE_SPEED,
        timestamp=None
    ):
        """
        Calcula as velocidades percentuais (esq, dir) para os motores de forma contínua e suave.
        """
        # 1. Se a linha foi perdida ou em GAP, reseta histórico e avança com velocidade calibrada de GAP
        if line_status in (LINE_LOST, LINE_GAP):
            self.reset()
            return VEL_AVANCO_GAP, VEL_AVANCO_GAP

        # 2. Se estiver aproximando de marcador verde, reduz velocidade base
        if green_signal == GREEN_APPROACH:
            base_speed = BASE_SPEED_APPROACH

        agora = time.perf_counter() if timestamp is None else timestamp

        # 3. Normalização dos erros:
        #    norm_x: positivo quando a linha está à direita do centro
        #    norm_ang: positivo quando a linha aponta para a direita (ângulo > 90)
        norm_x = error_x / float(CENTER_X)
        norm_ang = error_angle / 90.0

        # Termo Proporcional Combinado
        erro_p = (self.kp_pos * norm_x) + (self.kp_ang * norm_ang)

        # Ganho progressivo para desvios acentuados
        if abs(erro_p) > ERRO_LIMITE_KP:
            fator = self.kp_alto / max(self.kp_pos, 1.0)
            erro_p *= fator

        # 4. Cálculo dos termos Derivativo e Integral com proteção de dt
        termo_d = 0.0
        termo_i = 0.0

        if self.ultimo_tempo is not None and self.ultimo_erro is not None:
            dt = agora - self.ultimo_tempo
            if 0.001 <= dt <= 0.2:
                # Taxa de variação do erro (derivada)
                derivada_bruta = (erro_p - self.ultimo_erro) / dt

                # Filtro passa-baixas EMA para atenuar ruído de jitter da câmera
                self.derivada_filtrada = (self.alpha_derivada * self.derivada_filtrada) + (
                    (1.0 - self.alpha_derivada) * derivada_bruta
                )
                termo_d = self.kd * self.derivada_filtrada

                # Acumulador Integral com Anti-Windup
                self.integral += erro_p * dt
                self.integral = max(-self.limite_integral, min(self.limite_integral, self.integral))
                termo_i = self.ki * self.integral

        self.ultimo_erro = erro_p
        self.ultimo_tempo = agora

        # 5. Modulação Dinâmica da Velocidade Base
        # Reduz progressivamente o avanço central conforme o erro aumenta,
        # liberando torque para rotação fluida sem parar bruscamente o robô
        mag_erro = abs(erro_p)
        fator_vel = float(np.interp(mag_erro, [0.0, 0.25, 0.60, 1.0], [1.0, 0.85, 0.50, 0.20]))
        base_efetiva = base_speed * fator_vel

        # 6. Esforço de Giro e Contra-Rotação Proporcional
        # Permite que a roda interna vá suavemente para ré quando em curvas fechadas
        esforco_giro = (erro_p + termo_d + termo_i) * (base_speed * self.fator_contra_rotacao)

        vel_esq = base_efetiva + esforco_giro
        vel_dir = base_efetiva - esforco_giro

        # Limitação dentro da faixa de operação [-100, 100]
        vel_esq = max(-100.0, min(100.0, vel_esq))
        vel_dir = max(-100.0, min(100.0, vel_dir))

        return vel_esq, vel_dir

KP = 1.2
KP_ALTO = 2.0  # usado quando abs(erro) > ERRO_LIMITE_KP -- ajuste conforme testar
BASE_SPEED = 30.0
CENTER_X = FRAME_WIDTH // 2
VEL_VIRADA = 30.0

# quando abs(erro) ultrapassa isso (positivo = direita, negativo =
# esquerda), em vez da correção proporcional o robô entra numa manobra de
# giro: pivô no próprio eixo (roda esq/dir com sinais opostos, sem
# avançar nem recuar) pro lado do erro. O giro fica reavaliando o erro a
# cada ciclo e só para quando ele cai até ERRO_ALVO_GIRO (ou no timeout
# de segurança TIMEOUT_GIRO_ERRO, ambos em constants.py) -- ver
# executar_correcao_erro_grande().
LIMITE_ERRO_GIRO = 127.0
VEL_GIRO = 30.0  # intensidade do giro nas rodas (pivô no próprio eixo) -- ajuste conforme testar



# Instância global para chamadas diretas ou testes
_controlador_global = ControladorPID()


def calcular_comando_motor(
    line_status,
    error_x,
    error_angle,
    green_signal,
    base_speed=BASE_SPEED,
    timestamp=None
):
    """Interface funcional compatível com o loop de controle e testes."""
    return _controlador_global.calcular(
        line_status,
        error_x,
        error_angle,
        green_signal,
        base_speed=base_speed,
        timestamp=timestamp
    )


def executar_manobra_verde(motor_esq, motor_dir, sinal_verde, pid_controller=None):
    """
    Executa a manobra correspondente ao marcador verde detectado:
    - GREEN_LEFT: Pequeno avanço seguido de pivô à esquerda de 90°
    - GREEN_RIGHT: Pequeno avanço seguido de pivô à direita de 90°
    - GREEN_DOUBLE: Meia-volta de 180° (pivô duplo)
    """
    if sinal_verde == GREEN_DOUBLE:
        print("[control] Executando manobra de DUPLO VERDE (180°)...")
        motor_esq.set_velocidade(VEL_AVANCO_VERDE)
        motor_dir.set_velocidade(VEL_AVANCO_VERDE)
        time.sleep(TEMPO_AVANCO_VERDE * 0.5)

        # Pivô de 180° (gira para a esquerda)
        motor_esq.set_velocidade(-VEL_GIRO_VERDE)
        motor_dir.set_velocidade(VEL_GIRO_VERDE)
        t_ini = time.time()
        while (time.time() - t_ini < TEMPO_GIRO_VERDE * 1.5) and not mgr.terminate.is_set():
            time.sleep(0.005)

        # Continua girando até re-encontrar a linha
        while (time.time() - t_ini < TIMEOUT_MANOBRA_VERDE) and not mgr.terminate.is_set():
            if mgr.line_status.value == LINE_TRACKING and abs(mgr.error_angle.value) <= ERRO_ALVO_GIRO:
                break
            time.sleep(0.005)

        motor_esq.set_velocidade(0)
        motor_dir.set_velocidade(0)
        print("[control] Duplo verde concluído.")
        time.sleep(COOLDOWN_DUPLO_VERDE)

    elif sinal_verde == GREEN_LEFT:
        print("[control] Executando manobra de VERDE ESQUERDA (90°)...")
        motor_esq.set_velocidade(VEL_AVANCO_VERDE)
        motor_dir.set_velocidade(VEL_AVANCO_VERDE)
        t_ini = time.time()
        while (time.time() - t_ini < TEMPO_AVANCO_VERDE) and not mgr.terminate.is_set():
            time.sleep(0.005)

        motor_esq.set_velocidade(-VEL_GIRO_VERDE)
        motor_dir.set_velocidade(VEL_GIRO_VERDE)
        t_ini = time.time()
        while (time.time() - t_ini < TEMPO_GIRO_VERDE) and not mgr.terminate.is_set():
            time.sleep(0.005)

        while (time.time() - t_ini < TIMEOUT_MANOBRA_VERDE) and not mgr.terminate.is_set():
            if mgr.line_status.value == LINE_TRACKING and abs(mgr.error_angle.value) <= ERRO_ALVO_GIRO:
                break
            time.sleep(0.005)

        motor_esq.set_velocidade(0)
        motor_dir.set_velocidade(0)
        print("[control] Curva verde à esquerda concluída.")
        time.sleep(COOLDOWN_VERDE)

    elif sinal_verde == GREEN_RIGHT:
        print("[control] Executando manobra de VERDE DIREITA (90°)...")
        motor_esq.set_velocidade(VEL_AVANCO_VERDE)
        motor_dir.set_velocidade(VEL_AVANCO_VERDE)
        t_ini = time.time()
        while (time.time() - t_ini < TEMPO_AVANCO_VERDE) and not mgr.terminate.is_set():
            time.sleep(0.005)

        motor_esq.set_velocidade(VEL_GIRO_VERDE)
        motor_dir.set_velocidade(-VEL_GIRO_VERDE)
        t_ini = time.time()
        while (time.time() - t_ini < TEMPO_GIRO_VERDE) and not mgr.terminate.is_set():
            time.sleep(0.005)

        while (time.time() - t_ini < TIMEOUT_MANOBRA_VERDE) and not mgr.terminate.is_set():
            if mgr.line_status.value == LINE_TRACKING and abs(mgr.error_angle.value) <= ERRO_ALVO_GIRO:
                break
            time.sleep(0.005)

        motor_esq.set_velocidade(0)
        motor_dir.set_velocidade(0)
        print("[control] Curva verde à direita concluída.")
        time.sleep(COOLDOWN_VERDE)

    if pid_controller is not None:
        pid_controller.reset()


def executar_correcao_erro_grande(motor_esq, motor_dir, pid_controller=None):
    """
    Recuperação de curva extrema de 90° em L:
    1. Pré-avanço de vértice (TEMPO_AVANCO_VERTICE_90) para trazer o eixo das rodas sobre o canto.
    2. Pivô proporcional desacelerado: velocidade de giro decresce com o erro,
       garantindo travamento suave sobre a linha sem overshoot.
    """
    erro_ang = mgr.error_angle.value
    lado = "direita" if erro_ang > 0 else "esquerda"

    print(f"[control] Curva fechada de 90° ({erro_ang:.1f}° para {lado}). Executando transição suave...")

    # 1. Alinhamento de eixo das rodas sobre o vértice da curva
    motor_esq.set_velocidade(BASE_SPEED_APPROACH)
    motor_dir.set_velocidade(BASE_SPEED_APPROACH)
    t_ini_avanco = time.time()
    while (time.time() - t_ini_avanco < TEMPO_AVANCO_VERTICE_90) and not mgr.terminate.is_set():
=======
    motor_esq.set_velocidade(-VEL_VIRADA)
    motor_dir.set_velocidade(VEL_VIRADA)
    t_ini_giro = time.time()
    while (time.time() - t_ini_giro < TEMPO_VIRADA
           and not mgr.terminate.is_set()):
>>>>>>> origin/main
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
    nenhum viés pra frente/trás. A direção é fixada UMA VEZ no início da
    manobra (não recalcula a cada ciclo em cima de um erro instável).

    Por enquanto é por TEMPO FIXO (TEMPO_GIRO_ERRO): gira só por esse
    tempo e depois volta a seguir a linha normalmente, sem checar se
    achou o centro de novo."""
    erro_inicial = mgr.line_angle.value

    if erro_inicial > 0:
        vel_esq, vel_dir, lado = VEL_GIRO, -VEL_GIRO, "direita"
    else:
        vel_esq, vel_dir, lado = -VEL_GIRO, VEL_GIRO, "esquerda"

    print(f"[control] Erro {erro_inicial:.0f} >= {LIMITE_ERRO_GIRO}, girando pra {lado} "
          f"por {TEMPO_GIRO_ERRO}s...")

    motor_esq.set_velocidade(vel_esq)
    motor_dir.set_velocidade(vel_dir)

    t_ini = time.time()
    while (time.time() - t_ini < TEMPO_GIRO_ERRO
           and not mgr.terminate.is_set()):
        time.sleep(0.005)

    motor_esq.set_velocidade(0)
    motor_dir.set_velocidade(0)

    if mgr.terminate.is_set():
        print("[control] Giro interrompido (encerramento do programa).")
    else:
        print("[control] Giro concluído, retomando seguimento normal.")


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
