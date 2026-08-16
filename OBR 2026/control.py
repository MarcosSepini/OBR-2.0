"""
Módulo de controle dos motores BTS7960 para o robô seguidor de linha - OBR 2026.
Implementa controlador PID avançado com modulação dinâmica de velocidade base,
contra-rotação proporcional para curvas de 90° e pivô proporcional desacelerado.
"""

import time
import numpy as np

from motores import PonteHBTS7960
import mp_manager as mgr
from constants import (
    FRAME_WIDTH, CENTER_X,
    RPWM_ESQ, LPWM_ESQ, REN_ESQ, LEN_ESQ,
    RPWM_DIR, LPWM_DIR, REN_DIR, LEN_DIR,
    LINE_LOST, LINE_TRACKING, LINE_GAP, RED_STOP,
    GREEN_NONE, GREEN_LEFT, GREEN_RIGHT, GREEN_DOUBLE, GREEN_APPROACH,
    BASE_SPEED, BASE_SPEED_APPROACH,
    KP_POS, KP_ANG, KP_ALTO, KD, KI, ALPHA_DERIVADA, LIMITE_INTEGRAL, ERRO_LIMITE_KP,
    FATOR_CURVA_CONTRA_ROTACAO,
    LIMITE_ERRO_GIRO, ERRO_ALVO_GIRO, TIMEOUT_GIRO_ERRO,
    KP_GIRO, MIN_VEL_GIRO, MAX_VEL_GIRO, TEMPO_AVANCO_VERTICE_90,
    VEL_AVANCO_VERDE, TEMPO_AVANCO_VERDE, VEL_GIRO_VERDE, TEMPO_GIRO_VERDE,
    TIMEOUT_MANOBRA_VERDE, COOLDOWN_VERDE, COOLDOWN_DUPLO_VERDE,
    VEL_AVANCO_GAP, TIMEOUT_GAP_BUSCA, TIMEOUT_PERDA_LINHA_SEGURANCA
)


class ControladorPID:
    """
    Controlador PID avançado para seguimento de linha:
    - P (Proporcional): Combina o desvio lateral do centro (X) e a inclinação angular da linha.
    - D (Derivativo com Filtro Passa-Baixas): Amortece oscilações rápidas (efeito zig-zag/rebolado) sem ruído.
    - I (Integral com Anti-Windup): Elimina erros de estado estacionário em curvas longas.
    - Modulação Dinâmica de Velocidade Base: Desacelera o avanço em curvas acentuadas, permitindo contra-rotação suave.
    - Proteção Anti-Kick: Previne saltos abruptos de derivada ao retornar de manobras ou gaps.
    """

    def __init__(
        self,
        kp_pos=KP_POS,
        kp_ang=KP_ANG,
        kp_alto=KP_ALTO,
        kd=KD,
        ki=KI,
        alpha_derivada=ALPHA_DERIVADA,
        limite_integral=LIMITE_INTEGRAL,
        fator_contra_rotacao=FATOR_CURVA_CONTRA_ROTACAO
    ):
        self.kp_pos = kp_pos
        self.kp_ang = kp_ang
        self.kp_alto = kp_alto
        self.kd = kd
        self.ki = ki
        self.alpha_derivada = alpha_derivada
        self.limite_integral = limite_integral
        self.fator_contra_rotacao = fator_contra_rotacao

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
        time.sleep(0.005)

    # 2. Pivô Proporcional Desacelerado
    t_ini = time.time()
    while not mgr.terminate.is_set():
        if (time.time() - t_ini) >= TIMEOUT_GIRO_ERRO:
            break

        erro_atual = mgr.error_angle.value
        mag_erro = abs(erro_atual)

        if mgr.line_status.value == LINE_TRACKING and mag_erro <= ERRO_ALVO_GIRO:
            break

        # Velocidade de giro desacelera suavemente conforme o erro diminui
        vel_giro = max(MIN_VEL_GIRO, min(MAX_VEL_GIRO, KP_GIRO * mag_erro))

        if erro_atual > 0:
            vel_esq, vel_dir = vel_giro, -vel_giro
        else:
            vel_esq, vel_dir = -vel_giro, vel_giro

        motor_esq.set_velocidade(vel_esq)
        motor_dir.set_velocidade(vel_dir)
        time.sleep(0.005)

    motor_esq.set_velocidade(0)
    motor_dir.set_velocidade(0)

    if pid_controller is not None:
        pid_controller.reset()


def loop_controle():
    """
    Loop principal de controle dos motores com PID dinâmico.
    Consome os dados de mp_manager e guia as pontes H BTS7960.
    """
    print("[control] Inicializando drivers dos motores...")
    try:
        motor_esq = PonteHBTS7960(RPWM_ESQ, LPWM_ESQ, REN_ESQ, LEN_ESQ)
        motor_dir = PonteHBTS7960(RPWM_DIR, LPWM_DIR, REN_DIR, LEN_DIR)
        print("[control] Motores inicializados com sucesso.")
    except Exception as e:
        print(f"[control] ERRO ao inicializar motores (GPIO): {e}")
        return

    pid = ControladorPID()
    cooldown_verde_fim = 0.0
    t_linha_perdida_inicio = None

    try:
        while not mgr.terminate.is_set():
            # 1. Trava se a câmera não estiver pronta
            if not mgr.camera_ok.value:
                pid.reset()
                motor_esq.set_velocidade(0)
                motor_dir.set_velocidade(0)
                time.sleep(0.03)
                continue

            # 2. Parada de emergência / Faixa vermelha
            if mgr.red_detected.value or mgr.line_status.value == RED_STOP:
                print("[control] Vermelho detectado! Parada de pista.")
                pid.reset()
                motor_esq.set_velocidade(0)
                motor_dir.set_velocidade(0)
                time.sleep(0.1)
                continue

            # 2.1 Trava de segurança: Linha perdida por tempo excessivo
            if mgr.line_status.value == LINE_LOST:
                if t_linha_perdida_inicio is None:
                    t_linha_perdida_inicio = time.time()
                elif (time.time() - t_linha_perdida_inicio) > TIMEOUT_PERDA_LINHA_SEGURANCA:
                    motor_esq.set_velocidade(0)
                    motor_dir.set_velocidade(0)
                    time.sleep(0.05)
                    continue
            else:
                t_linha_perdida_inicio = None

            # 3. Manobras de Marcadores Verdes (se fora do cooldown)
            sinal_verde = mgr.green_signal.value
            agora = time.time()
            if sinal_verde in (GREEN_LEFT, GREEN_RIGHT, GREEN_DOUBLE) and agora >= cooldown_verde_fim:
                executar_manobra_verde(motor_esq, motor_dir, sinal_verde, pid_controller=pid)
                cooldown_verde_fim = time.time() + COOLDOWN_VERDE
                continue

            # 4. Recuperação de Curvas Acentuadas de 90° (Pivô suave desacelerado)
            if mgr.line_status.value == LINE_TRACKING and abs(mgr.error_angle.value) >= LIMITE_ERRO_GIRO:
                executar_correcao_erro_grande(motor_esq, motor_dir, pid_controller=pid)
                continue

            # 5. Seguimento de Linha Fluido com Modulação Dinâmica de Velocidade
            vel_esq, vel_dir = pid.calcular(
                mgr.line_status.value,
                mgr.error_x.value,
                mgr.error_angle.value,
                mgr.green_signal.value,
                base_speed=BASE_SPEED
            )

            motor_esq.set_velocidade(vel_esq)
            motor_dir.set_velocidade(vel_dir)

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("[control] Interrompido pelo usuário.")
    except Exception as e:
        print(f"[control] ERRO inesperado no loop de controle: {e}")
    finally:
        motor_esq.parar()
        motor_dir.parar()
        motor_esq.fechar()
        motor_dir.fechar()
        print("[control] Motores desativados e GPIO liberado.")


if __name__ == "__main__":
    loop_controle()
