"""
Script de teste unitário e validação de integração para o sistema OBR 2026.
"""

import numpy as np
import cv2

from constants import (
    FRAME_WIDTH, FRAME_HEIGHT, CENTER_X,
    LINE_LOST, LINE_TRACKING, LINE_GAP, RED_STOP,
    GREEN_NONE, GREEN_LEFT, GREEN_RIGHT, GREEN_DOUBLE, GREEN_APPROACH,
    BASE_SPEED
)
from line_cam import LineFollowerVision
from control import calcular_comando_motor
import mp_manager as mgr


def test_motor_calculations():
    print("\n--- Testando calcular_comando_motor ---")
    
    # 1. Linha centralizada
    vesq_center, vdir_center = calcular_comando_motor(LINE_TRACKING, error_x=0.0, error_angle=0.0, green_signal=GREEN_NONE, base_speed=BASE_SPEED)
    print(f"Linha Centralizada: Esq={vesq_center:.2f}%, Dir={vdir_center:.2f}%")
    assert abs(vesq_center - vdir_center) < 1e-3, "Motores devem ter velocidades iguais quando linha estiver centralizada"
    assert abs(vesq_center - BASE_SPEED) < 1e-3, f"Velocidade deve ser BASE_SPEED ({BASE_SPEED})"

    # 2. Linha para a direita (error_x > 0, error_angle > 0)
    vesq_dir, vdir_dir = calcular_comando_motor(LINE_TRACKING, error_x=40.0, error_angle=20.0, green_signal=GREEN_NONE, base_speed=BASE_SPEED)
    print(f"Linha à Direita: Esq={vesq_dir:.2f}%, Dir={vdir_dir:.2f}%")
    assert vesq_dir > vdir_dir, "Motor esquerdo deve ser mais rápido que direito para curvar à direita"

    # 3. Linha para a esquerda (error_x < 0, error_angle < 0)
    vesq_esq, vdir_esq = calcular_comando_motor(LINE_TRACKING, error_x=-40.0, error_angle=-20.0, green_signal=GREEN_NONE, base_speed=BASE_SPEED)
    print(f"Linha à Esquerda: Esq={vesq_esq:.2f}%, Dir={vdir_esq:.2f}%")
    assert vdir_esq > vesq_esq, "Motor direito deve ser mais rápido que esquerdo para curvar à esquerda"

    # 4. Aproximação de verde
    vesq_app, vdir_app = calcular_comando_motor(LINE_TRACKING, error_x=0.0, error_angle=0.0, green_signal=GREEN_APPROACH, base_speed=BASE_SPEED)
    print(f"Aproximação de Verde: Esq={vesq_app:.2f}%, Dir={vdir_app:.2f}%")
    assert vesq_app < vesq_center, "Velocidade na aproximação de verde deve ser menor que a base normal"

    # 5. Curva fechada de 90° com Contra-Rotação
    vesq_90, vdir_90 = calcular_comando_motor(LINE_TRACKING, error_x=80.0, error_angle=50.0, green_signal=GREEN_NONE, base_speed=BASE_SPEED)
    print(f"Curva Fechada 90° (Contra-Rotação): Esq={vesq_90:.2f}%, Dir={vdir_90:.2f}%")
    assert vesq_90 > 0 and vdir_90 < 0, "Curva de 90° deve acionar contra-rotação fluida da roda interna (ré suave)"

    # 6. GAP / Linha perdida
    vesq_gap, vdir_gap = calcular_comando_motor(LINE_GAP, error_x=0.0, error_angle=0.0, green_signal=GREEN_NONE, base_speed=BASE_SPEED)
    print(f"GAP / Linha Perdida: Esq={vesq_gap:.2f}%, Dir={vdir_gap:.2f}%")
    assert abs(vesq_gap - vdir_gap) < 1e-3, "Em GAP deve seguir reto"

    print("[OK] Testes de motor passaram com sucesso!")


def test_pid_derivative_damping():
    print("\n--- Testando Amortecimento Derivativo do ControladorPID ---")
    from control import ControladorPID

    pid = ControladorPID(kd=0.3, alpha_derivada=0.5)

    # 1. Ponto inicial: t=0.0s, erro=0
    vesq_0, vdir_0 = pid.calcular(LINE_TRACKING, error_x=0.0, error_angle=0.0, green_signal=GREEN_NONE, timestamp=0.0)

    # 2. Erro aumentando rapidamente para a direita (t=0.02s, error_x=40)
    #    A derivada é positiva, portanto deve somar correção extra para curva rápida
    vesq_subindo, vdir_subindo = pid.calcular(LINE_TRACKING, error_x=40.0, error_angle=20.0, green_signal=GREEN_NONE, timestamp=0.02)
    correcao_subindo = vesq_subindo - vdir_subindo

    # Criamos um PID estático (sem D) para comparar
    pid_sem_d = ControladorPID(kd=0.0)
    vesq_p_only, vdir_p_only = pid_sem_d.calcular(LINE_TRACKING, error_x=40.0, error_angle=20.0, green_signal=GREEN_NONE, timestamp=0.02)
    correcao_p_only = vesq_p_only - vdir_p_only

    print(f"Erro Subindo (t=0.02s): Correção PID={correcao_subindo:.2f} vs P puro={correcao_p_only:.2f}")
    assert correcao_subindo > correcao_p_only, "Termo derivativo deve amplificar a correção quando o erro está aumentando"

    # 3. Erro diminuindo (robô retornando ao centro: error_x cai de 40 para 10 em t=0.04s)
    #    A derivada é negativa, portanto deve amortecer a curva para não ultrapassar (overshoot)
    vesq_descendo, vdir_descendo = pid.calcular(LINE_TRACKING, error_x=10.0, error_angle=5.0, green_signal=GREEN_NONE, timestamp=0.04)
    vesq_p_descendo, vdir_p_descendo = pid_sem_d.calcular(LINE_TRACKING, error_x=10.0, error_angle=5.0, green_signal=GREEN_NONE, timestamp=0.04)
    correcao_descendo = vesq_descendo - vdir_descendo
    correcao_p_descendo = vesq_p_descendo - vdir_p_descendo

    print(f"Erro Descendo (t=0.04s): Correção PID={correcao_descendo:.2f} vs P puro={correcao_p_descendo:.2f}")
    assert correcao_descendo < correcao_p_descendo, "Termo derivativo deve amortecer a curva quando o robô está voltando ao centro"

    # 4. Teste de Reset (Anti-Kick)
    pid.reset()
    assert pid.ultimo_erro is None, "Reset deve limpar o último erro"
    assert pid.ultimo_tempo is None, "Reset deve limpar o último tempo"

    print("[OK] Teste de amortecimento derivativo passou com 100% de precisão!")


def test_vision_processing():
    print("\n--- Testando LineFollowerVision com Imagens Sintéticas ---")
    vision = LineFollowerVision(FRAME_WIDTH, FRAME_HEIGHT)

    # 1. Imagem com fita preta reta centralizada
    img_center = np.ones((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8) * 255
    cv2.line(img_center, (CENTER_X, FRAME_HEIGHT), (CENTER_X, 0), (0, 0, 0), 20)
    status, center_pt, angle, err_x, err_ang, green_sig, red_flag = vision.process_frame(img_center)
    print(f"Linha Central: Status={status}, Ponto={center_pt}, Angulo={angle:.1f}°, ErrX={err_x:.1f}, ErrAng={err_ang:.1f}°")
    assert status == LINE_TRACKING, "Deveria rastrear a linha central"
    assert abs(err_x) < 5.0, f"Erro X deveria ser próximo de 0, obteve {err_x}"
    assert abs(err_ang) < 5.0, f"Erro Angular deveria ser próximo de 0, obteve {err_ang}"

    # 2. Imagem com fita preta inclinada para a direita
    img_right = np.ones((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8) * 255
    cv2.line(img_right, (CENTER_X, FRAME_HEIGHT), (CENTER_X + 60, 0), (0, 0, 0), 20)
    status, center_pt, angle, err_x, err_ang, green_sig, red_flag = vision.process_frame(img_right)
    print(f"Linha Inclinada Direita: Status={status}, Ponto={center_pt}, Angulo={angle:.1f}°, ErrX={err_x:.1f}, ErrAng={err_ang:.1f}°")
    assert status == LINE_TRACKING
    assert err_x > 0, f"ErrX deveria ser positivo para linha à direita, obteve {err_x}"
    assert err_ang > 0, f"ErrAng deveria ser positivo para linha inclinada à direita, obteve {err_ang}"

    # 3. Imagem com fita preta inclinada para a esquerda
    img_left = np.ones((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8) * 255
    cv2.line(img_left, (CENTER_X, FRAME_HEIGHT), (CENTER_X - 60, 0), (0, 0, 0), 20)
    status, center_pt, angle, err_x, err_ang, green_sig, red_flag = vision.process_frame(img_left)
    print(f"Linha Inclinada Esquerda: Status={status}, Ponto={center_pt}, Angulo={angle:.1f}°, ErrX={err_x:.1f}, ErrAng={err_ang:.1f}°")
    assert status == LINE_TRACKING
    assert err_x < 0, f"ErrX deveria ser negativo para linha à esquerda, obteve {err_x}"
    assert err_ang < 0, f"ErrAng deveria ser negativo para linha inclinada à esquerda, obteve {err_ang}"

    # 4. Imagem com fita preta e marcador verde à direita (colado na linha)
    vision_r = LineFollowerVision(FRAME_WIDTH, FRAME_HEIGHT)
    img_green_right = np.ones((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8) * 255
    cv2.line(img_green_right, (CENTER_X, FRAME_HEIGHT), (CENTER_X, 0), (0, 0, 0), 20)
    # Bloco verde encostado na fita preta (x=160 a 200, y=140 a 180)
    green_bgr = (40, 200, 40)
    cv2.rectangle(img_green_right, (CENTER_X, 140), (CENTER_X + 45, 185), green_bgr, -1)
    status, center_pt, angle, err_x, err_ang, green_sig, red_flag = vision_r.process_frame(img_green_right)
    print(f"Marcador Verde Direita: Status={status}, Sinal={green_sig}")

    # 5. Imagem com fita preta e marcador verde à esquerda (colado na linha)
    vision_l = LineFollowerVision(FRAME_WIDTH, FRAME_HEIGHT)
    img_green_left = np.ones((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8) * 255
    cv2.line(img_green_left, (CENTER_X, FRAME_HEIGHT), (CENTER_X, 0), (0, 0, 0), 20)
    cv2.rectangle(img_green_left, (CENTER_X - 45, 140), (CENTER_X, 185), green_bgr, -1)
    status, center_pt, angle, err_x, err_ang, green_sig, red_flag = vision_l.process_frame(img_green_left)
    print(f"Marcador Verde Esquerda: Status={status}, Sinal={green_sig}")

    # 6. Imagem com fita vermelha (Stop)
    vision_r = LineFollowerVision(FRAME_WIDTH, FRAME_HEIGHT)
    img_red = np.ones((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8) * 255
    cv2.rectangle(img_red, (20, 50), (FRAME_WIDTH - 20, 150), (0, 0, 240), -1)
    status, center_pt, angle, err_x, err_ang, green_sig, red_flag = vision_r.process_frame(img_red)
    print(f"Pista Vermelha: RedDetected={red_flag}, Status={status}")
    assert red_flag is True, "Deveria detectar a fita vermelha"

    print("[OK] Testes de visão computacional passaram com sucesso!")


if __name__ == "__main__":
    test_motor_calculations()
    test_pid_derivative_damping()
    test_vision_processing()
    print("\n TODOS OS TESTES FORAM CONCLUÍDOS COM SUCESSO! ")
