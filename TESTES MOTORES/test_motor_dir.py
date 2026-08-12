"""
Teste isolado do motor_dir, fora do resto do sistema (sem line_cam, sem
control.py, sem multiprocessing). Roda direto: python3 test_motor_dir.py

Objetivo: descobrir se "um lado manda 23 e o outro 0" é hardware
(fiação/canal da ponte BTS7960) ou alguma outra coisa -- o control.py já
manda sinais simétricos (+X / -X) pros dois motores, então se um lado não
responde, o problema está fisicamente nesse canal.

Depois de rodar, anote o que aconteceu em cada motor/direção e, se quiser,
troque os CONECTORES dos motores (não o código) entre o canal esquerdo e
o direito na ponte -- se o defeito "for junto" com o motor físico, é o
motor. Se ficar no mesmo canal da ponte, é a BTS7960/fiação desse canal.
"""

import time
from motores import PonteHBTS7960

# ajuste aqui se necessário -- mesmo mapeamento do control.py
RPWM_DIR, LPWM_DIR, REN_DIR, LEN_DIR = 12, 13, 5, 6
RPWM_ESQ, LPWM_ESQ, REN_ESQ, LEN_ESQ = 18, 19, 20, 21

VELOCIDADE_TESTE = 30.0  # %
DURACAO_TESTE = 1.5      # segundos por direção


def testar_motor(nome, motor):
    print(f"\n--- Testando {nome} ---")

    print(f"[{nome}] FRENTE ({VELOCIDADE_TESTE}%) por {DURACAO_TESTE}s...")
    motor.set_velocidade(VELOCIDADE_TESTE)
    time.sleep(DURACAO_TESTE)
    motor.set_velocidade(0)
    input(f"[{nome}] O motor girou pra frente de verdade? Anote e aperte ENTER pra continuar...")

    print(f"[{nome}] RÉ (-{VELOCIDADE_TESTE}%) por {DURACAO_TESTE}s...")
    motor.set_velocidade(-VELOCIDADE_TESTE)
    time.sleep(DURACAO_TESTE)
    motor.set_velocidade(0)
    input(f"[{nome}] O motor girou de ré de verdade? Anote e aperte ENTER pra continuar...")


def main():
    motor_dir = PonteHBTS7960(RPWM_DIR, LPWM_DIR, REN_DIR, LEN_DIR)
    motor_esq = PonteHBTS7960(RPWM_ESQ, LPWM_ESQ, REN_ESQ, LEN_ESQ)

    try:
        testar_motor("motor_dir", motor_dir)
        testar_motor("motor_esq", motor_esq)
    finally:
        motor_dir.parar()
        motor_esq.parar()
        motor_dir.fechar()
        motor_esq.fechar()
        print("\nTeste concluído, GPIO liberado.")


if __name__ == "__main__":
    main()
