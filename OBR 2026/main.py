"""
Ponto de entrada principal para o robô seguidor de linha - OBR 2026.
Inicia e gerencia os processos concorrentes de visão computacional (line_cam)
e controle de motores (control).
"""

import sys
import time
from multiprocessing import set_start_method, get_start_method, Process

import mp_manager as mgr
from line_cam import capturar_e_processar
from control import loop_controle


def main():
    print("=" * 60)
    print("  OBR 2026 - Robô Seguidor de Linha")
    print("=" * 60)

    # Configuração de multiprocessamento compatível com Linux e Windows
    if sys.platform != "win32":
        try:
            if get_start_method(allow_none=True) != "fork":
                set_start_method("fork")
        except RuntimeError:
            pass

    mgr.reset_estado()

    # Processo de Percepção / Visão Computacional
    proc_camera = Process(
        target=capturar_e_processar,
        name="line_cam",
        daemon=True
    )

    # Processo de Controle de Motores
    proc_motores = Process(
        target=loop_controle,
        name="control",
        daemon=True
    )

    processos = [proc_camera, proc_motores]

    print("[main] Iniciando processos...")
    for p in processos:
        p.start()
        print(f"[main] Processo '{p.name}' iniciado com PID {p.pid}.")

    try:
        while not mgr.terminate.is_set() and all(p.is_alive() for p in processos):
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[main] Sinal de interrupção recebido (Ctrl+C). Encerrando...")

    finally:
        mgr.terminate.set()

        print("[main] Aguardando encerramento dos processos...")
        for p in processos:
            p.join(timeout=1.5)

        for p in processos:
            if p.is_alive():
                print(f"[main] Forçando encerramento de '{p.name}'...")
                p.terminate()
                p.join(timeout=0.5)

        print("[main] Todos os processos foram finalizados com segurança.")


if __name__ == "__main__":
    main()
