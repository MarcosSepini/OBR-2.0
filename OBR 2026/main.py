
import time
from multiprocessing import set_start_method, get_start_method, Process

import mp_manager as mgr
from line_cam import capturar_e_processar
from control import loop_controle
from led_branco import led_branco_loop


def main():
    if get_start_method(allow_none=True) != "fork":
        set_start_method("fork")

    proc_camera = Process(
        target=capturar_e_processar,
        name="line_cam"
    )

    proc_motores = Process(
        target=loop_controle,
        name="control"
    )

    # proc_led = Process(target=led_branco_loop, name="led")

    processos = [proc_camera, proc_motores]

    for p in processos:
        p.start()

    try:
        while all(p.is_alive() for p in processos):
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Encerrando (Ctrl+C)...")

    finally:
        mgr.terminate.set()

        for p in processos:
            p.join(timeout=2)

        for p in processos:
            if p.is_alive():
                p.terminate()
                p.join()

        # Libera a memia compartilhada
        try:
            mgr.shm.close()
        except Exception:
            pass

        try:
            mgr.shm.unlink()
        except FileNotFoundError:
            pass
        except Exception as e:
            print(f"Erro ao liberar shared_memory: {e}")


if __name__ == "__main__":
    main()
