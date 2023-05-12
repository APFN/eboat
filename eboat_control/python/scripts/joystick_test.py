import pygame
import time

def main():
    pygame.init()
    pygame.joystick.init()

    num_joysticks = pygame.joystick.get_count()
    if num_joysticks == 0:
        print("Nenhum joystick conectado.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    joystick_name = joystick.get_name()
    joystick_id = joystick.get_id()

    print(f"Joystick {joystick_name} (Entrada: {joystick_id})")

    try:
        
        while True:
            pygame.event.pump()

            # Obtém os valores dos eixos
            for i in range(joystick.get_numaxes()):
                axis = joystick.get_axis(i)
                print(f"Eixo {i}: {axis:.2f}")

            # Obtém os valores dos botões
            for i in range(joystick.get_numbuttons()):
                button = joystick.get_button(i)
                print(f"Botão {i}: {button}")

            # Obtém os valores dos hats (direcionais)
            for i in range(joystick.get_numhats()):
                hat = joystick.get_hat(i)
                print(f"Hat {i}: {hat}")

            print("--------------------")             
            time.sleep(1)

    except KeyboardInterrupt:
        print("Programa encerrado pelo usuário.")

    finally:
        joystick.quit()
        pygame.quit()


def find_joystick():
    pygame.init()
    pygame.joystick.init()

    num_joysticks = pygame.joystick.get_count()
    if num_joysticks == 0:
        print("Nenhum joystick encontrado.")
        return

    print(f"Total de joysticks encontrados: {num_joysticks}")

    for i in range(num_joysticks):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        joystick_name = joystick.get_name()
        joystick_id = joystick.get_id()
        print(f"Joystick {i}: {joystick_name} (Entrada: {joystick_id})")

    pygame.quit()

if __name__ == "__main__":
    main()    
    # find_joystick()
