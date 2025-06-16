import socket
import time
import pygame

# 初始化 Pygame
pygame.init()
pygame.joystick.init()
# pygame.display.set_mode((1, 1))

# 檢查是否有搖桿
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("未檢測到搖桿")
    exit()

# 初始化第一個搖桿
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"已連接到搖桿: {joystick.get_name()}")

# Set up TCP server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 5000))  # Listen on all interfaces
server_socket.listen(1)

print("Waiting for ESP8266 connection...")

try:
    while True:
        print("Listening for new connection...")
        client_socket, addr = server_socket.accept()
        print(f"Connected by {addr}")

        try:
            while True:
                # Send data to ESP8266
                        # 處理事件
                msg = ""
                # for event in pygame.event.get():
                #     if event.type == pygame.JOYAXISMOTION:
                #         # 搖桿軸（例如左搖桿 X/Y 軸）
                #         print(f"軸 {event.axis} 值: {event.value:.2f}")
                #         match event.axis:
                #             case 0:
                #                 if event.value > 0:
                #                     msg = "d"
                #                 elif event.value < 0:
                #                     msg = "a"
                #             case 1:
                #                 if event.value < 0:
                #                     msg = "w"
                #                 elif event.value > 0:
                #                     msg = "s"
                            
                #     elif event.type == pygame.JOYBUTTONDOWN:
                #         # 按鈕按下
                #         print(f"按鈕 {event.button} 按下")
                #         if event.button == 1:
                #             msg = "e"
                #         elif event.button == 3:
                #             msg = "q"
                #         else:
                #             msg = "f" #即停                            
                #     elif event.type == pygame.JOYHATMOTION:
                #         # 方向鍵（D-Pad）
                #         print(f"方向鍵 值: {event.value}")

                pygame.event.pump()

                lx = -joystick.get_axis(0)  # 左搖桿左右 -> vy
                ly = -joystick.get_axis(2)  # 左搖桿上下（取反）-> vx
                rx = joystick.get_button(3) - joystick.get_button(1)
                print(lx, ly, rx)
                vx = ly * 0.2
                vy = lx * 0.2
                w = rx * 0.6

                msg = f"{vx:.2f},{vy:.2f},{w:.2f}"

                client_socket.sendall((msg + '\r\n').encode())
                time.sleep(0.1)

                # Optionally receive response
                '''
                data = client_socket.recv(1024)
                if not data:
                    print("ESP8266 disconnected.")
                    break  # exit inner loop to relisten
                print("ESP8266 says:", data.decode())
                '''

        except (ConnectionResetError, BrokenPipeError):
            print("Connection lost. Relistening...")
        except KeyboardInterrupt:
            print("退出程式")
            break
        finally:
            client_socket.close()
            pygame.quit()
except KeyboardInterrupt:
    print("Server shutting down.")
finally:
    server_socket.close()