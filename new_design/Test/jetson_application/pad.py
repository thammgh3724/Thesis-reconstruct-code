import pygame
import time

def gamepad_test():
    pygame.init()
    newValue = False
    # Kiểm tra nếu có joystick nào được kết nối
    if pygame.joystick.get_count() == 0:
        print("Không có gamepad nào được kết nối.")
        return

    # Lấy joystick đầu tiên
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Đã phát hiện gamepad: {joystick.get_name()}")
    print(f"Số lượng nút: {joystick.get_numbuttons()}")
    print(f"Số lượng axes (cần điều khiển): {joystick.get_numaxes()}")
    print(f"Số lượng hats (D-pad): {joystick.get_numhats()}")

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    print(f"Button {event.button} pressed")
                    newValue = True
                elif event.type == pygame.JOYBUTTONUP:
                    print(f"Button {event.button} released")
                    newValue = True
                elif event.type == pygame.JOYAXISMOTION:
                    print(f"Axis {event.axis} moved to {event.value}")
                    newValue = True
                elif event.type == pygame.JOYHATMOTION:
                    print(f"D-pad moved to {event.value}")  # Tuple (x, y)
                    newValue = True
                else: 
                    newValue = False
            # Giảm tải CPU bằng cách thêm thời gian ngủ nhỏ
            # print(newValue)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Thoát chương trình.")
    finally:
        pygame.quit()

if __name__ == "__main__":
    gamepad_test()
