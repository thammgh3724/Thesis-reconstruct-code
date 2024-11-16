import threading
import pygame
import time

class GamepadHandler(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        pygame.init()
        self.joystick = None
        self.buffer = [0] * 6        # Assuming there are 6 parameters to track
        self.slide_signal = [0, 0]   # Control slide signal
        self.gripper_signal = [0, 0] # Control gripper signal
        self.stop_signal = [0]       # Stop urgent signal
        self.newValue = False
        self.mode = "gamepad"        # Default to gamepad mode
        self.debounce_time = 0.5     # Time to wait before toggling mode again
        self.last_toggle_time = 0    # Last time the mode was toggled
        self.isGoHome = True         # To check if after mode is switched, system is home yet.
        self.modeChanged = False

    def getBuffer(self):
        return self.buffer
    
    def getMode(self):
        return self.mode
    
    def getModeChanged(self):
        return self.modeChanged
    
    def getSlidersSignal(self):
        return self.slide_signal
    
    def getGripperSignal(self):
        return self.gripper_signal
    
    def getStopSignal(self):
        return self.stop_signal

    def toggle_mode(self):
        """Toggle between 'gamepad', 'hand_detect', and 'auto' modes."""
        current_time = time.time()
        if current_time - self.last_toggle_time > self.debounce_time:
            # Switch between gamepad, hand_detect, and auto
            if self.mode == "gamepad":
                self.mode = "hand_detect"
            # elif self.mode == "hand_detect":
            #     self.mode = "auto"
            else:
                self.mode = "gamepad"
            print(f"Switched mode to: {self.mode}")
            self.isGoHome = False
            self.modeChanged = True
            self.last_toggle_time = current_time


    def handle_dpad_x(self, value):
        global debounce
        if value == 1.0:
            self.slide_signal[0] = 2
        elif value == -1.0:
            self.slide_signal[0] = 1
        else:
            self.slide_signal[0] = 0
            if __debug__: print("D-pad X-axis released")
            # debounce = 0

    def handle_dpad_y(self, value):
        if value == 1.0:
            self.slide_signal[1] = 1
        elif value == -1.0:
            self.slide_signal[1] = 2
        else:
            self.slide_signal[1] = 0
            if __debug__: print("D-pad Y-axis released")

    def handle_button_press(self, button): # Chưa thêm nút chuyển đổi giữa auto và manual
        global debounce
        if button == 0:
            self.buffer[5] = 1
        elif button == 1:
            self.buffer[4] = 2
        elif button == 3:
            self.buffer[4] = 1
        elif button == 4:
            self.buffer[5] = 2
        elif button == 6:                  # Gripper control
            if __debug__: print("Left bumper pressed")
            self.gripper_signal[1] = 1
        elif button == 7:
            if __debug__: print("Right bumper pressed")
            self.gripper_signal[1] = 2
        elif button == 10:
            if __debug__: print("Back button pressed")
        elif button == 11:
            if __debug__: print("Start button pressed")
        elif button == 13:
            if __debug__: print("Left stick button pressed")
        elif button == 14:
            if __debug__: print("Right stick button pressed")
        elif button == 8: # Button to stop urgent
            if __debug__: print("Left trigger (L2) pressed")
            self.stop_signal[0] = 1
            debounce = 2
        elif button == 9: # Button to change mode, can modify
            if __debug__: print("Right trigger (R2) pressed")
            self.toggle_mode()
            debounce = 2
        else: 
            if __debug__: print("Hello there!")

    def handle_button_release(self, button):
        if button == 0:
            if __debug__: print("Button A released")
            self.buffer[5] = 0
        elif button == 1:
            if __debug__: print("Button B released")
            self.buffer[4] = 0
        elif button == 3:
            if __debug__: print("Button X released")
            self.buffer[4] = 0
        elif button == 4:
            if __debug__: print("Button Y released")
            self.buffer[5] = 0
        elif button == 6:
            if __debug__: print("Left bumper released")
            self.gripper_signal[1] = 0
        elif button == 7:
            if __debug__: print("Right bumper released")
            self.gripper_signal[1] = 0
        elif button == 10:
            if __debug__: print("Back button released")
        elif button == 11:
            if __debug__: print("Start button released")
        elif button == 13:
            if __debug__: print("Left stick button released")
        elif button == 14:
            if __debug__: print("Right stick button released")
        elif button == 8:
            if __debug__: print("Left trigger (L2) released")
        elif button == 9:
            if __debug__: print("Right trigger (R2) released")
        else: 
            if __debug__: print("Hello there!")

    def handle_axis_motion(self, axis, value):
        if axis == 0:  # X-axis of the left stick
            if value >= -1.1 and value < -0.9:
                self.buffer[0] = 1
            elif value > 0.9 and value <= 1.1:
                self.buffer[0] = 2
            elif value >= -0.2 and value <= 0.2: 
                self.buffer[0] = 0
        elif axis == 1:  # Y-axis of the left stick
            if value >= -1.1 and value < -0.9:
                self.buffer[1] = 1
            elif value > 0.9 and value <= 1.1:
                self.buffer[1] = 2
            elif value >= -0.2 and value <= 0.2: 
                self.buffer[1] = 0
        elif axis == 2:  # X-axis of the right stick
            if value >= -1.1 and value < -0.9:
                self.buffer[3] = 1
            elif value > 0.9 and value <= 1.1:
                self.buffer[3] = 2
            elif value >= -0.2 and value <= 0.2: 
                self.buffer[3] = 0
        elif axis == 3:  # Y-axis of the right stick
            if value >= -1.1 and value < -0.9:
                self.buffer[2] = 1
            elif value > 0.9 and value <= 1.1:
                self.buffer[2] = 2
            elif value >= -0.2 and value <= 0.2: 
                self.buffer[2] = 0
        elif axis == 5:  # Left trigger (L2)
            if __debug__: print(f"Left trigger (L2) value: {value}")
        elif axis == 4:  # Right trigger (R2)
            if __debug__: print(f"Right trigger (R2) value: {value}")
        elif axis == 6:  # D-pad X-axis
            self.handle_dpad_x(value)
        elif axis == 7:  # D-pad Y-axis
            self.handle_dpad_y(value)

    def run(self):
        # Check if there is a joystick
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            if __debug__: print(f"Joystick found: {self.joystick.get_name()}")

            while True:
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        self.newValue = True
                        self.handle_button_press(event.button)
                    elif event.type == pygame.JOYBUTTONUP:
                        self.newValue = False
                        self.handle_button_release(event.button)
                    elif event.type == pygame.JOYAXISMOTION:
                        self.handle_axis_motion(event.axis, event.value)
                        if self.buffer == [0] * len(self.buffer):
                            self.newValue = False
                        else:
                            self.newValue = True
                    elif event.type == pygame.JOYHATMOTION:
                        x_value, y_value = event.value
                        self.handle_dpad_x(x_value)
                        self.handle_dpad_y(y_value)
                        if (x_value == 0 and y_value == 0):
                            self.newValue = False
                        else: 
                            self.newValue = True
                    else:
                        self.newValue = False
                # Update newValue flag if there are changes
                time.sleep(0.01)

if __name__ == "__main__":
    gamepad_handler = GamepadHandler()
    gamepad_handler.start()
