
from inputs import get_gamepad
from threading import Thread

class JoystickInput:
    def __init__(self):
        Thread(target=self._listen_for_events, daemon=True).start()
        self.cmd = False
        
    def _listen_for_events(self):
        while True:
            events = self._get_joystick_events()
            for event in events:
                if event.code == "ABS_HAT0X" and event.state != 0:
                    self.cmd = True
                    
                # print(event.ev_type, event.code, event.state)

    def _get_joystick_events(self):
        return get_gamepad()
    
    def get_cmd(self) -> bool:
        return self.cmd

def main():
    joystick = JoystickInput()
    while True:
        events = joystick._get_joystick_events()
        for event in events:
            print(event.ev_type, event.code, event.state)


if __name__ == "__main__":
    main()
