
import struct
from threading import Thread

class JoystickInput:
    def __init__(self):
        Thread(target=self._listen_for_events, daemon=True).start()
        self.cmd = False
        
    def _listen_for_events(self):
        with open('/dev/input/js0', 'rb') as js_device:
            while True:
                event = js_device.read(8)
                if event:
                    time_val, value, type_, number = struct.unpack('IhBB', event)
                    # print(f"时间: {time_val}, 值: {value}, 类型: {type_}, 编号: {number}")
                    if type_ == 2 and value > 0 and number == 6:
                        print("send cmd")
                        self.cmd = True
    
    def get_cmd(self) -> bool:
        return self.cmd

def main():
    joystick = JoystickInput()
    with open('/dev/input/js0', 'rb') as js_device:
        while True:
            event = js_device.read(8)
            if event:
                time_val, value, type_, number = struct.unpack('IhBB', event)
                print(f"时间: {time_val}, 值: {value}, 类型: {type_}, 编号: {number}")

if __name__ == "__main__":
    main()
