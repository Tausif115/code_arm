import serial
import serial.tools.list_ports
import keyboard
import time

def find_arduino():
    print("Searching for Arduino...")
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if any(x in p.description for x in ["CH340", "Arduino", "USB Serial"]):
            print(f"Connected to {p.device}")
            return p.device
    return None

port = find_arduino()
if not port:
    print("No Arduino found. Check DIP switches 3&4.")
    exit()

ser = serial.Serial(port, 115200, timeout=0.1)
time.sleep(2)

gripper_open = False
last_g_state = False

def send(cmd):
    ser.write(f"{cmd}\n".encode())

print("\n--- CONTROL KEYS ---")
print("Q/A: Actuator 1 | W/S: Actuator 2 | E/D: Actuator 3")
print("UP/DOWN: NEMA23 | LEFT/RIGHT: NEMA17-1 | Z/X: NEMA17-2")
print("G: Toggle Gripper (0/70) | ESC: Emergency Stop")

try:
    while True:
        # 1. Linear Actuators (Hold key to move)
        if keyboard.is_pressed('q'): send("DC1 255")
        elif keyboard.is_pressed('a'): send("DC1 -255")
        else: send("DC1 0")

        if keyboard.is_pressed('w'): send("DC2 255")
        elif keyboard.is_pressed('s'): send("DC2 -255")
        else: send("DC2 0")

        if keyboard.is_pressed('e'): send("DC3 255")
        elif keyboard.is_pressed('d'): send("DC3 -255")
        else: send("DC3 0")

        # 2. Steppers (Relative nudges)
        if keyboard.is_pressed('up'): send("S1 100")
        elif keyboard.is_pressed('down'): send("S1 -100")
        
        if keyboard.is_pressed('right'): send("S2 100")
        elif keyboard.is_pressed('left'): send("S2 -100")

        # 3. Gripper Toggle (Press G once)
        g_now = keyboard.is_pressed('g')
        if g_now and not last_g_state:
            if not gripper_open:
                send("GRIP 70")
                gripper_open = True
            else:
                send("GRIP 0")
                gripper_open = False
            time.sleep(0.8) # Wait for move
            send("GRIP_OFF") # Silence the jitter
        last_g_state = g_now

        if keyboard.is_pressed('esc'):
            send("STOP")
            break

        time.sleep(0.05)
except KeyboardInterrupt:
    send("STOP")
    ser.close()