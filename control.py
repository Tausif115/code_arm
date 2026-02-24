import serial
import serial.tools.list_ports
import time
from pynput import keyboard

# --- Serial Setup ---
def find_arduino():
    print("Scanning for Arduino Mega...")
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if any(x in p.description for x in ["CH340", "Arduino", "USB Serial"]) or "usbserial" in p.device:
            return p.device
    return None

target_port = find_arduino()
if not target_port:
    print("ERROR: No Arduino found.")
    exit()

try:
    ser = serial.Serial(target_port, 115200, timeout=0.1)
    time.sleep(2) 
    print(f"Connected to {target_port}")
except Exception as e:
    print(f"Failed to open port: {e}")
    exit()

def send(cmd):
    ser.write(f"{cmd}\n".encode())

# --- Control Logic ---
gripper_open = False
active_keys = set()

def on_press(key):
    global gripper_open
    try:
        # Handle letter keys (q, a, w, s, etc.)
        k = key.char.lower()
    except AttributeError:
        # Handle special keys (arrows, esc)
        k = key.name

    if k not in active_keys:
        active_keys.add(k)
        
        # --- Linear Actuators ---
        if k == 'q': send("DC1 255")
        elif k == 'a': send("DC1 -255")
        elif k == 'w': send("DC2 255")
        elif k == 's': send("DC2 -255")
        elif k == 'e': send("DC3 255")
        elif k == 'd': send("DC3 -255")

        # --- Steppers ---
        elif k == 'right': send("S1 200")
        elif k == 'left': send("S1 -200")
        elif k == 'up': send("S2 150")
        elif k == 'down': send("S2 -150")

        # --- Gripper Toggle ---
        elif k == 'g':
            if not gripper_open:
                send("GRIP 70")
                gripper_open = True
            else:
                send("GRIP 0")
                gripper_open = False
            time.sleep(0.8)
            send("GRIP_OFF")

def on_release(key):
    try:
        k = key.char.lower()
    except AttributeError:
        k = key.name

    if k in active_keys:
        active_keys.remove(k)
        
        # Stop DC motors immediately when key is released
        if k in ['q', 'a']: send("DC1 0")
        if k in ['w', 's']: send("DC2 0")
        if k in ['e', 'd']: send("DC3 0")

    if k == 'esc':
        send("STOP")
        print("Shutting down...")
        return False # Stops the listener

print("\n--- ROBOTIC ARM CONTROLS (MAC VERSION) ---")
print("Q/A, W/S, E/D: Actuators | Arrows: Base & Tilt | G: Toggle Gripper | ESC: Quit")

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()