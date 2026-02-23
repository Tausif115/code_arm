import serial
import serial.tools.list_ports
import keyboard
import time

def find_arduino():
    print("Scanning for Arduino Mega...")
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if any(x in p.description for x in ["CH340", "Arduino", "USB Serial"]):
            return p.device
    return None

target_port = find_arduino()

if target_port:
    ser = serial.Serial(target_port, 115200, timeout=0.1)
    time.sleep(2) # Allow Mega to initialize
    print(f"Connected to {target_port}")
else:
    print("ERROR: No Arduino found. Check USB and DIP switches 3 & 4.")
    exit()

# State variables for the Gripper toggle
gripper_open = False
last_g_state = False

def send(cmd):
    ser.write(f"{cmd}\n".encode())

print("\n--- ROBOTIC ARM CONTROLS ---")
print("Q/A: Actuator 1 | W/S: Actuator 2 | E/D: Actuator 3")
print("LEFT/RIGHT Arrows: Base NEMA 23 (180° Range)")
print("UP/DOWN Arrows:    Tilt NEMA 23 (120° Range)")
print("G Key: Toggle Gripper Open/Close")
print("ESC: Emergency Stop All Motors")

try:
    while True:
        # 1. Linear Actuators (Momentary)
        if keyboard.is_pressed('q'): send("DC1 255")
        elif keyboard.is_pressed('a'): send("DC1 -255")
        else: send("DC1 0")

        if keyboard.is_pressed('w'): send("DC2 255")
        elif keyboard.is_pressed('s'): send("DC2 -255")
        else: send("DC2 0")

        if keyboard.is_pressed('e'): send("DC3 255")
        elif keyboard.is_pressed('d'): send("DC3 -255")
        else: send("DC3 0")

        # 2. NEMA 23 Steppers (Incremental) 
        # Base Rotation
        if keyboard.is_pressed('right'): send("S1 200")
        elif keyboard.is_pressed('left'): send("S1 -200")
        
        # Gripper Tilt (New NEMA 23 on pins 40,41,42)
        if keyboard.is_pressed('up'): send("S2 150")
        elif keyboard.is_pressed('down'): send("S2 -150")

        # 3. Gripper Toggle Logic 
        g_now = keyboard.is_pressed('g')
        if g_now and not last_g_state:
            if not gripper_open:
                send("GRIP 70")
                gripper_open = True
            else:
                send("GRIP 0")
                gripper_open = False
            
            time.sleep(0.8)   # Allow time for metal gears to move
            send("GRIP_OFF")  # Cut power to stop jittering
        last_g_state = g_now

        # 4. Emergency Stop
        if keyboard.is_pressed('esc'):
            send("STOP")
            print("EMERGENCY STOP TRIGGERED")
            break

        time.sleep(0.05) # Prevent serial buffer flooding

except KeyboardInterrupt:
    send("STOP")
    ser.close()
    print("Connection Closed.")