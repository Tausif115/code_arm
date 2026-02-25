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
    time.sleep(2) # Wait for Arduino reboot
    print(f"Connected to {target_port}")
else:
    print("ERROR: No Arduino found. Check connections.")
    exit()

def send(cmd):
    ser.write(f"{cmd}\n".encode())

print("\n--- ROBOTIC ARM CONTROLS ---")
print("Q/A, W/S, E/D: Linear Actuators 1, 2, 3")
print("LEFT/RIGHT Arrows: Base Rotation")
print("UP/DOWN Arrows:    Wrist Rotation (360 Range)")
print("G: Close Gripper | F: Open Gripper")
print("ESC: Emergency Stop")

try:
    while True:
        # 1. Linear Actuators (Momentary)
        for i, (fwd, rev) in enumerate([('q','a'), ('w','s'), ('e','d')], 1):
            if keyboard.is_pressed(fwd): send(f"DC{i} 255")
            elif keyboard.is_pressed(rev): send(f"DC{i} -255")
            else: send(f"DC{i} 0")

        # 2. Base Stepper (S1)
        if keyboard.is_pressed('right'): send("S1 200")
        elif keyboard.is_pressed('left'): send("S1 -200")

        # 3. Wrist Rotation (360 Range Logic)
        # Using specific commands to prevent serial buffer stalling
        if keyboard.is_pressed('up'):
            send("WRIST_CW")
        elif keyboard.is_pressed('down'):
            send("WRIST_CCW")
        else:
            send("WRIST_STOP")

        # 4. Separate Gripper Commands
        if keyboard.is_pressed('g'):
            print("Action: Closing Gripper")
            send("GRIP 180")
            time.sleep(0.8) # Allow time for mechanical movement
            send("GRIP_OFF")
            
        elif keyboard.is_pressed('f'):
            print("Action: Opening Gripper")
            send("GRIP 0")
            time.sleep(0.8)
            send("GRIP_OFF")

        # 5. Emergency Stop
        if keyboard.is_pressed('esc'):
            send("STOP")
            print("SYSTEM HALTED")
            break

        time.sleep(0.05) # Prevent CPU spiking

except KeyboardInterrupt:
    send("STOP")
    ser.close()
    print("Connection Closed.")