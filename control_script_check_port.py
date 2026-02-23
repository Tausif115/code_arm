import serial
import serial.tools.list_ports
import keyboard
import time

def find_arduino():
    print("Scanning for Arduino Mega...")
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # Looking for common identifiers for Mega + WiFi (CH340) or standard Mega
        if "CH340" in p.description or "Arduino" in p.description or "USB Serial" in p.description:
            return p.device
    return None

# Connection Initialization
target_port = find_arduino()

if target_port:
    try:
        # Using 115200 to match the Arduino Mega's high-speed buffer
        ser = serial.Serial(target_port, 115200, timeout=0.1)
        time.sleep(2)  # Wait for Mega to reset
        print(f"Connected to {target_port} successfully!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        exit()
else:
    print("NO COM PORT FOUND! Check USB and DIP Switches (3 & 4 ON).")
    exit()

def send_cmd(command):
    """Formats and sends the string command to Arduino."""
    ser.write(f"{command}\n".encode())

print("\nReady for Control:")
print("WASD: Drive | UP/DOWN: Base | O/P: Gripper | CTRL+C: Exit")

try:
    while True:
        # Rover Drive Logic 
        # Wheels stop immediately when no WASD key is held
        if keyboard.is_pressed('w'):
            send_cmd("DRIVE 200 200")
        elif keyboard.is_pressed('s'):
            send_cmd("DRIVE -200 -200")
        elif keyboard.is_pressed('a'):
            send_cmd("DRIVE -150 150")
        elif keyboard.is_pressed('d'):
            send_cmd("DRIVE 150 -150")
        else:
            send_cmd("DRIVE 0 0") # Stop if no key is pressed

        #Arm Stepper Logic
        # Independent of drive logic
        if keyboard.is_pressed('up'):
            send_cmd("BASE 100")
        elif keyboard.is_pressed('down'):
            send_cmd("BASE -100")

        # Gripper Logic 
        # Independent of drive and arm logic
        if keyboard.is_pressed('o'): # Open
            send_cmd("GRIP 180")
        elif keyboard.is_pressed('p'): # Close
            send_cmd("GRIP 0")

        time.sleep(0.05) # Prevent serial buffer overflow
        
except KeyboardInterrupt:
    print("\nShutting down...")
    send_cmd("STOP") # Custom Arduino function to halt all movement
    ser.close()
    print("Serial connection closed.")