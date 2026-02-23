import serial
import keyboard
import time

# Update 'COM3' to your actual Arduino port
ser = serial.Serial('COM3', 115200, timeout=0.1)
time.sleep(2) # Wait for connection

print("Rover Arm Control Active. Use WASD for Drive, Arrow Keys for Arm.")

def send_cmd(command):
    ser.write(f"{command}\n".encode())

while True:
    try:
        # --- Rover Drive Logic ---
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

        # --- Arm Stepper Logic ---
        if keyboard.is_pressed('up'):
            send_cmd("BASE 100")
        elif keyboard.is_pressed('down'):
            send_cmd("BASE -100")

        # --- Gripper Logic ---
        if keyboard.is_pressed('o'): # Open
            send_cmd("GRIP 180")
        elif keyboard.is_pressed('p'): # Close
            send_cmd("GRIP 0")

        time.sleep(0.05) # Prevent serial buffer overflow
        
    except KeyboardInterrupt:
        send_cmd("STOP")
        ser.close()
        break