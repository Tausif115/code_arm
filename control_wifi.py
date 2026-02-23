import socket
import keyboard
import time

# Update this with the IP address printed in your Serial Monitor
ROVER_IP = "192.168.1.XX" 
PORT = 80 

def send_cmd(command):
    try:
        # We open and close the socket quickly for each command
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.05) # Fast timeout for responsiveness
            s.connect((ROVER_IP, PORT))
            s.sendall(f"{command}\n".encode())
    except Exception as e:
        # Use pass to keep the loop running even if one packet fails
        pass 

print(f"Connecting to Rover at {ROVER_IP}...")
print("Controls: WASD (Drive), Arrow Keys (Base Stepper), O/P (Gripper)")

while True:
    try:
        #Rover Drive Logic 
        if keyboard.is_pressed('w'):
            send_cmd("DRIVE 200 200")
        elif keyboard.is_pressed('s'):
            send_cmd("DRIVE -200 -200")
        elif keyboard.is_pressed('a'):
            send_cmd("DRIVE -150 150")
        elif keyboard.is_pressed('d'):
            send_cmd("DRIVE 150 -150")
        else:
            send_cmd("DRIVE 0 0") # Critical: Stops motors when keys released

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

        time.sleep(0.05) # Prevent flooding the WiFi chip
        
    except KeyboardInterrupt:
        send_cmd("STOP")
        print("\nStopping and exiting...")
        break