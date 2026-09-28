"""
WASD remote control for the stepper bot (ESP32-C3, UDP).

Hold W/A/S/D to drive; release to stop. Sends a small UDP packet
every 50ms with the currently held direction (or 'X' if nothing
is held), matching the ESP32's 300ms watchdog timeout.

Setup:
    pip install keyboard
    (Windows: run this script as Administrator, or the 'keyboard'
     library can't see global key presses.
     Linux: run with sudo for the same reason.)

Usage:
    1. Set ESP_IP below to the IP address printed in the ESP32's
       Serial Monitor on boot.
    2. Run: python wasd_control.py
    3. Hold W/A/S/D to drive. Ctrl+C to quit.
"""

import socket
import time
import keyboard

ESP_IP = "192.168.50.175"   # <-- set this to your ESP32-C3's IP address
ESP_PORT = 4210
SEND_INTERVAL_S = 0.05     # 50ms, well under the ESP32's 300ms watchdog

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send(cmd: str):
    sock.sendto(cmd.encode(), (ESP_IP, ESP_PORT))


def main():
    print(f"Sending WASD commands to {ESP_IP}:{ESP_PORT}")
    print("Hold W/A/S/D to drive, release to stop. Ctrl+C to quit.")

    try:
        while True:
            if keyboard.is_pressed('w'):
                send('D')
            elif keyboard.is_pressed('s'):
                send('A')
            elif keyboard.is_pressed('a'):
                send('S')
            elif keyboard.is_pressed('d'):
                send('W')
            else:
                send('X')
            time.sleep(SEND_INTERVAL_S)
    except KeyboardInterrupt:
        send('X')
        print("\nStopped.")


if __name__ == "__main__":
    main()
