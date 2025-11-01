import serial
import time

arduino = serial.Serial("COM5", 9600, timeout=1)
time.sleep(2)  # Allow Arduino to initialize

AUTHORIZED_ID = "0006464470"

print("RFID + Motion System Running...")

while True:
    try:
        card = input("Tap card: ").strip()
        print("Card Scanned:", card)

        if card == AUTHORIZED_ID:
            print("✅ Authorized card! Checking motion...")
            arduino.write(b'1')  # Tell Arduino authorized card tapped
        else:
            print("❌ Unauthorized card.")
            arduino.write(b'2')

    except KeyboardInterrupt:
        print("\nExiting...")
        arduino.close()
        break
