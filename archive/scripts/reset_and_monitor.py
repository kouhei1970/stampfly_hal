#!/usr/bin/env python3
import serial
import time
import subprocess
import sys

def reset_and_monitor():
    print("Resetting ESP32 device...")

    # Reset the device
    try:
        result = subprocess.run(['esptool.py', '--port', '/dev/cu.usbmodem1101', 'run'],
                              capture_output=True, text=True, timeout=10)
        print("Reset command completed")
    except Exception as e:
        print(f"Reset failed: {e}")
        return False

    # Wait a moment for reset to complete
    time.sleep(1)

    print("Starting monitor...")

    try:
        ser = serial.Serial('/dev/cu.usbmodem1101', 115200, timeout=1)
        start_time = time.time()

        bmi270_init_found = False
        app_main_found = False
        chip_id_found = False

        # Monitor for 15 seconds
        while (time.time() - start_time) < 15:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            current_time = time.time() - start_time
            print(f"[{current_time:6.1f}s] {line}")

            # Check for key events
            if "app_main" in line.lower() or "stampfly hal" in line.lower():
                if not app_main_found:
                    print(">>> 📱 App_main detected!")
                    app_main_found = True

            if "bmi270" in line.lower() and ("init" in line.lower() or "DEBUG:" in line):
                if not bmi270_init_found:
                    print(">>> 🔧 BMI270 initialization detected!")
                    bmi270_init_found = True

            if "chip id" in line.lower() and "0x24" in line:
                print(">>> ✅ Correct Chip ID (0x24) detected!")
                chip_id_found = True
                break

        ser.close()

        print("\n" + "="*60)
        print("SUMMARY:")
        print(f"App main detected: {'✅' if app_main_found else '❌'}")
        print(f"BMI270 init detected: {'✅' if bmi270_init_found else '❌'}")
        print(f"Chip ID success: {'✅' if chip_id_found else '❌'}")

        return chip_id_found

    except Exception as e:
        print(f"Monitor error: {e}")
        return False

if __name__ == "__main__":
    success = reset_and_monitor()
    sys.exit(0 if success else 1)