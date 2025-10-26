#!/usr/bin/env python3
import serial
import time

def quick_monitor():
    try:
        ser = serial.Serial('/dev/cu.usbmodem1101', 115200, timeout=0.5)
        print("=== デバイスの現在の出力 ===")

        for i in range(20):  # 10秒間監視
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[{i*0.5:3.1f}s] {line}")

        ser.close()
    except Exception as e:
        print(f"エラー: {e}")

if __name__ == "__main__":
    quick_monitor()