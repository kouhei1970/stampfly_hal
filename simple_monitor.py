#!/usr/bin/env python3
import serial
import time

# シリアルポート設定
port = '/dev/cu.usbmodem101'
baud = 115200

print(f"BMI270テスト - シリアルモニター")
print(f"ポート: {port}")
print("=" * 50)

try:
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(0.5)  # 接続安定化

    # 20秒間出力を読み取り
    start_time = time.time()
    lines_printed = 0

    while (time.time() - start_time) < 20 and lines_printed < 100:
        if ser.in_waiting > 0:
            try:
                data = ser.read(ser.in_waiting)
                text = data.decode('utf-8', errors='ignore')
                if text.strip():
                    for line in text.split('\n'):
                        if line.strip():
                            print(line.strip())
                            lines_printed += 1
            except:
                pass
        time.sleep(0.1)

    ser.close()
    print("\nモニター終了")

except Exception as e:
    print(f"エラー: {e}")