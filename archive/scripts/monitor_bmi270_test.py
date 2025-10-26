#!/usr/bin/env python3
import serial
import time
import sys

def monitor_serial():
    try:
        # シリアルポート設定
        port = '/dev/cu.usbmodem101'
        baud = 115200

        print(f"BMI270 SPI書き込みテスト - シリアルモニター開始")
        print(f"ポート: {port}, ボーレート: {baud}")
        print("=" * 60)

        # シリアルポート接続
        ser = serial.Serial(port, baud, timeout=1)

        # 少し待機
        time.sleep(1)

        # データ受信・表示
        start_time = time.time()
        line_count = 0

        while True:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        current_time = time.time() - start_time
                        print(f"[{current_time:06.2f}] {line}")
                        line_count += 1

                        # BMI270関連のメッセージをハイライト
                        if "BMI270" in line or "SPI" in line:
                            if "❌" in line or "ERROR" in line or "Failed" in line:
                                print(f"    >>> エラー検出: {line}")
                            elif "✅" in line or "passed" in line or "successful" in line:
                                print(f"    >>> 成功: {line}")

                        # 一定時間後に終了（テスト完了判定）
                        if line_count > 200 or current_time > 30:
                            print("\nモニタリング完了（30秒経過または200行受信）")
                            break

                except UnicodeDecodeError:
                    pass

            time.sleep(0.01)

    except serial.SerialException as e:
        print(f"シリアルポートエラー: {e}")
    except KeyboardInterrupt:
        print("\nキーボード割り込みでモニタリング終了")
    finally:
        if 'ser' in locals():
            ser.close()
        print("シリアルポート閉じました")

if __name__ == "__main__":
    monitor_serial()