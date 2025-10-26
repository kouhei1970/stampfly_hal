#!/usr/bin/env python3
"""
BMI270 SPI通信テスト用モニタリングスクリプト
ESP32-S3からのシリアル出力を監視してBMI270の初期化状況を確認
"""

import serial
import time
import sys
import re

def monitor_bmi270_test(port='/dev/cu.usbmodem1101', baudrate=115200, timeout=40):
    """BMI270テスト結果を監視（起動時ログ含む）"""

    print(f"BMI270 SPI通信テスト監視開始")
    print(f"ポート: {port}, ボーレート: {baudrate}")
    print(f"タイムアウト: {timeout}秒")
    print("デバイスをリセットしてください...")
    print("-" * 60)

    try:
        # シリアル接続
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(1)  # 安定化待ち

        start_time = time.time()
        chip_id_found = False
        initialization_success = False
        bmi270_init_started = False
        app_main_started = False

        while (time.time() - start_time) < timeout:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # タイムスタンプ付きで出力
                current_time = time.time() - start_time
                print(f"[{current_time:6.1f}s] {line}")

                # 重要なイベントをチェック
                if "app_main" in line.lower() or "stampfly hal" in line.lower():
                    if not app_main_started:
                        print(">>> 📱 App_main開始検出")
                        app_main_started = True

                if "BMI270" in line and ("init" in line.lower() or "initializing" in line.lower()):
                    if not bmi270_init_started:
                        print(">>> 🔧 BMI270初期化開始検出")
                        bmi270_init_started = True

                if "chip id" in line.lower() and "0x24" in line:
                    print(">>> ✅ 正しいChip ID (0x24)を検出!")
                    chip_id_found = True

                if "chip id" in line.lower() and ("0x04" in line or "0xf4" in line or "0x00" in line):
                    print(">>> ❌ 不正なChip ID検出")

                if "verification: success" in line.lower() or "initialization completed successfully" in line.lower():
                    print(">>> ✅ BMI270初期化成功!")
                    initialization_success = True
                    break

                if ("initialization" in line.lower() and "failed" in line.lower()) or ("init failed" in line.lower()):
                    print(">>> ❌ BMI270初期化失敗")
                    break

            except UnicodeDecodeError:
                continue

    except serial.SerialException as e:
        print(f"シリアル通信エラー: {e}")
        return False
    except KeyboardInterrupt:
        print("\n監視を中断しました")
        return False
    finally:
        try:
            ser.close()
        except:
            pass

    # 結果サマリー
    print("-" * 60)
    print("テスト結果サマリー:")
    print(f"  App main開始: {'✅ 検出' if app_main_started else '❌ 未検出'}")
    print(f"  BMI270初期化開始: {'✅ 検出' if bmi270_init_started else '❌ 未検出'}")
    print(f"  Chip ID検出: {'✅ 成功' if chip_id_found else '❌ 失敗'}")
    print(f"  初期化成功: {'✅ 成功' if initialization_success else '❌ 失敗'}")

    return chip_id_found and initialization_success

if __name__ == "__main__":
    # ポート指定があれば使用
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/cu.usbmodem1101'

    success = monitor_bmi270_test(port)
    sys.exit(0 if success else 1)