#!/usr/bin/env python3
"""
StampFly Serial Monitor Utility
信頼性の高いシリアルモニタリングツール

Usage:
    python3 utilities/monitor.py [port] [baudrate]

    port: デバイスポート (デフォルト: /dev/cu.usbmodem101)
    baudrate: ボーレート (デフォルト: 115200)

例:
    python3 utilities/monitor.py
    python3 utilities/monitor.py /dev/cu.usbmodem101
    python3 utilities/monitor.py /dev/cu.usbmodem101 115200
"""

import serial
import time
import sys
import signal
from datetime import datetime

# デフォルト設定
DEFAULT_PORT = '/dev/cu.usbmodem101'
DEFAULT_BAUDRATE = 115200

class StampFlyMonitor:
    def __init__(self, port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = True
        self.line_count = 0

    def signal_handler(self, sig, frame):
        """Ctrl+C ハンドラー"""
        print("\n" + "=" * 70)
        print(f"モニター終了 - {self.line_count}行読み取り")
        print("=" * 70)
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        sys.exit(0)

    def connect(self):
        """シリアルポート接続"""
        try:
            print(f"StampFly Serial Monitor")
            print("=" * 70)
            print(f"ポート: {self.port}")
            print(f"ボーレート: {self.baudrate}")
            print(f"開始時刻: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            print("終了: Ctrl+C")
            print("=" * 70)

            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.5,
                write_timeout=1
            )

            # デバイスリセット（DTRトグル）
            print("デバイスをリセット中...")
            self.ser.setDTR(False)
            time.sleep(0.1)
            self.ser.setDTR(True)
            time.sleep(0.5)

            print("接続完了 - 出力開始")
            print("=" * 70)

            return True

        except serial.SerialException as e:
            print(f"エラー: シリアルポート接続失敗 - {e}")
            print(f"\n利用可能なポートを確認してください:")
            print(f"  ls /dev/cu.*")
            return False
        except Exception as e:
            print(f"予期しないエラー: {e}")
            return False

    def monitor(self):
        """シリアル出力監視"""
        if not self.ser or not self.ser.is_open:
            print("エラー: シリアルポートが開いていません")
            return

        buffer = ""

        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    # データ読み取り
                    data = self.ser.read(self.ser.in_waiting)
                    text = data.decode('utf-8', errors='replace')
                    buffer += text

                    # 行単位で処理
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.rstrip('\r')
                        if line:
                            print(line)
                            self.line_count += 1

                else:
                    # データ待ち
                    time.sleep(0.01)

            except serial.SerialException as e:
                print(f"\nシリアル通信エラー: {e}")
                break
            except UnicodeDecodeError as e:
                print(f"\nデコードエラー: {e}")
                continue
            except KeyboardInterrupt:
                break

        # 残りバッファ出力
        if buffer:
            print(buffer.rstrip())

    def run(self):
        """実行"""
        # シグナルハンドラー登録
        signal.signal(signal.SIGINT, self.signal_handler)

        # 接続
        if not self.connect():
            sys.exit(1)

        # 監視開始
        try:
            self.monitor()
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            print(f"\n接続終了 - 合計 {self.line_count}行")


def main():
    """メイン関数"""
    port = DEFAULT_PORT
    baudrate = DEFAULT_BAUDRATE

    # コマンドライン引数処理
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            baudrate = int(sys.argv[2])
        except ValueError:
            print(f"エラー: 無効なボーレート '{sys.argv[2]}'")
            sys.exit(1)

    # モニター実行
    monitor = StampFlyMonitor(port, baudrate)
    monitor.run()


if __name__ == '__main__':
    main()
