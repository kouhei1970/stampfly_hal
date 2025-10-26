#!/usr/bin/env python3
"""
StampFly Flash & Monitor Utility
ビルド・フラッシュ・モニタリングを一括実行

Usage:
    python3 utilities/flash_monitor.py [options]

Options:
    --port PORT        デバイスポート (デフォルト: /dev/cu.usbmodem101)
    --baudrate BAUD    ボーレート (デフォルト: 115200)
    --build-only       ビルドのみ実行（フラッシュしない）
    --flash-only       フラッシュのみ実行（ビルドしない）
    --no-monitor       モニター起動しない

例:
    python3 utilities/flash_monitor.py                    # ビルド→フラッシュ→モニター
    python3 utilities/flash_monitor.py --build-only       # ビルドのみ
    python3 utilities/flash_monitor.py --flash-only       # フラッシュのみ
    python3 utilities/flash_monitor.py --no-monitor       # モニター無し
"""

import subprocess
import sys
import os
import argparse
import time

# デフォルト設定
DEFAULT_PORT = '/dev/cu.usbmodem101'
DEFAULT_BAUDRATE = 115200
ESP_IDF_EXPORT = '. ~/esp/esp-idf/export.sh'

class StampFlyFlashMonitor:
    def __init__(self, port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def run_command(self, command, description):
        """コマンド実行"""
        print("=" * 70)
        print(f"▶ {description}")
        print("=" * 70)

        try:
            # ESP-IDF環境を含むコマンド実行
            full_command = f"{ESP_IDF_EXPORT} && {command}"
            result = subprocess.run(
                full_command,
                shell=True,
                cwd=self.project_root,
                executable='/bin/zsh',
                check=True,
                capture_output=False
            )
            print(f"✅ {description} - 成功\n")
            return True

        except subprocess.CalledProcessError as e:
            print(f"❌ {description} - 失敗 (終了コード: {e.returncode})\n")
            return False

    def build(self):
        """プロジェクトビルド"""
        return self.run_command('idf.py build', 'ビルド実行')

    def flash(self):
        """デバイスへフラッシュ"""
        return self.run_command(
            f'idf.py -p {self.port} flash',
            f'フラッシュ実行 ({self.port})'
        )

    def monitor(self):
        """シリアルモニター起動"""
        print("=" * 70)
        print("▶ シリアルモニター起動")
        print("=" * 70)

        # monitor.py を使用
        monitor_script = os.path.join(self.project_root, 'utilities', 'monitor.py')

        try:
            subprocess.run(
                ['python3', monitor_script, self.port, str(self.baudrate)],
                cwd=self.project_root
            )
        except KeyboardInterrupt:
            print("\nモニター終了")

    def run(self, build_only=False, flash_only=False, no_monitor=False):
        """実行"""
        print("=" * 70)
        print("StampFly Flash & Monitor Utility")
        print("=" * 70)
        print(f"プロジェクト: {self.project_root}")
        print(f"ポート: {self.port}")
        print(f"ボーレート: {self.baudrate}")
        print("=" * 70)
        print()

        # ビルド
        if not flash_only:
            if not self.build():
                print("❌ ビルド失敗 - 処理中断")
                return False

        # フラッシュ
        if not build_only:
            if not self.flash():
                print("❌ フラッシュ失敗 - 処理中断")
                return False

            # フラッシュ後の待機
            print("デバイス起動待機中...")
            time.sleep(2)

        # モニター
        if not no_monitor and not build_only:
            self.monitor()

        print("\n" + "=" * 70)
        print("✅ 処理完了")
        print("=" * 70)
        return True


def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(
        description='StampFly Flash & Monitor Utility',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
例:
  %(prog)s                      # ビルド→フラッシュ→モニター
  %(prog)s --build-only         # ビルドのみ
  %(prog)s --flash-only         # フラッシュのみ
  %(prog)s --no-monitor         # モニター無し
  %(prog)s --port /dev/cu.usbmodem102
        """
    )

    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f'デバイスポート (デフォルト: {DEFAULT_PORT})')
    parser.add_argument('--baudrate', type=int, default=DEFAULT_BAUDRATE,
                        help=f'ボーレート (デフォルト: {DEFAULT_BAUDRATE})')
    parser.add_argument('--build-only', action='store_true',
                        help='ビルドのみ実行')
    parser.add_argument('--flash-only', action='store_true',
                        help='フラッシュのみ実行')
    parser.add_argument('--no-monitor', action='store_true',
                        help='モニター起動しない')

    args = parser.parse_args()

    # 矛盾チェック
    if args.build_only and args.flash_only:
        print("エラー: --build-only と --flash-only は同時に指定できません")
        sys.exit(1)

    # 実行
    utility = StampFlyFlashMonitor(args.port, args.baudrate)
    success = utility.run(
        build_only=args.build_only,
        flash_only=args.flash_only,
        no_monitor=args.no_monitor
    )

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
