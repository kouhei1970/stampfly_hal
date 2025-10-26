# StampFly開発ユーティリティ

このフォルダにはStampFly開発で使用する便利なツールが含まれています。

## 📁 ファイル一覧

### monitor.py - シリアルモニター
信頼性の高いシリアルモニタリングツール。`idf.py monitor`が使えない環境でも動作します。

**使用方法:**
```bash
# 基本使用（デフォルトポート: /dev/cu.usbmodem101）
python3 utilities/monitor.py

# カスタムポート指定
python3 utilities/monitor.py /dev/cu.usbmodem102

# ボーレート指定
python3 utilities/monitor.py /dev/cu.usbmodem101 115200

# 実行可能にした場合
./utilities/monitor.py
```

**機能:**
- 自動デバイスリセット（DTRトグル）
- UTF-8デコード（エラー処理付き）
- Ctrl+Cで安全終了
- 読み取り行数カウント
- タイムスタンプ表示

---

### flash_monitor.py - ビルド・フラッシュ・モニター統合ツール
ビルドからモニタリングまでを一括実行します。

**使用方法:**
```bash
# フルフロー実行（ビルド→フラッシュ→モニター）
python3 utilities/flash_monitor.py

# ビルドのみ
python3 utilities/flash_monitor.py --build-only

# フラッシュのみ（ビルド済みの場合）
python3 utilities/flash_monitor.py --flash-only

# モニター無し（フラッシュ後に終了）
python3 utilities/flash_monitor.py --no-monitor

# カスタムポート指定
python3 utilities/flash_monitor.py --port /dev/cu.usbmodem102

# 実行可能にした場合
./utilities/flash_monitor.py
```

**オプション:**
- `--port PORT` - デバイスポート指定（デフォルト: /dev/cu.usbmodem101）
- `--baudrate BAUD` - ボーレート指定（デフォルト: 115200）
- `--build-only` - ビルドのみ実行
- `--flash-only` - フラッシュのみ実行
- `--no-monitor` - モニター起動しない

---

## 🔧 セットアップ

### 必要な環境
- Python 3.x
- pyserial ライブラリ

### pyserialインストール
```bash
pip3 install pyserial
```

### スクリプトを実行可能にする（オプション）
```bash
chmod +x utilities/*.py
```

---

## 💡 使用例

### 開発中の典型的なワークフロー

1. **コード編集後、すぐに実機確認:**
```bash
python3 utilities/flash_monitor.py
```

2. **シリアル出力のみ確認（再フラッシュ不要）:**
```bash
python3 utilities/monitor.py
```

3. **ビルドエラー確認（フラッシュ前）:**
```bash
python3 utilities/flash_monitor.py --build-only
```

4. **複数デバイスで開発:**
```bash
# デバイス1
python3 utilities/flash_monitor.py --port /dev/cu.usbmodem101

# デバイス2
python3 utilities/flash_monitor.py --port /dev/cu.usbmodem102
```

---

## ⚠️ idf.py monitor が使えない理由

ESP-IDFの`idf.py monitor`はTTY（対話型ターミナル）が必要ですが、以下の環境では動作しません:
- Claude Code等の非対話型環境
- SSH経由のリモート実行
- CI/CD パイプライン
- バックグラウンド実行

このユーティリティは上記環境でも動作する設計になっています。

---

## 📝 トラブルシューティング

### デバイスが見つからない
```bash
# 利用可能なポートを確認
ls /dev/cu.*

# USB接続確認
# - USBケーブルが接続されているか
# - StampFlyの電源が入っているか（LED点灯確認）
```

### Permission denied エラー
```bash
# スクリプトに実行権限を付与
chmod +x utilities/monitor.py
chmod +x utilities/flash_monitor.py
```

### pyserial が見つからない
```bash
# pyserial インストール
pip3 install pyserial

# 確認
python3 -c "import serial; print(serial.__version__)"
```

---

## 🎯 推奨ワークフロー

**CLAUDE.mdに記載の通り、以下のコマンドを使用してください:**

```bash
# 開発フロー（推奨）
python3 utilities/flash_monitor.py

# モニターのみ
python3 utilities/monitor.py
```

従来の`idf.py flash monitor`の代わりに、上記ユーティリティの使用を標準としています。
