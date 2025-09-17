# BMI270設定ファイルアップロード問題分析

## 問題の現象

```
I (3449) BMI270: Status change at 0ms: 0xFF -> 0x02 (message=2)
E (3959) BMI270: Initialization timeout after 510ms. Final status: 0x02 (message=2)
E (3959) BMI270: Status interpretations: 0x01=success, 0x02=in_progress, others=error
E (3959) BMI270: Initialization timeout: ESP_ERR_TIMEOUT
```

## データシート仕様確認

### INTERNAL_STATUSレジスタ解釈
- **0x01 (message=1)**: 初期化完了（成功）
- **0x02 (message=2)**: **初期化エラー** - データシートによると「init error」
- **現在の解釈が間違い**: 0x02は「進行中」ではなく「エラー」

### 正しい初期化シーケンス（データシート準拠）

1. **PWR_CONF設定**: Advanced Power Save無効化
2. **INIT_CTRL = 0x00**: 初期化制御リセット（必須）
3. **設定ファイルアップロード**: INIT_DATAレジスタへ
4. **INIT_CTRL = 0x01**: 初期化開始
5. **INTERNAL_STATUS監視**: 0x01まで待機

## 現在のコードの問題点

### 1. INIT_ADDRレジスタ更新の問題

```cpp
// 現在のコード（問題あり）
addr_offset += bytes_to_write / 2;  // BMI270 requires word addressing
```

**問題**: データシートによると、INIT_ADDR_0/1は**バイト単位**で更新する必要がある。

**正しい実装**:
```cpp
addr_offset += bytes_to_write;  // バイト単位で更新
```

### 2. チャンク間のアドレス更新タイミング

現在のコードはチャンク書き込み**後**にアドレスを更新しているが、データシートでは**次のチャンク書き込み前**に更新する必要がある。

### 3. 設定ファイルの整合性問題

BMI270には複数の設定ファイルバリエーションが存在：
- 基本設定ファイル
- 特定機能対応設定ファイル
- OEMカスタマイズ設定ファイル

使用している`bmi270_config_file`が StampFly のハードウェア構成と適合していない可能性。

## Boschコミュニティの報告事例

### STM32での類似問題
- **症状**: INTERNAL_STATUS = 0x02でタイムアウト
- **原因**: 設定ファイルアップロード時のバーストライト問題
- **解決**: I2C/SPIバーストライト制限の適切な処理

### アドレス更新問題
- **問題**: INIT_ADDR_0/1の更新方法が不正確
- **解決**: バイト単位での正確なアドレス計算

## 推奨修正内容

### 1. INIT_ADDRレジスタ更新修正
```cpp
// 修正前
addr_offset += bytes_to_write / 2;  // 間違い

// 修正後
addr_offset += bytes_to_write;      // バイト単位
```

### 2. ステータス解釈修正
```cpp
// 現在の誤った解釈
// 0x02 = in_progress

// 正しい解釈
// 0x02 = initialization error
```

### 3. エラー時の詳細診断追加
- 設定ファイルサイズ検証
- チェックサム検証（可能な場合）
- SPI通信エラー監視強化

### 4. 設定ファイル検証
使用している`bmi270_config_file`がBosch公式の最新版かつStampFly対応版であることを確認。

## 次の調査ステップ

1. **INIT_ADDRレジスタ更新方法の修正**
2. **設定ファイルの正当性確認**
3. **SPI バーストライト実装の検証**
4. **BMI270公式サンプルコードとの比較**

## 結論

現在の問題は設定ファイルアップロード処理の不備によるもので、タイムアウト時間の延長では解決しない。特にINIT_ADDRレジスタの更新方法とステータス解釈の修正が必要。