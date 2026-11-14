# CH32H417-Latency-Tester

[InputLagScope](https://github.com/608/InputLagScope)で使用するためのCH32H417用ファームウェアです。

## 必要なもの

- CH32H417の開発ボード : https://ja.aliexpress.com/item/1005012984455145.html
- (推奨)ジャンパワイヤー : Amazonなどで購入可能

## 接続

計測対象のコントローラーを開発ボード上のSS&HSポートに接続し、FS&PDポートをPCへ接続します。

PA0をボタンの信号ラインと接続し、PA1をスティックの信号ラインと接続します。

FWの初回書き込み時のみLINKEポートをPCに接続する必要があります。

## インストール

### インストールに必要なもの

- WCH-LinkUtility V2.80

### バイナリのダウンロード

[Releases](https://github.com/608/CH32H417-Latency-Tester/releases)から`CH32H417-Latency-Tester-v1.0.0.bin`をダウンロードします。

### インストール

1. LINKEポートをPCに接続します。
2. WCH-LinkUtilityを起動し、`Core`で`RISC-V`、`Series`で`CH32H41X`を選択します。
3. `Target` > `Clear All Code Flash-By Power off`を実行します。
4. `File` > `Open Firmware`から`CH32H417-Latency-Tester-v1.0.0.bin`を開き、`addr`に`0x08000000`を設定して、`Program`と`Verify`にチェックを入れます。
5. `Target` > `Program`を実行し、ログに`Succeed`と表示されることを確認します。

## ビルド

```sh
git config --global core.longpaths true
bash tools/build_all.sh
```
