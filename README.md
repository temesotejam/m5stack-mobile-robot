# M5Stack Mobile Robot

M5Stack Basic を中心に、GNSS・IMU・サーボ制御を統合する移動ロボット用ファームウェアです。

## Hardware

- M5Stack Basic (ESP32, 16 MB flash)
- M5Stack BASE AAA
- M5Stack Module13.2 GoPlus2 (M025-B)
- M5Stack Module GNSS (M135)

## Web flasher

GitHub Pages で公開するブラウザ書き込みページ:

https://temesotejam.github.io/m5stack-mobile-robot/

初回の GitHub Pages デプロイ後、Chrome または Edge でページを開き、USB 接続した M5Stack Basic を選択して書き込みます。Web Serial API を使用するため HTTPS が必要です。

ファームウェアは GitHub Actions が PlatformIO でビルドし、ESP Web Tools 用の merged binary と manifest を生成して Pages に配置します。

## Firmware status

現在の初期ファームウェアは、ハードウェア認識とセンサ値の確認を行う bring-up 用です。

- M5Stack Basic のディスプレイ表示
- GoPlus2 (I2C 0x38) の接続確認
- GNSS Module の NEO-M9N UART データ取得
- BMI270 / BMM150 / BMP280 の初期化と読み取り
- シリアルログ出力

**起動時にサーボやモータを動かす命令は送信しません。** GoPlus2 のサーボ制御 API は用意しますが、実際の動作試験は電源・配線を確認してから個別に行います。

## Local build

PlatformIO Core を使用します。

- Environment: `m5stack-basic`
- Board: `m5stack-core-esp32`
- Framework: Arduino

Build:

```sh
pio run -e m5stack-basic
```

## Hardware notes

- Basic の I2C: GPIO21 (SDA), GPIO22 (SCL)
- GNSS UART: Serial2, RX=GPIO16 / TX=GPIO17, 38400 bps
- GoPlus2: I2C address 0x38. Basic から I2C で指令し、サーボ信号は GoPlus2 側が生成します。
- GNSS Module: NEO-M9N、BMI270、BMM150、BMP280 を搭載。BMM150 は BMI270 の AUX 経由で読み取ります。
- BASE AAA は電源ベースとして扱い、センサ・サーボ・駆動系の電源条件は実機で確認してから確定します。

GNSS Module の UART DIP スイッチは、GPIO16/17 を使う設定にしてください。モジュールのスイッチ設定や搭載センサのアドレス違いに備え、IMU は 0x68 / 0x69 を順に確認します。

詳細は [docs/HARDWARE.md](docs/HARDWARE.md) を参照してください.
