# susumu_blinkstick_ros2
[BlinkStick](https://www.blinkstick.com/)というUSBで制御できるLEDを使って、LEDアニメーションを実現するROS2パッケージです。
<br>
<br>
![サンプル画像](docs/blinkstick_strip.gif)
<br>

## 動作確認環境
CPU: Intel  
OS: Ubuntu 22.04  
ROS: Humble  
<br>
<br>

## セットアップ
あらかじめ、[BlinkStickを使うための準備](#BlinkStickを使うための準備)を行い、
ターミナルからblinkstickコマンドでLED制御できる状態にしておく必要があります。
下記はワークスペースがros2_wsであることを前提としています。
```
cd ~/ros2_ws/src
git clone https://github.com/sato-susumu/susumu_ros2_interfaces.git
git clone https://github.com/sato-susumu/susumu_blinkstick_ros2.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```
<br>
<br>

## ノードの起動
```bash
ros2 launch susumu_blinkstick_ros2 blinkstick_node.launch.py
```
または
```bash
ros2 run susumu_blinkstick_ros2 blinkstick_node
```
<br>
<br>

## LEDアニメーション指示
### SOLID
```bash
ros2 topic pub -1 /led susumu_ros2_interfaces/LED "{pattern: 'SOLID'}"
```
*   指定した単色でLEDを点灯させます。`color1`で色を指定。指定していない内容はデフォルト値になります。

### BLINK
```bash
ros2 topic pub -1 /led susumu_ros2_interfaces/LED "{pattern: 'BLINK', duration: 10.0, color1: 'red'}"
```
*   指定した色でLEDを点滅させます。`color1`で色、`duration`で持続時間を指定。

### TWO_COLOR_FADE
```bash
ros2 topic pub -1 /led susumu_ros2_interfaces/LED "{pattern: 'TWO_COLOR_FADE', duration: 5.0, color1: 'yellow', color2: 'purple'}"
```
*   `color1`と`color2`で指定した2色間でLEDの色を滑らかに変化させます。`duration`で持続時間を指定。

### LEFT_TO_RIGHT
```bash
ros2 topic pub -1 /led susumu_ros2_interfaces/LED "{pattern: 'LEFT_TO_RIGHT', priority: 2, decay_rate: 0.8, speed: 2.5}"
```
*   指定した色が左から右へ移動するように点灯します。`color1`で色、`speed`でアニメーション速度、`decay_rate`で残像の減衰率を指定、`priority`で優先度を指定。

### RIGHT_TO_LEFT
```bash
ros2 topic pub -1 /led susumu_ros2_interfaces/LED "{pattern: 'RIGHT_TO_LEFT'}"
```
*   指定した色が右から左へ移動するように点灯します。

### WAVE
```bash
ros2 topic pub -1 /led susumu_ros2_interfaces/LED "{pattern: 'WAVE', duration: 12.0, color1: 'green', speed: 8.0}"
```
*   指定した色が波のように移動するように点灯します。

### INWARD
```bash
ros2 topic pub -1 /led susumu_ros2_interfaces/LED "{pattern: 'INWARD', duration: 10.0, color1: 'orange', speed: 8.0}"
```
*   指定した色が外側から内側へ移動するように点灯します。

### OUTWARD
```bash
ros2 topic pub -1 /led susumu_ros2_interfaces/LED "{pattern: 'OUTWARD', duration: 10.0, color1: 'pink', speed: 8.0}"
```
*   指定した色が内側から外側へ移動するように点灯します。
<br>
<br>

## blinkstick_nodeの詳細
### 起動時のパラメータ指定
`blinkstick_node`は、起動時に以下のパラメータを受け付けます。

* **`led_count` (int, デフォルト: 8):** 接続されているBlinkStickデバイスのLEDの数を指定します。
* **`brightness` (double, デフォルト: 0.5):** LEDの明るさを指定します。

### 動的なパラメータ変更
ノード実行中に、`ros2 param set`コマンドを使用して、ノードのパラメータを動的に変更できます。たとえば、`brightness`を"1.0"に変更するには、次のようにします。

```bash
ros2 param set /blinkstick_node brightness 1.0
```

現在のパラメータ一覧や値を確認するには、以下を使用します。

```bash
ros2 param list
ros2 param get /blinkstick_node brightness
```

### サブスクライブするトピック

* **`led` (`susumu_ros2_interfaces/LED`):** LEDのアニメーション指示をサブスクライブします。
<br>
<br>


## /ledトピックの詳細
このトピックは、`susumu_ros2_interfaces/msg/LED` 型です。

| フィールド    | 型       | デフォルト値 | 説明                                                                                                                                            |
|--------------|----------|-----------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| `pattern`    | `string` | `"SOLID"`   | LEDアニメーションのパターンを指定します。指定可能な値: `"SOLID"`, `"BLINK"`, `"TWO_COLOR_FADE"`, `"LEFT_TO_RIGHT"`, `"RIGHT_TO_LEFT"`, `"WAVE"`, `"INWARD"`, `"OUTWARD"` |
| `priority`   | `int32`  | `1`         | アニメーションの優先度を指定します。値が大きいほど優先度が高くなります。                                                                                    |
| `duration`   | `float32`| `3.0`       | アニメーションの持続時間（秒）を指定します。                                                                                                            |
| `color1`     | `string` | `"white"`   | アニメーションに使用する1つ目の色を指定します。基本的な色名（`"red"`, `"green"`など）または16進数RGBコード（`"#FF0000"`, `"FF0000"`など）で指定します。                                      |
| `color2`     | `string` | `"black"`   | `TWO_COLOR_FADE` アニメーションで使用する2つ目の色を指定します。色の指定方法は `color1` と同様です。                                                                    |
| `decay_rate` | `float32`| `0.8`       | 減衰率（0.0~1.0）を指定します。アニメーションによっては適用されない場合があります。                                                                                        |
| `speed`      | `float32`| `8.0`       | アニメーションの速度や点滅頻度などを指定します。アニメーションのパターンによって意味が異なります。       
<br>
<br>

## BlinkStickを使うための準備
### ハードウェアの認識確認
```
lsusb
```
「Bus 001 Device 013: ID 20a0:41e5 Clay Logic BlinkStick」が表示されればOK。

### Python用APIのセットアップ
```bash
git clone https://github.com/arvydas/blinkstick-python.git
cd blinkstick-python
pip install .
```
ターミナルを終了して別ターミナルを立ち上げる。


### BlinkStickデバイスを非rootユーザーで操作するためのudev設定
```bash
echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"20a0\", ATTR{idProduct}==\"41e5\", MODE:=\"0666\"" | sudo tee /etc/udev/rules.d/85-blinkstick.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 動作確認
```bash
blinkstick random
```
適当な色に変化すればOK

```bash
blinkstick --blink red --repeats 3
```
赤色でON/OFFを3回繰り返す

```bash
blinkstick --pulse 0000FF --repeats 3
```
青色で徐々に明るく暗くを3回繰り返す

## トラブルシューティング
### [Errno 5] Input/Output Error
```
Exception in thread Thread-1 (_worker_loop):
Traceback (most recent call last):
  File "/home/taro/.local/lib/python3.10/site-packages/blinkstick/blinkstick.py", line 252, in _usb_ctrl_transfer
    return self.device.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, data_or_wLength)
  File "/home/taro/.local/lib/python3.10/site-packages/usb/core.py", line 1082, in ctrl_transfer
    ret = self._ctx.backend.ctrl_transfer(
  File "/home/taro/.local/lib/python3.10/site-packages/usb/backend/libusb1.py", line 893, in ctrl_transfer
    ret = _check(self.lib.libusb_control_transfer(
  File "/home/taro/.local/lib/python3.10/site-packages/usb/backend/libusb1.py", line 604, in _check
    raise USBError(_strerror(ret), ret, _libusb_errno[ret])
usb.core.USBError: [Errno 5] Input/Output Error
```
根本的な原因や解決方法はよくわかっていません。  
PCのUSBポートとBlinkStickを直接接続したときに発生しました。   
USBハブを経由して接続することで発生しなくなりました。。。  

### Access denied
```
blinkstick random
```
実行時、次のメッセージが表示された場合
```
Traceback (most recent call last):
省略
    raise USBError(_strerror(ret), ret, _libusb_errno[ret])
usb.core.USBError: [Errno 13] Access denied (insufficient permissions)
```
[BlinkStickデバイスを非rootユーザーで操作するためのudev設定](#BlinkStickデバイスを非rootユーザーで操作するためのudev設定)を参照してください。
<br>
<br>

## リンク
[blinkstickのページ](https://www.blinkstick.com/)  
[blinkstick-python(Python用SDK)](https://github.com/arvydas/blinkstick-python)  
[公式フォーラム](https://forums.blinkstick.com)  
[回路図](https://www.blinkstick.com/help/schematics)  
