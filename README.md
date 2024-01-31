# jishupuro

## APIキーの設定
```
pip3 install python-dotenv
```
として、./envというファイルに
```
api_key = “<API_KEY>”
```
を書き込む
参考：https://qiita.com/wooooo/items/7b57eaf32c22195df843


## 実行方法（radxaで操作する場合）
- ipはM5ATOM S3に表示される。ディズプレイをつないで`ip a`としてもよい。
- sshして（　`ssh rock@<ip>`)
  ```
  rossetip
  rossetlocal
  sudo systemctl stop roscore.service
  roscore
  ```
  としてroscoreを立ち上げたのち、全てのデバイスを繋いだことを確認してから
  ```
  cd /jishupuro
  ./setup.sh
  ```
  とする。

- PC側では
  ```
  rossetip
  rossetmaster <radoxa’s ip>
  cd jishupuro
  python3 hume_sound_ros_denden.py
  ```
- rossetip, rossetlocal/master はターミナルを立ち上げるたびに行う必要があることに注意されたい。

### radxaのwifi設定
```
sudo su
nmcli r wifi on
nmcli dev wifi で見えるnetworkを確認．
nmcli dev wifi connect “SSID” password “PASSWORD”などとして接続．
```

## supported device
M5Stack用NeoPixel互換LED搭載 HEXボード: https://www.switch-science.com/products/6058?_pos=1&_sid=04326fe49&_ss=r


## hume_sound_ros.py, hume_sound_ros_denden.pyについて
- hume_sound_ros_denden.pyは、hume_sound_ros.pyに着信音の再生を加えたもの
- audiobuffer(L141くらい)のtopic_nameを適切に設定する必要がある。
  * 学科PCでは `topic_name='/audio/audio',`
  * Respeakerでは `topic_name = '/audio/audio', `

## setup.shについて
### マイクの立ち上げ
- 学科PCでは、必要なら`sudo apt install ros-noetic-audio-capture`してから、
  ```
  roslaunch audio_capture capture_wave.launch
  ```

### eye_module, m5stackの準備
- これらのデバイスでrosserialが使えるようにするには、 `ls /dev/ttyA*`　としてポート名を確認し、
  ```
  rosrun rosserial_python serial_node.py _port:=<port> _baud:=57600
  ```
  とすれば良い。
- 複数デバイスを立ち上げたいときは `__name:=a`　を上記のrosrunの末尾につければ良い。
- eye_moduleがきちんとeye_statusを受け取っているかは
  ```
  rostopic pub -1 /eye_status std_msgs/UInt16 "data: 0"
  ```
  で確認できる
- デバイス名の固定は、
  ```
  udevadm info -q property -n /dev/ttyA* | grep -E "ID_SERIAL_SHORT=|ID_VENDOR_ID=|ID_MODEL_ID="
  ```
  としてデータを取得し、
  ```
  cd /etc/udev/rules.d
  sudo emacs -nw
  ```
  として　99-usb-<SERIAL_SHORT>.rulesに
  ```
  SUBSYSTEM=="tty", ATTRS{idVendor}=="**", ATTRS{idProduct}=="**", ATTRS{serial}=="**", SYMLINK+="ttyACM-righteye", GROUP="dialout"
  ```
  などと書き込んだのち、
  ```
  sudo udevadm control --reload
  sudo udevadm trigger
  ```
  とすれば良い。

これらをまとめたのがsetup.shであり、`./setup.sh`で起動できる。

