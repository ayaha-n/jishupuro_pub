#!/bin/bash

# スクリプトを実行するディレクトリに移動する（必要に応じて変更）
cd ~/jishupuro_pub

# launch ファイルを実行
roslaunch capture.launch &
roslaunch stereo_image_sandbox speech_recognition_radxa.launch device:="\"bluealsa:DEV=64:B7:08:82:B6:12,PROFILE=a2dp\"" &

# 各シリアルノードを実行
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM-righteye _baud:=57600 __name:=b &
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM-lefteye _baud:=57600 __name:=a &
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM-m5stack _baud:=57600 __name:=c &

# bluetoothスピーカの立ち上げ
# python ~/rcb4eus/radxa/speak-jp/make_green.pyの内容
# Pythonプログラムを実行してブロックする
python3 -c "
from bluezero import adapter
from bluezero import device

dongles = adapter.list_adapters()
print('dongles available: ', dongles)
dongle = adapter.Adapter(dongles[0])
tmp = device.Device(dongle.address, '64:B7:08:82:B6:12')
tmp.connect()
"

#gnome-terminal -- roslaunch stereo_image_sandbox speech_recognition_radxa.launch device:="\"bluealsa:DEV=64:B7:08:82:B6:12,PROFILE=a2dp\""

# 待機してすべてのプロセスが終了するまで待つ
wait

#bash ~/rcb4eus/radxa/speak-jp/run-speak-jp.shの内容
#roslaunch stereo_image_sandbox speech_recognition_radxa.launch device:="\"bluealsa:DEV=64:B7:08:82:B6:12,PROFILE=a2dp\""
#gnome-terminal -- roslaunch stereo_image_sandbox speech_recognition_radxa.launch device:="\"bluealsa:DEV=64:B7:08:82:B6:12,PROFILE=a2dp\""

#echo "All processes have prepared."
