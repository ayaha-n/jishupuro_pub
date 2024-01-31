#!/bin/bash

# スクリプトを実行するディレクトリに移動する（必要に応じて変更）
cd ~/jishupuro

# launch ファイルを実行
roslaunch capture.launch &

# 各シリアルノードを実行
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM-righteye _baud:=57600 __name:=b &
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM-lefteye _baud:=57600 __name:=a &
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM-m5stack _baud:=57600 __name:=c &

# 待機してすべてのプロセスが終了するまで待つ
wait
