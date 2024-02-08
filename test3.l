;;
(ros::roseus "test3")
(load "package://pr2eus/speak.l")
(play-sound
 (pathname "~/jishupuro_pub/purugacha.wav"
)
	   :topic-name "/robotsound_jp"
	   :wait t)
