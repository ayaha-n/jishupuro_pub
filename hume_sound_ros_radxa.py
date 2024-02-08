from __future__ import division

from dotenv import load_dotenv
load_dotenv()
import os

import shutil
import array
import asyncio
from base64 import b64encode
from io import BytesIO
from threading import Lock
import traceback
import time
from datetime import datetime
import websockets

from audio_common_msgs.msg import AudioData
from hume import HumeStreamClient
from hume.models.config import BurstConfig
from hume.models.config import ProsodyConfig
from hume import StreamSocket
import numpy as np
from pydub import AudioSegment
from pydub import effects
import rospy
import soundfile as sf

# from audio_buffer import AudioBuffer
from std_msgs.msg import UInt16
from ros_speak import play_sound
from pathlib import Path
class AudioBuffer(object):

    def __init__(self, topic_name='~audio',
                 input_sample_rate=16000,
                 window_size=10.0,
                 bitdepth=16,
                 n_channel=1, target_channel=0,
                 get_latest_data=False,
                 discard_data=False,
                 auto_start=False):
        self.is_subscribing = True
        self.get_latest_data = get_latest_data
        self.discard_data = discard_data
        self._window_size = window_size
        self.audio_buffer_len = int(self._window_size * input_sample_rate)
        self.lock = Lock()
        self.bitdepth = bitdepth
        self.n_channel = n_channel
        self.target_channel = min(self.n_channel - 1, max(0, target_channel))
        self.input_sample_rate = input_sample_rate
        self.type_code = {}
        for code in ['b', 'h', 'i', 'l']:
            self.type_code[array.array(code).itemsize] = code

        self.dtype = self.type_code[self.bitdepth / 8]
        self.audio_buffer = np.array([], dtype=self.dtype)

        self.max_value = 2 ** (self.bitdepth - 1) - 1

        self.topic_name = topic_name

        if auto_start:
            self.subscribe()

    def __len__(self):
        return len(self.audio_buffer)

    @property
    def window_size(self):
        return self._window_size

    @window_size.setter
    def window_size(self, size):
        with self.lock:
            self._window_size = size
            self.audio_buffer_len = int(self._window_size
                                        * self.input_sample_rate)
            self.audio_buffer = np.array([], dtype=self.dtype)

    @staticmethod
    def from_rosparam(**kwargs):
        n_channel = rospy.get_param('~n_channel', 1)
        target_channel = rospy.get_param('~target_channel', 0)
        mic_sampling_rate = rospy.get_param('~mic_sampling_rate', 16000)
        bitdepth = rospy.get_param('~bitdepth', 16)
        return AudioBuffer(input_sample_rate=mic_sampling_rate,
                           bitdepth=bitdepth,
                           n_channel=n_channel,
                           target_channel=target_channel,
                           **kwargs)

    def subscribe(self):
        self.audio_buffer = np.array([], dtype=self.dtype)
        self.sub_audio = rospy.Subscriber(
            self.topic_name, AudioData, self.audio_cb)

    def unsubscribe(self):
        self.sub_audio.unregister()

    def _read(self, size, normalize=False):
        with self.lock:
            if self.get_latest_data:
                audio_buffer = self.audio_buffer[-size:]
            else:
                audio_buffer = self.audio_buffer[:size]
                if self.discard_data:
                    self.audio_buffer = self.audio_buffer[size:]
        if normalize is True:
            audio_buffer = audio_buffer / self.max_value
        return audio_buffer

    def sufficient_data(self, size):
        return len(self.audio_buffer) < size

    def read(self, size=None, wait=False, normalize=False):
        if size is None:
            size = self.audio_buffer_len
        size = int(size * self.input_sample_rate)
        while wait is True \
                and not rospy.is_shutdown() and len(self.audio_buffer) < size:
            rospy.sleep(0.001)
        return self._read(size, normalize=normalize)

    def close(self):
        try:
            self.sub_audio.unregister()
        except Exception:
            pass
        self.audio_buffer = np.array([], dtype=self.dtype)

    def audio_cb(self, msg):
        audio_buffer = np.frombuffer(msg.data, dtype=self.dtype)
        audio_buffer = audio_buffer[self.target_channel::self.n_channel]
        with self.lock:
            self.audio_buffer = np.append(
                self.audio_buffer, audio_buffer)
            self.audio_buffer = self.audio_buffer[
                -self.audio_buffer_len:]


rospy.init_node('hume_sound')


audio_buffer = AudioBuffer(
    # topic_name='/left_ear/audio',
    topic_name='/audio',
    #topic_name = '/audio/audio',
    window_size=2,
    auto_start=True)
#api_key = os.getenv('api_key')

api_key = os.environ.get('HUME_API')

def on_shutdown():
    print('on_shutdown called')
    audio_buffer.unsubscribe()
    audio_buffer.audio_buffer = np.array([], dtype=audio_buffer.dtype)

rospy.on_shutdown(on_shutdown)
rospy.sleep(1.0)


wave_counter = 0

def encode_audio():
    global wave_counter
    wav_outpath = '/tmp/hoge.wav'
    bytes_io = BytesIO()
    with sf.SoundFile(wav_outpath, mode='w',
                      samplerate=audio_buffer.input_sample_rate,
                      channels=audio_buffer.n_channel,
                      format='wav') as f:
        tmp = audio_buffer.read()
        f.write(tmp)

    segment = AudioSegment.from_file(file=wav_outpath,
                                     format="wav")
    segment.export(bytes_io, format="wav")
    shutil.copy(wav_outpath, '/tmp/hoge{}.wav'.format(wave_counter))
    wave_counter += 1
    return b64encode(bytes_io.read())


client = HumeStreamClient(api_key)
configs = [BurstConfig(),ProsodyConfig()]

async def classify_emotions(pred):
#humeにおける感情分析では種類が多すぎるので7種に分割
    star_scores = []
    heart_scores = []
    sad_scores = []
    angry_scores = []
    tired_scores = []
    happy_scores = []
    neutral_scores = []

    for emotion in pred['emotions']: #predは辞書であってリストではないので上の行とまとめて書くのは無理
        if emotion['name'] in ['Excitement','Interest','Admiration','Surprise (positive)','desire','Triumph', 'Ecstasy', 'Joy','Satisfaction','Amusement', 'Contentment']: #orで結ぶならif emotion['name'] == 'Excitement' or emotion['name'] == 'Interest' or ...
            star_scores.append(emotion['score'])
        elif emotion['name'] in ['Adoration','Love','Entrancement','Romance', 'Relief','Aesthetic Appreciation']:
            heart_scores.append(emotion['score'])
        elif emotion['name'] in ['Awkwardness','Disappointment','Distress','Anxiety','Sadness','Pain','Surprise (negative)','Fear','Empathic Pain','Horror']:
            sad_scores.append(emotion['score'])  
        elif emotion['name'] in ['Anger', 'Disgust']:                                                     
            angry_scores.append(emotion['score'])  
        elif emotion['name'] in ['Boredom','Tiredness','Contempt','Doubt']:     
            tired_scores.append(emotion['score'])
       # elif emotion['name'] in []:
        #    happy_scores.append(emotion['score'])
        elif emotion['name'] in ['Calmness','Awe','Confusion','Embarrassment','Envy','Sympathy','Pride','Realization','Determination','Nostalgia','Craving','Concentration','Contemplation','Guilt','Shame']:                
            neutral_scores.append(emotion['score'])
            
    avg_star_score = sum(star_scores)/len(star_scores) if star_scores else 0
    avg_heart_score = sum(heart_scores)/len(heart_scores) if heart_scores else 0  
    avg_sad_score = sum(sad_scores)/len(sad_scores) if sad_scores else 0  
    avg_angry_score = sum(angry_scores)/len(angry_scores) if angry_scores else 0  
    avg_tired_score = sum(tired_scores)/len(tired_scores) if tired_scores else 0  
    #avg_happy_score = sum(happy_scores)/len(happy_scores) if happy_scores else 0  
    avg_neutral_score = sum(neutral_scores)/len(neutral_scores) if neutral_scores else 0

    state = max([
        ('star', avg_star_score),
        ('heart', avg_heart_score),
        ('sad', avg_sad_score),
        ('angry', avg_angry_score),
        ('tired', avg_tired_score),
     #   ('happy', avg_happy_score),
        ('neutral', avg_neutral_score)
    ], key=lambda x: x[1])
    global state_name
    state_name, state_score = state
    
    print(f"The dominant emotion is {state_name} with a score of {state_score}")
#    await asyncio.sleep(1)

    
async def set_eye_status(state_name):                                                                                                                                          
    if state_name == 'neutral':                                                                                                                                                
        data_value = 1                                                                                                                                                         
    elif state_name == 'sad':                                                                                                                                                  
        data_value = 5                                                                                                                                                         
    elif state_name == 'tired':                                                                                                                                                
        data_value = 3                                                                                                                                                         
    elif state_name == 'angry':                                                                                                                                                
        data_value = 4                                                                                                                                                         
    elif state_name == 'star':                                                                                                                                                
        data_value = 6  
    elif state_name == 'heart':                                                                                                                                                
        data_value = 7                                                                                                                                                                  
    else:                                                                                                                                                    
        data_value = 0  # デフォルトの値                                                                                                                                       
                                                                                                                                                                               
    # UInt16型のメッセージを作成し、dataを設定してパブリッシュ                                                                                                                 
    eye_status_message = UInt16(data=data_value)                                                                                                                               
    eye_status_publisher.publish(eye_status_message) 

async def send_audio():
    global state_name
    try:
        async with client.connect(configs) as socket:
            socket: StreamSocket

            while True:
                result = None
                await socket.reset_stream()
                send_bytes = encode_audio()
                print(len(send_bytes))
                if len(send_bytes) <= 100:
                    break
                #time1=datetime.now()
                try:
                    result = await socket.send_bytes(send_bytes)
                except websockets.exceptions.ConnectionClosedError as e:
                    print("send bytes failed")
                    continue
                #time2 = datetime.now()
                #time.sleep(1.0)
                print(result)
                if 'predictions' in result['prosody']:
                    for pred in result['prosody']['predictions']:
                        emotions_sorted = sorted(pred['emotions'], key=lambda x: -x['score'])
                        emotions_sorted = emotions_sorted[:8] #score上位８個を取り出す
                        #for emotion in emotions_sorted:
                            #print(emotion)
                            #print(emotions_sorted[0])
                        await classify_emotions(pred)
                    await set_eye_status(state_name)
                    
                if 'predictions' in result['burst']:
                    for pred in result['burst']['predictions']:
                        emotions_sorted = sorted(pred['emotions'], key=lambda x: -x['score'])
                        emotions_sorted = emotions_sorted[:8] #score上位８個を取り出す
                        #for emotion in emotions_sorted:
                            #print(emotion)
                            #print(emotions_sorted[0])
                        await classify_emotions(pred)
                    await set_eye_status(state_name)
               
               
                #end = datetime.now()
                #print(time2-time1)
                #print(end-time2)
                
                await asyncio.sleep(1)

    except Exception:
        print(traceback.format_exc())



#main関数
# ROSノードを初期化
#rospy.init_node('eye_status_publisher', anonymous=True)
#play_sound('package://rostwitter/resource/camera.wav', topic_name='robotsound_jp')
#play_sound(sound='home/rock/jishupuro_pub/purugacha.wav', topic_name='robotsound_jp')
#file_path = Path("/home/rock/jishupuro_pub/purugacha.wav")
#play_sound(sound=str(file_path), topic_name='robotsound_jp')
play_sound('package://jishupuro_pkg/resource/purugacha.wav', topic_name='robotsound_\
jp', wait=True) 

# Publisherを作成
eye_status_publisher = rospy.Publisher('/eye_status', UInt16, queue_size=1)

# グローバル変数としてstate_nameを定義
state_name = 'neutral'  # 初期値を設定

asyncio.run(send_audio())
