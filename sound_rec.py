import sys, os
import pyaudio
import wave 

def audiostart():
    audio = pyaudio.PyAudio() 
    stream = audio.open( format = pyaudio.paInt16,
                         rate = 44100,
                         channels = 1, 
                         input_device_index = 0,
                        input = True, 
                         frames_per_buffer = 256)
    return audio, stream

def audiostop(audio, stream):
    stream.stop_stream()
    stream.close()
    audio.terminate()

def read_plot_data(stream):
    data = stream.read(1024)
    return data
    
    
def rec_exec(file_path):
    # 録音データをファイルに保存
    try:
        # ディレクトリが存在しない場合は作成
        os.makedirs(os.path.dirname(file_path), exist_ok=True)

        # ファイルが存在する場合は新しいファイル名を生成
        base, ext = os.path.splitext(file_path)
        index = 1
        while os.path.exists(file_path):
            file_path = f"{base}_{index}{ext}"
            index += 1

        wave_f = wave.open(file_path, 'wb')
        wave_f.setnchannels(1)
        wave_f.setsampwidth(2)
        wave_f.setframerate(44100)
        wave_f.writeframes(b''.join(rec_data))
        wave_f.close()
    except Exception as e:
        print(f"Error saving file: {e}")
    
    
    
    
    

if __name__ == '__main__':
    args = sys.argv
    
    if len(args) ==2:
        # Audio インスタンス取得
        (audio,stream) = audiostart()
        
        rec_data = []
        print("Start")
        # 音声を読み出し
        while True:
            try:
                data = read_plot_data(stream)
                rec_data.append(data)
            except KeyboardInterrupt:
                print("stop")
                break
         # Audio デバイスの解放
        audiostop(audio,stream)

        #保存実行
        rec_exec( args[1] )
