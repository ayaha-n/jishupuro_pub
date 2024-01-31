import asyncio
from dotenv import load_dotenv
load_dotenv()
import os
from hume import HumeStreamClient, StreamSocket
from hume.models.config import ProsodyConfig

async def main():
    client = HumeStreamClient(os.getenv('api_key'))
    config = ProsodyConfig()
    async with client.connect([config]) as socket:
        result = await socket.send_file("./sound_rec/sound_rec.wav")
        #print(result)
        for pred in result['prosody']['predictions']:
                emotions_sorted = sorted(pred['emotions'], key=lambda x: -x['score'])
                for emotion in emotions_sorted:
                    print(emotion)


asyncio.run(main())
