import asyncio, json, base64, io
import numpy as np, soundfile as sf, websockets

URL = "ws://115.190.27.75:19026"

async def tts(text, prompt_wav, prompt_text, out_wav):
    # 1. 读取参考音频，重采样到 16kHz 单声道
    audio, sr = sf.read(prompt_wav)
    if audio.ndim > 1:
        audio = audio.mean(axis=1)
    if sr != 16000:
        from scipy.signal import resample
        audio = resample(audio, int(len(audio) * 16000 / sr)); sr = 16000
    buf = io.BytesIO()
    sf.write(buf, audio, sr, format='WAV', subtype='PCM_16')
    prompt_b64 = base64.b64encode(buf.getvalue()).decode()

    # 2. 发送合成请求
    async with websockets.connect(URL, max_size=50*1024*1024) as ws:
        await ws.send(json.dumps({
            "command": "synthesize",
            "params": {
                "text": text,
                "mode": "zero_shot",
                "stream": False,
                "prompt_text": prompt_text,
                "prompt_audio": prompt_b64
            }
        }))
        res = json.loads(await ws.recv())
        if "error" in res:
            print("错误:", res["error"]); return
        # 3. 解码 base64 PCM 保存为 wav
        pcm = np.frombuffer(base64.b64decode(res["audio"]), dtype=np.int16)
        sf.write(out_wav, pcm.astype(np.float32) / 32768, res["sample_rate"])
        print(f"已保存 {out_wav}, 时长 {res['duration']:.2f}s")

asyncio.run(tts(
    "你好，这是语音合成测试。",
    "参考音频.wav",           # 你的参考音频（任意时长的一段人声）
    "参考音频对应的文字内容",   # 参考音频里说的话
    "output.wav"
))
客户端依赖：pip install websockets soundfile numpy scipy