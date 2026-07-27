#!/usr/bin/env python3
"""
Fun-ASR 流式语音识别服务客户端示例
支持从任意服务器请求 ASR 服务
"""

import asyncio
import json
import sys

import numpy as np
import soundfile as sf
import websockets

# ASR 服务地址
ASR_URL = "ws://115.190.27.75:19012"


async def asr_recognize(audio_file: str, server_url: str = ASR_URL):
    """
    调用 ASR 服务识别音频文件
    
    Args:
        audio_file: 音频文件路径（支持 wav/mp3/flac 等）
        server_url: ASR 服务地址
    
    Returns:
        str: 识别结果文本
    """
    print(f"【ASR 语音识别】")
    print(f"服务地址: {server_url}")
    print(f"音频文件: {audio_file}\n")
    
    # 1. 读取音频并转为 16kHz 单声道
    try:
        audio, sr = sf.read(audio_file)
    except Exception as e:
        print(f"✗ 读取音频失败: {e}")
        return None
    
    if audio.ndim > 1:
        audio = audio.mean(axis=1)
    
    print(f"原始采样率: {sr} Hz")
    
    if sr != 16000:
        from scipy.signal import resample
        audio = resample(audio, int(len(audio) * 16000 / sr))
        sr = 16000
    
    duration = len(audio) / 16000
    print(f"音频时长: {duration:.2f} 秒")
    print(f"目标采样率: 16000 Hz\n")
    
    # 2. 连接 WebSocket 服务
    try:
        async with websockets.connect(server_url, max_size=10*1024*1024) as ws:
            # 3. 发送开始信号
            await ws.send(json.dumps({
                "type": "start",
                "sample_rate": 16000,
                "chunk_interval": 0.3  # 服务端每 300ms 处理一次
            }))
            
            start_resp = json.loads(await ws.recv())
            print(f"会话已建立: {start_resp.get('session_id')}\n")
            
            # 4. 启动接收任务（处理流式返回的识别结果）
            results = []
            
            async def receive_results():
                """接收识别结果"""
                while True:
                    try:
                        # 调整超时：长音频最后一次完整识别可能需要更长时间
                        msg = await asyncio.wait_for(ws.recv(), timeout=5)
                        data = json.loads(msg)
                        results.append(data)
                        
                        if data.get("status") == "success":
                            text = data.get("text", "")
                            is_final = data.get("is_final", False)
                            
                            if is_final:
                                print(f"\n【最终结果】 {text}")
                                break
                            else:
                                # 显示中间结果（可选，便于调试）
                                print(f"【识别中】 {text[:80]}{'...' if len(text) > 80 else ''}")
                        
                        elif data.get("status") == "stopped":
                            print("会话已停止")
                            break
                    
                    except asyncio.TimeoutError:
                        print("接收超时")
                        break
            
            recv_task = asyncio.create_task(receive_results())
            
            # 5. 分块发送原始 PCM 音频（关键：发二进制，不是 base64）
            chunk_size = int(16000 * 0.1)  # 100ms 一块
            num_chunks = (len(audio) + chunk_size - 1) // chunk_size
            print(f"发送音频数据（{num_chunks} 块）...")
            
            for i in range(num_chunks):
                start_idx = i * chunk_size
                end_idx = min((i + 1) * chunk_size, len(audio))
                chunk = audio[start_idx:end_idx]
                
                # 转为 16-bit PCM 并发送原始二进制
                pcm = (chunk * 32767).astype(np.int16)
                await ws.send(pcm.tobytes())
                
                # 模拟实时音频流（可选，服务端会缓冲处理）
                await asyncio.sleep(0.05)
            
            print("音频发送完毕\n")
            
            # 6. 发送停止信号，触发最终识别
            await ws.send(json.dumps({"type": "stop"}))
            print("停止信号已发送，等待最终结果...\n")
            
            # 7. 等待接收完成
            await recv_task
            
            # 8. 提取最终结果
            final_results = [r for r in results if r.get("is_final")]
            
            if final_results:
                final_text = final_results[0].get("text", "")
                print(f"\n{'='*60}")
                print(f"✅ 识别成功")
                print(f"{'='*60}")
                print(final_text)
                print(f"{'='*60}\n")
                return final_text
            else:
                # 如果没有标记 is_final 的结果，取最后一条
                if results:
                    last_text = results[-1].get("text", "")
                    print(f"\n⚠️  未收到最终标记，返回最后一条识别结果:")
                    print(last_text)
                    return last_text
                else:
                    print("\n✗ 未收到任何识别结果")
                    return None
    
    except ConnectionRefusedError:
        print(f"✗ 无法连接到服务器: {server_url}")
        print("  请检查服务是否启动")
        return None
    
    except Exception as e:
        print(f"✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        return None


async def test_ping(server_url: str = ASR_URL):
    """测试服务连通性"""
    try:
        async with websockets.connect(server_url) as ws:
            await ws.send(json.dumps({"type": "ping"}))
            resp = json.loads(await asyncio.wait_for(ws.recv(), timeout=3))
            print(f"✓ 服务正常: {resp}")
            return True
    except Exception as e:
        print(f"✗ 服务异常: {e}")
        return False


async def main():
    """主函数"""
    if len(sys.argv) < 2:
        print("用法: python asr_client_demo.py <音频文件路径>")
        print("\n示例:")
        print("  python asr_client_demo.py test.wav")
        print("  python asr_client_demo.py /path/to/audio.mp3")
        print("\n测试连通性:")
        print("  python asr_client_demo.py --ping")
        return
    
    if sys.argv[1] == "--ping":
        await test_ping()
        return
    
    audio_file = sys.argv[1]
    result = await asr_recognize(audio_file)
    
    if result:
        print(f"识别字数: {len(result)} 字")


if __name__ == "__main__":
    asyncio.run(main())
