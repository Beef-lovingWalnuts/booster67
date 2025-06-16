import pyaudio

p = pyaudio.PyAudio()
print("=== 可用输入设备列表 ===")
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    if info["maxInputChannels"] > 0:
        print(f"Index {i}: {info['name']} - 输入通道: {info['maxInputChannels']}")
p.terminate()