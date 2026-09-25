"""处理器基类 —— 所有视觉处理模块的统一接口。

约定：
- process(frame): 收到 BGR 原始帧，返回 (标注后的 BGR 帧, result 字典)。
  result 字典里放要发给 MCU 的量（如 steering / 目标坐标），
  串口输出层和网页 /info 都从这里取。
- on_key(key): 可选。接收网页/命令传来的字符串参数做运行时调整
  （如循迹切换识别的线颜色）。
"""


class Processor:
    name = "base"

    def process(self, frame):
        return frame, {}

    def on_key(self, key):
        pass
