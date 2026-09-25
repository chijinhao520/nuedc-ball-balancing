"""串口输出 —— 把视觉处理结果打包成帧，通过 UART 发给 MSPM0。

物理链路：X5 /dev/ttyS1（40pin pin8=TXD / pin10=RXD）↔ 逻辑板视觉口 ↔ MSPM0 UART1(PB4/PB5)，
交叉接线 + 共地，115200 8N1。

帧格式（定长 9 字节，小端，MCU 好解析）：
  字节:  0    1    2      3~4        5       6~7      (末)
        0xAA 0x55 TYPE  steer_i16  flags   aux_i16   XOR
  TYPE : 0x01=循迹(line/seg)  0x02=颜色(color)  0x03=检测(detect)
         0x04=槽内球位置(ball_groove)
  steer: int16 = steering*1000（-1000~+1000），无转向量填 0
         TYPE=0x04 时该字段改载 x_est_cm*100（单位 0.01cm，量程 ±1250）——
         发的是 alpha-beta 观测器平滑后的位置，不是原始测量：X5 侧有全精度
         浮点与真实时间戳，比 MCU 侧再滤一遍更准；MCU 用 x_est + v 外推到
         自己的控制频率即可（实际周期见任务表）。
  flags: bit0 = found/valid    bit1 = ambiguous（疑似干扰/双峰，仅 0x04）
         bit2 = 观测器有效（仅 0x04；失锁超 predict_ms 后清零，此时 aux 不可用）
  aux  : int16，循迹=branches / 颜色=count / 检测=count
         TYPE=0x04 = 球速 v*10（单位 0.1cm/s；int16 编码范围 -3276.8..3276.7cm/s）
  XOR  : TYPE..aux(高字节) 共 6 字节异或校验

本仓库 firmware/common/vision_link.c 已支持 TYPE=0x04，位置限值为 ±1300。
当前 MCU 快照只保留 flags bit0；bit1/bit2 虽随帧发送但未向应用层暴露。
发送端 steer_raw 按 int16 限幅；球位超出接收端 ±1300 时会被拒收。
旧版本接收器若仍只允许 TYPE 1..3 或 ±1000，会把球位帧记为 bad_payload。
当前任务表中球位控制为 50Hz、摆杆执行器服务为 200Hz；两者不是同一周期。
"""
import struct
import threading
import time
import serial

FRAME_HEAD = b'\xAA\x55'
TYPE_LINE = 0x01
TYPE_COLOR = 0x02
TYPE_DETECT = 0x03
TYPE_BALL = 0x04


class SerialOut:
    def __init__(self, port="/dev/ttyS1", baud=115200):
        self.port = port
        self.baud = baud
        self._ser = None
        self._lock = threading.Lock()
        self.tx_count = 0
        self.ok = False
        # M0 状态上行（同一根线的反方向，交叉接线本就双向）：
        # M0 每 200ms 发一行 "[m0] m=3 st=2 t=1234 x=-123 pk=520"，
        # 这里解析成 dict 供网页显示。计时权威源在 M0，X5 只显示。
        self.m0 = {}
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            self.ok = True
            print(f"[serial_out] {port} @ {baud} 打开成功")
            threading.Thread(target=self._read_loop, daemon=True).start()
        except Exception as e:
            print(f"[serial_out] 打开 {port} 失败: {e}（串口输出禁用，视觉仍正常）")

    def _read_loop(self):
        buf = b''
        while self.ok:
            try:
                data = self._ser.read(64)
            except Exception:
                time.sleep(0.5)
                continue
            if not data:
                continue
            buf += data
            if len(buf) > 1024:
                buf = buf[-256:]          # 乱码防积压
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self._parse_m0(line.decode('ascii', 'ignore').strip())

    def _parse_m0(self, line):
        if not line.startswith('[m0]'):
            return                        # M0 的 CLI 回显等其它行一律忽略
        d = {}
        for tok in line[4:].split():
            k, _, v = tok.partition('=')
            if v.lstrip('-').isdigit():
                d[k] = int(v)
        if d:
            d['ts'] = time.time()
            self.m0 = d

    def send(self, type_id, steer=0.0, found=False, aux=0, flags_extra=0,
             steer_raw=None):
        """steer_raw 给定时直接作为 int16 载荷，绕过 steering*1000 的映射与
        ±1000 限幅（球位置用 x_cm*100，量程比转向量宽）。"""
        if not self.ok:
            return
        steer_i = (max(-32768, min(32767, int(round(steer_raw)))) if steer_raw is not None
                   else max(-1000, min(1000, int(round(steer * 1000)))))
        aux_i = max(-32768, min(32767, int(aux)))
        flags = (0x01 if found else 0x00) | (flags_extra & 0xFE)
        body = struct.pack('<BhBh', type_id, steer_i, flags, aux_i)
        xor = 0
        for b in body:
            xor ^= b
        frame = FRAME_HEAD + body + bytes([xor])
        with self._lock:
            try:
                self._ser.write(frame)
                self.tx_count += 1
            except Exception:
                pass

    def send_result(self, mode, result):
        """按处理器 result 字典自动映射帧字段。"""
        if mode in ("line", "seg"):
            self.send(TYPE_LINE, result.get("steer", 0.0),
                      result.get("found", False), result.get("branches", 0))
        elif mode == "color":
            self.send(TYPE_COLOR, 0.0, result.get("count", 0) > 0,
                      result.get("count", 0))
        elif mode in ("detect", "ball_yolo"):
            self.send(TYPE_DETECT, 0.0, result.get("count", 0) > 0,
                      result.get("count", 0))
        elif mode in ("ball_groove", "ball_yolo1d"):
            # ball_yolo1d 继承 BallGroove1D，result 字段同构（x_est_cm/v_cm_s/
            # obs/ambiguous 都在），走同一 TYPE_BALL 帧。漏掉它时的症状是
            # 视觉一切正常但 M0 一帧球数据都收不到（send_result 静默不匹配）。
            # 失锁时仍按 found=0 发帧：让 MSPM0 侧区分「链路断」（新鲜度超时）
            # 与「链路通但没锁住球」（fresh=1/valid=0），两者处置不同。
            flags = (0x02 if result.get("ambiguous") else 0x00) | \
                    (0x04 if result.get("obs") else 0x00)
            self.send(TYPE_BALL, found=result.get("found", False),
                      aux=round(result.get("v_cm_s", 0.0) * 10.0),
                      flags_extra=flags,
                      steer_raw=round(result.get("x_est_cm", 0.0) * 100.0))

    def close(self):
        if self._ser:
            self._ser.close()
