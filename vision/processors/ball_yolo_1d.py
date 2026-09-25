"""球位一维检测 —— YOLO 检出 + 复用 BallGroove1D 的全部下游。

为什么要这个类，而不是直接用 BallYoloBPU：
  BallYoloBPU 只输出框（score/box/center），而控制脚本要的是 x_cm 与 v_cm_s。
  位置标定（poly）、alpha-beta 观测器、跳变门控、检出率统计、`/key/caln` 标定
  命令、save/load —— 这些在 BallGroove1D 里全都有且已实测可用，没有理由重写。
  故这里只把「怎么找到球」换成 YOLO，下游一行不改，控制脚本零改动。

为什么要换掉 CV 检测：
  BallGroove1D 靠空槽背景相减压制管内刻度尺纹理（球对比度约 29，而纹理噪声
  抬高的门限可达 62），因此**背景必须在当前摆杆角度采**。07-31 实测：背景在
  tgt10=710 采集后，摆杆动到 716（+0.6° 电机角 = 0.13° 摆杆角）就立刻失锁，
  20 步扫描全是 nan。而控制需要 ±3° 摆杆角的活动范围 —— 背景相减在动态下
  根本不成立。YOLO 不吃背景，这正是它要解决的问题。

ROI 仍然生效：YOLO 全图检测会把管外的球（桌上、手里）也检出来，故只接受
框中心落在 roi 的 [ul, ur] 横向区间、且纵向落在管带内的目标。
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

from .ball_groove_1d import BallGroove1D
from .yolo_seg_post import (
    decode_yolov8_detect,
    decode_yolov8_detect_quantized,
    letterbox_bgr2nv12,
)
try:
    from hobot_dnn import pyeasy_dnn as dnn
except ImportError:
    from hobot_dnn_rdkx5 import pyeasy_dnn as dnn


class BallYolo1D(BallGroove1D):
    name = "ball_yolo1d"
    MODEL_NAME = "steel_ball_yolov8n_detect_640.bin"
    DETECT_HEAD_SIZES = (80, 80, 40, 40, 20, 20)
    DETECT_HEAD_CHANNELS = (64, 1, 64, 1, 64, 1)

    @classmethod
    def default_model_path(cls) -> Path:
        # 用独立变量：BallYoloBPU 也读 STEEL_BALL_BPU_MODEL，共用会让两个处理器
        # 各加载一份同一模型（多占几秒启动与一份 BPU 内存）。
        configured = (os.environ.get("STEEL_BALL_1D_BPU_MODEL")
                      or os.environ.get("STEEL_BALL_BPU_MODEL"))
        if configured:
            return Path(configured)
        return Path(__file__).resolve().parents[1] / "models" / cls.MODEL_NAME

    # 阈值为何这么低：PTQ 全 INT8 量化把分类头的**绝对置信度**压垮了 ——
    # 同一帧 PC float 模型给 0.851，板端量化模型只给 0.1063（logit 从 +1.74
    # 掉到 -2）。但掉的只是分数标定，定位没掉：两者球心只差 0.5px。
    #
    # 关键在于**分离度靠的不是分数高低，而是有没有检出**：实测空管即使把
    # 阈值压到 0.005 也是全图零检出，而有球时各位置分数为
    # 中段 0.106 / 左端 0.039 / 右端管口遮挡处 0.013。故阈值要匹配量化后的
    # 分数量级，而不是去追 float 模型的 0.85。取 0.008 覆盖最不利的右端遮挡位。
    # （试过 p3_class_int16 混合精度想把分数拉回来，实测 0.1047 vs 0.1063
    #  几乎无差别 —— 掉的是分数标定，不是 P3 分类头精度能救的。）
    def __init__(self, model=None, score: float = 0.008, iou: float = 0.4):
        super().__init__()
        self.model_path = Path(model) if model else self.default_model_path()
        if not self.model_path.exists():
            raise FileNotFoundError(f"钢珠 BPU 模型不存在: {self.model_path}")
        models = dnn.load(str(self.model_path))
        if not models:
            raise RuntimeError(f"BPU 模型加载失败: {self.model_path}")
        self.model = models[0]
        self.score = score
        self.iou = iou
        self.infer_ms = 0.0
        # 纵向容差：框中心偏离管带中心线超过此值即判为管外目标（像素）
        self.y_tol = 60.0

    # ---------- 推理 ----------

    @staticmethod
    def _float_output(output) -> np.ndarray:
        array = np.asarray(output.buffer)
        scales = np.asarray(output.properties.scale_data,
                            dtype=np.float32).reshape(-1)
        if scales.size == 0 or np.issubdtype(array.dtype, np.floating):
            return array.astype(np.float32, copy=False)
        if scales.size == 1:
            return array.astype(np.float32) * scales[0]
        for axis, size in enumerate(array.shape):
            if size == scales.size:
                shape = [1] * array.ndim
                shape[axis] = scales.size
                return array.astype(np.float32) * scales.reshape(shape)
        raise ValueError(f"输出量化系数 {scales.size} 与张量 {array.shape} 不匹配")

    @classmethod
    def _restore_detect_outputs(cls, outputs):
        """BPU 输出按 4 字节对齐 pad 过，需按 (C, side, side) 还原并裁掉尾部。"""
        restored = []
        for index, (array, side, channels) in enumerate(zip(
                outputs, cls.DETECT_HEAD_SIZES, cls.DETECT_HEAD_CHANNELS)):
            if array.ndim != 4:
                raise ValueError(f"第 {index} 路 detect 输出不是 4 维: {array.shape}")
            batch = array.shape[0]
            expected = (batch, channels, side, side)
            if array.shape == expected:
                restored.append(array)
                continue
            if batch == 0 or array.size % (batch * channels):
                raise ValueError(f"第 {index} 路 detect 输出形状非法: {array.shape}")
            packed = np.ascontiguousarray(array).reshape(batch, channels, -1)
            valid = side * side
            if packed.shape[2] < valid:
                raise ValueError(f"第 {index} 路 packed 长度不足: "
                                 f"{packed.shape[2]} < {valid}")
            restored.append(packed[:, :, :valid].reshape(expected))
        return restored

    def _detect(self, frame):
        """返回按分数降序的检测列表 [(u, v, w, h, score), ...]。"""
        nv12, meta = letterbox_bgr2nv12(frame, 640)
        t0 = self._now()
        raw = self.model.forward(nv12)
        self.infer_ms = (self._now() - t0) * 1000.0

        if len(raw) != 6:
            raise RuntimeError(f"期望 6 路 detect 输出，实得 {len(raw)}（模型不对？）")
        arrays = [np.asarray(o.buffer) for o in raw]
        scales = [np.asarray(o.properties.scale_data,
                             dtype=np.float32).reshape(-1) for o in raw]
        # 量化快路：整型输出 + 单一 scale，可跳过逐元素反量化
        if all(np.issubdtype(a.dtype, np.integer) and s.size == 1
               for a, s in zip(arrays, scales)):
            dets = decode_yolov8_detect_quantized(
                self._restore_detect_outputs(arrays),
                [float(s[0]) for s in scales], meta,
                score_threshold=self.score, iou_threshold=self.iou)
        else:
            outs = self._restore_detect_outputs(
                [self._float_output(o) for o in raw])
            dets = decode_yolov8_detect(outs, meta, score_threshold=self.score,
                                        iou_threshold=self.iou)
        out = []
        for d in dets:
            x1, y1, x2, y2 = (float(v) for v in d.box)
            out.append(((x1 + x2) * 0.5, (y1 + y2) * 0.5,
                        x2 - x1, y2 - y1, float(d.score)))
        out.sort(key=lambda t: -t[4])
        return out

    def _in_tube(self, u, v):
        """框中心是否落在管上 —— 挡掉桌面/手上的球。"""
        roi = self.roi
        if not (roi["ul"] <= u <= roi["ur"]):
            return False
        # 管带中心线按两端 y 线性插值（与 _gather_index 的取法一致）
        t = (u - roi["ul"]) / max(1.0, float(roi["ur"] - roi["ul"]))
        yc = roi["y0"] + (roi["y1"] - roi["y0"]) * t
        return abs(v - yc) <= self.y_tol

    # ---------- 主流程 ----------

    def process(self, frame):
        dets = [d for d in self._detect(frame) if self._in_tube(d[0], d[1])]
        found = bool(dets)
        u_px = dets[0][0] if found else (self._last_u or 0.0)
        w_px = dets[0][2] if found else 0.0
        sc = dets[0][4] if found else 0.0
        second = (dets[1][4] / dets[0][4]) if len(dets) > 1 else 0.0

        # 跳变门控与观测器：与 CV 路径完全一致，保证控制侧行为不变
        lost_n = self._stats["lost_run"]
        jump = None if self._last_u is None else abs(u_px - self._last_u)
        over_jump = bool(jump is not None and found and
                         jump > self.max_jump_px * (1 + lost_n))
        ambiguous = bool(second > 0.7 or over_jump)

        u_est, v_px_s, obs_valid = self._observe(u_px, found, over_jump)
        x_cm = float(np.polyval(self.poly, u_px))
        x_est_cm = float(np.polyval(self.poly, u_est))
        dxdu = (float(np.polyval(np.polyder(self.poly), u_est))
                if len(self.poly) > 1 else 0.0)
        v_cm_s = v_px_s * dxdu

        # 复用 CV 的 result 契约：把 YOLO 的量填进同名字段，控制脚本无需区分
        #   contrast/grad → 置信度×100    thr_c/thr_e → 阈值×100    width_px → 框宽
        r = self._bookkeep(found, x_cm, u_px, sc * 100.0, sc * 100.0, w_px,
                           "yolo", "yolo", ambiguous, over_jump, second,
                           None, jump, found,
                           self.score * 100.0, self.score * 100.0,
                           x_est_cm, v_cm_s, obs_valid)
        r["infer_ms"] = round(self.infer_ms, 1)
        r["n_det"] = len(dets)
        self._collect_cal(u_px, found, ambiguous)
        self._log_csv(r)
        return self._annotate(frame, r), r

    def on_key(self, key):
        """在 BallGroove1D 的标定命令之外，加 YOLO 自己的两个阈值。"""
        cmd, _, arg = key.partition(':')
        if cmd == 'score':
            self.score = float(arg)
            print(f"[ball_yolo1d] score={self.score}")
        elif cmd == 'ytol':
            self.y_tol = float(arg)
            print(f"[ball_yolo1d] y_tol={self.y_tol}")
        else:
            super().on_key(key)
