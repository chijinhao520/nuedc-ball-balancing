"""H 题槽内钢球一维定位 —— 双通道（灰度 + 梯度）ROI 列剖面 + 亚像素质心。

摄像头随摆杆固定（题面说明第 6 条允许），于是倾角不进成像、车身振动对
相机与摆杆是共模、背景恒为同一条槽 —— 三维目标检测退化成一维找峰。
与 ball_hybrid / ball_yolo_bpu 那套「白板上数多颗钢珠」完全不同场景：
这里是单球、恒定背景、只求一个 x_cm。

链路：取 ROI 带 → 两路列剖面 → 各扣空槽背景 → 平滑 → 找峰 → 融合 →
      阈值质心取亚像素峰位 u → x_cm = poly(u)

【双通道，梯度为主】镜面钢球反射什么就是什么亮度，可能比白槽亮也可能暗。
  · 通道 B 纵向梯度能量 e(u) = **主通道**：白管内壁是漫反射均匀面、沿纵向
    平滑，而球必然有强明暗过渡（球边缘 + 高光边界）——梯度与球的亮暗无关，
    且对遮挡、管端跳变、沿轴照明梯度天然免疫（ROI 仅约半个球径高，大面积
    均匀遮挡把整条带填满后纵向差分≈0）。
  · 通道 A 灰度 g(u) = 交叉验证 + 精度微调：干净帧精度略优，但易被干扰骗。
  优先级由自家白管实测定（datasets/groove_calib，相机固定、背景相减取真值）：
  管左段被黑色异物遮挡时灰度偏 −16.42cm 直接失效、梯度仍 +0.30cm；干净帧
  两路分别 −0.16 / +0.24cm。故两路一致时取灰度，否则信梯度。
  「用与亮度无关的量」这一思路借鉴第三方开源实现（steel_ball_detector.py 用
  「低饱和 + 非绿」的色度判据）；但那依赖**绿色** PPR 管，本队用**白管**
  （白管自身低饱和，与钢球无法用饱和度区分），故改用梯度能量作等价替代。

【观测器】alpha-beta 在像素域递推，输出平滑位置 x_est 与速度 v、运动方向 dir，
  丢帧时可短时预测（predict_ms 内）。串口把 x_est 与 v 一起发给 MSPM0，
  由 MCU 外推到自己的控制频率（视觉约 50Hz vs 控制 200Hz）。

ROI 高度的硬约束（题面第 7 条 + 图 3）：凹槽内宽 1.3cm，刻度线强制贴在
凹槽外的剖面边沿、间距 0.1cm。720p 下 45.7 px/cm → 槽宽 59px，球径 45.7px，
刻度线是每 4.6px 一条的高频条纹。ROI 取 50px：主要理由是 SNR（越界会被不含
球的壁面像素稀释 1.78 倍），刻度线是被 g₀ 差分+平滑+宽质心窗压住的次要风险。

标定命令走网页 /key/<k>，见 on_key。配置落 config/ball_groove_1d.json。
"""
import json
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from .base import Processor

BASE_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = BASE_DIR / 'config' / 'ball_groove_1d.json'
LOG_DIR = BASE_DIR / 'logs'


class BallGroove1D(Processor):
    name = "ball_groove"

    def __init__(self):
        # ROI 带：左右端各自的槽中心 y，支持相机绕光轴装歪（1° 在 1143px 跨度上
        # 就漂 20px，占 59px 槽宽的三分之一，固定矩形 ROI 必然一端出界）。
        # y1 == y0 时自动退化为矩形。默认值按 720p 居中，须现场标定覆盖。
        self.roi = {"y0": 360.0, "y1": 360.0, "h": 50, "ul": 60, "ur": 1220}
        self.ball_px = 45.7          # 球径成像宽度，720p/45.7px每cm 下 φ1cm
        self.smooth_px = 15          # 平滑核宽；远小于球径，抑噪不糊峰
        # 门限 = max(绝对地板, snr_k × 本帧噪声底)。自适应部分是主导项，
        # 让门限随照明/分辨率/球径自动缩放，不必每换条件就重标一次。
        # snr_k=4 的依据：真实白管视频实测信噪比 灰度 12.6× / 梯度 22.1×
        # （1213 帧，refdata/probe_channels.py），4 有充分余量。
        self.snr_k = 4.0
        self.min_contrast = 6.0      # 灰度通道绝对地板
        # 梯度通道绝对地板。自家白管实测（datasets/groove_calib，相机固定）：
        # 有球峰值 24~26、无球帧最大仅 2.9 → 判别比 29.7×。地板取 6.0
        # （无球上限的 2 倍），自适应项在噪声升高时接管。
        self.min_grad = 6.0
        # 真实白管视频 1213 帧中 98.2% 是暗峰 → 默认定死 dark，不再 auto。
        # 钢球在白管上就是暗轮廓；auto 仍可用于加 LED 后出现强高光的复测。
        self.polarity = "dark"       # dark | bright | auto
        # 单帧最大位移：a=(5/7)g·sin6°≈0.73m/s²、v_max≈0.5m/s，60fps 下约 38px。
        # 超限只标 ambiguous 不拒绝——避免误伤真实快速运动（借鉴第三方实现的
        # 物理门控思想，但它是硬拒绝，我们保留输出交由下游决策）。
        self.max_jump_px = 38.0
        # ROI 两端屏蔽宽度（px）。<0 表示自动取 edge_scale×球径；0 表示不屏蔽
        # （仅当 ul/ur 已精确标成球心可达范围时才该关）。
        # edge_scale=1.5 由实测扫描定：0.6 时灰度的有球/无球判别比只有 1.0×
        # （完全分不开），1.5 升到 9.5×，2.0 到 18.7×；梯度从 1.0 起就稳定在
        # 29.7× 不依赖它。取 1.5 兼顾两通道与球心可达范围（±12cm / 管 25cm）。
        self.edge_px = -1
        self.edge_scale = 1.5
        # 采了空槽背景后管端的**静态**跳变已被 g₀/e₀ 完全减掉，edge 只需挡住球心
        # 物理不可达的极窄区（球心可达 ±12cm、管半长 12.5cm → 余量仅 0.5cm）。
        # 实测：球可贴到距 ul 仅 7px 处，用 1.5×球径 会把靠边的球一起屏蔽掉。
        self.edge_scale_bg = 0.3
        self.poly = [0.0217, -13.6]  # x_cm = polyval(poly, u)，标定后覆盖
        self.draw_overlay = True

        self._g0 = None              # 空槽背景：灰度列均值
        self._e0 = None              # 空槽背景：梯度能量列均值
        self._bg_left = 0
        self._bg_n = 30
        self._bg_acc = None
        self._idx = None             # ROI gather 索引缓存
        self._idx_key = None
        self._cal_points = []        # [(u_px, x_cm)] 标定点
        # 多帧标定采集态 [x_cm, n_target, samples]。单帧 cal 的噪声直接进 poly，
        # 而赛题要求球停准 ±5cm，标定点自身误差必须压到亚毫米 → 取中位数。
        self._cal_pending = None
        self._last = {"found": False}
        self._last_u = None
        self._lost_since = None
        # alpha-beta 观测器（借鉴第三方方案 §5.4）。在**像素域**递推，再由 poly
        # 转厘米 —— 这样 poly 未标定时结构仍正确，标定后自动变准。
        # 相比裸差分 v=Δu/Δt：噪声不被小 dt 放大、丢帧时可短时预测、延迟小于
        # 滑窗平均。beta = α²/(2−α) 取临界阻尼。
        #
        # α=0.7 由闭环仿真定（tools/sim_vision_control.py，32 次蒙特卡洛，
        # 实测锚点：QD4310 60ms 阶跃、a=(5/7)g·sinθ、量测 σ=0.53mm）：
        #   α    0.20   0.30   0.40   0.55   0.70   0.90
        #   RMS  2.82   2.26   1.96   1.73   1.62   1.58  mm
        # 原值 0.4 的论证（「球慢，无需高增益」）只看了估计侧，漏了闭环侧：
        # 滤波引入的**相位滞后**对控制的伤害大于噪声，而球本身是二阶积分、
        # 天然低通，不需要观测器再重滤一次。0.7 之后收益递减（0.9 仅再降 2%）
        # 且抗噪裕度变薄，故取 0.7。
        # 注：最优 α 随量测噪声上升而下降。若现场实测 σ 明显大于 0.53mm
        # （用 /key/csv: 记录后看 found 帧的帧间抖动），用 /key/alpha: 回调。
        # 第三方的自适应 α（slow 0.20 → fast 0.95）已仿真验证**不要照搬**：
        # 原参 RMS 3.56mm 反而更差（它在小残差时重滤波，方向相反）；按本闭环
        # 重调到 0.70→0.95 后是 1.61mm，与固定 0.7 的 1.62mm 无差别 ——
        # 多一套自适应逻辑换不到东西。
        self.alpha = 0.7
        self.beta = 0.7 ** 2 / (2 - 0.7)
        self.still_cm_s = 1.0        # |v| 低于此判静止（第三方判稳用 1~2.5cm/s）
        self.predict_ms = 150.0      # 失锁后纯预测的最长时长，超时判观测失效
        self._obs_u = None           # 位置估计（px）
        self._obs_v = 0.0            # 速度估计（px/s）
        self._obs_t = None
        self._now = time.time        # 可替换的时间源（离线回放/测试用）
        # 统计（借鉴第三方实现的报告指标）：最长连续丢失、槽内垂直位置稳定性、
        # 帧间跳变分布。垂直位置系统性偏移比对比度更早暴露支架松动。
        self._stats = {"frames": 0, "found": 0, "lost_run": 0, "max_lost_run": 0}
        self._y_hist = deque(maxlen=600)     # 10s @60fps
        self._jump_hist = deque(maxlen=600)
        self._agg = {}
        self._csv = None
        self._csv_rows = 0
        self.load()

    # ---------- ROI 提取 ----------

    def _gather_index(self, frame_h):
        """构造 ROI 的行索引矩阵（只随 ROI 参数变化，故缓存）。"""
        key = (tuple(sorted(self.roi.items())), frame_h)
        if key == self._idx_key:
            return self._idx
        ul, ur = int(self.roi["ul"]), int(self.roi["ur"])
        half = int(self.roi["h"]) // 2
        cols = np.arange(ul, ur + 1)
        t = (cols - ul) / max(1, ur - ul)
        centers = self.roi["y0"] + (self.roi["y1"] - self.roi["y0"]) * t
        rows = np.round(centers[None, :] +
                        np.arange(-half, half + 1)[:, None]).astype(np.int32)
        np.clip(rows, 0, frame_h - 1, out=rows)
        self._idx, self._idx_key = (rows, cols), key
        return self._idx

    def _profiles(self, frame):
        """返回 (灰度列均值 g, 梯度能量列均值 e, 列坐标 cols, ROI 灰度带 band)。
        只对 ROI 行范围做灰度转换 —— 全帧 cvtColor 在 720p 要 ~2ms，
        ROI 内不到 0.2ms。"""
        rows, cols = self._gather_index(frame.shape[0])
        y_min, y_max = int(rows.min()), int(rows.max())
        sub = frame[y_min:y_max + 1, cols[0]:cols[-1] + 1]
        gray = cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY).astype(np.float32)
        band = np.take_along_axis(gray, rows - y_min, axis=0)
        g = band.mean(axis=0)
        # 纵向相邻行差分的绝对值均值。管壁的固有渐变对每列相同，由 e₀ 扣除。
        e = np.abs(np.diff(band, axis=0)).mean(axis=0)
        return g, e, cols, band

    # ---------- 峰位求解 ----------

    def _smooth(self, d):
        k = max(3, int(self.smooth_px) | 1)
        return cv2.GaussianBlur(d.reshape(1, -1), (k, 1), 0).ravel()

    def _centroid(self, curve, peak, strength):
        """阈值质心。球径成像 45.7px 是宽峰，相邻三点抛物线在 1px 间距下受
        噪声支配，故用 (值 − 0.3 峰值) 为权重在 ±0.7 球径窗口内求质心。"""
        win = max(3, int(self.ball_px * 0.7))
        lo, hi = max(0, peak - win), min(len(curve), peak + win + 1)
        seg = curve[lo:hi]
        w = np.clip(seg - 0.3 * strength, 0.0, None)
        if w.sum() <= 0:
            return float(peak)
        return float(lo + (w * np.arange(len(seg))).sum() / w.sum())

    def _peak(self, d, mode):
        """在已扣背景的曲线上找峰。mode: auto|dark|bright（梯度通道固定 bright）。
        返回 (亚像素峰位, 峰值, 阈上宽度, 极性, 次峰比)。"""
        d = self._smooth(d)
        # 端部屏蔽：球心物理上到不了管端（球会被端面挡住），而管端「白管→支架」
        # 那种沿管轴的亮度跳变会在灰度上造成巨大假暗峰。实测一次：假峰 132
        # @管端 盖过了球的真峰，只有纵向梯度通道没被骗（它对沿轴跳变免疫）。
        # 若 ul/ur 已按标定精确设成球心可达范围，可置 edge_px=0 关掉。
        scale = (self.edge_scale_bg if self._g0 is not None else self.edge_scale)
        edge = int(self.edge_px if self.edge_px >= 0 else scale * self.ball_px)
        edge = min(edge, max(0, (len(d) - 3) // 2))
        view = d[edge:len(d) - edge] if edge > 0 else d

        dark, bright = -float(view.min()), float(view.max())
        if mode == "dark":
            polarity, strength = "dark", dark
        elif mode == "bright":
            polarity, strength = "bright", bright
        else:
            polarity, strength = ("dark", dark) if dark >= bright else ("bright", bright)
        curve = -d if polarity == "dark" else d

        peak = edge + int(np.argmax(curve[edge:len(curve) - edge])) if edge > 0 \
            else int(np.argmax(curve))
        sub_u = self._centroid(curve, peak, strength)
        # 阈上宽度只在有效区内统计，否则端部残留会把宽度撑大
        width = float((curve[edge:len(curve) - edge] >= 0.5 * strength).sum()
                      if edge > 0 else (curve >= 0.5 * strength).sum())
        # 次峰与噪声底都取自「有效区内、距主峰 1.5 个球径以外」：次峰用于标记
        # 干扰/双峰歧义，噪声底用于自适应门限。
        idx = np.arange(len(curve))
        mask = np.abs(idx - peak) > 1.5 * self.ball_px
        if edge > 0:
            mask &= (idx >= edge) & (idx < len(curve) - edge)
        if mask.any():
            second = float(curve[mask].max())
            noise = float(np.percentile(np.abs(d[mask]), 90))
        else:
            second, noise = 0.0, 0.0
        return (sub_u, strength, width, polarity,
                second / strength if strength > 1e-6 else 0.0, noise)

    def _observe(self, u_meas, found, over_jump):
        """alpha-beta 观测器。返回 (u_est, v_px_s, obs_valid)。

        用真实时间戳而非假设固定帧间隔 —— 自动曝光下帧率会波动，固定 dt 会
        让速度估计系统性偏大或偏小。跳变超物理极限的帧只预测不修正，避免
        单帧误检把速度带飞。"""
        now = self._now()
        if self._obs_t is None:
            if found:
                self._obs_u, self._obs_v, self._obs_t = u_meas, 0.0, now
            return (u_meas if found else 0.0), 0.0, found
        dt = now - self._obs_t
        self._obs_t = now
        if dt <= 1e-4 or dt > 1.0:          # 首帧/长时间停顿：重置而非硬算
            if found:
                self._obs_u, self._obs_v = u_meas, 0.0
            return (self._obs_u or 0.0), 0.0, found

        u_pred = self._obs_u + self._obs_v * dt
        if found and not over_jump:
            resid = u_meas - u_pred
            self._obs_u = u_pred + self.alpha * resid
            self._obs_v = self._obs_v + (self.beta / dt) * resid
            valid = True
        else:
            # 纯预测：位置外推，速度保持。超过 predict_ms 判观测失效。
            self._obs_u = u_pred
            valid = (self._lost_since is None or
                     (now - self._lost_since) * 1000.0 <= self.predict_ms)
            if not valid:
                self._obs_v = 0.0
        return self._obs_u, self._obs_v, valid

    def _vertical_center(self, band, peak_idx):
        """峰位附近列的球心纵向位置（归一化到 ROI 高）。球的上下两条边缘在
        纵向梯度上形成双峰，其质心即球心。用途不是定位，而是自检：该值系统性
        偏离 0.5 说明相机与摆杆的相对位置变了（支架松动），比对比度更早报警。"""
        half = max(2, int(self.ball_px * 0.5))
        lo, hi = max(0, peak_idx - half), min(band.shape[1], peak_idx + half + 1)
        gy = np.abs(np.diff(band[:, lo:hi], axis=0)).mean(axis=1)
        w = np.clip(gy - float(gy.min()), 0.0, None)
        if w.sum() <= 0:
            return 0.5
        # +0.5：差分值位于两行之间
        return float(((w * np.arange(len(gy))).sum() / w.sum() + 0.5) /
                     max(1, band.shape[0] - 1))

    # ---------- 主流程 ----------

    def process(self, frame):
        g, e, cols, band = self._profiles(frame)

        if self._bg_left > 0:
            acc = (g.copy(), e.copy()) if self._bg_acc is None else \
                (self._bg_acc[0] + g, self._bg_acc[1] + e)
            self._bg_acc = acc
            self._bg_left -= 1
            if self._bg_left == 0:
                self._g0 = acc[0] / float(self._bg_n)
                self._e0 = acc[1] / float(self._bg_n)
                self._bg_acc = None
                print(f"[ball_groove] 空槽背景已采集（{self._bg_n} 帧平均，双通道）")
            r = {"found": False, "state": "bg_collect", "bg_left": self._bg_left}
            return self._annotate(frame, r), r

        # 通道 A：灰度。无背景时用中位数兜底。
        dg = g - (self._g0 if self._g0 is not None else float(np.median(g)))
        u_g, c_g, w_g, pol, second, n_g = self._peak(dg, self.polarity)
        # 通道 B：梯度能量。球必有强明暗过渡，与亮暗无关，故只找正峰。
        de = e - (self._e0 if self._e0 is not None else float(np.median(e)))
        u_e, c_e, w_e, _, _, n_e = self._peak(de, "bright")

        # 自适应门限：绝对地板与「snr_k × 本帧噪声底」取大者
        thr_g = max(self.min_contrast, self.snr_k * n_g)
        thr_e = max(self.min_grad, self.snr_k * n_e)
        gray_ok = c_g >= thr_g
        grad_ok = c_e >= thr_e
        agree = abs(u_g - u_e) <= 0.5 * self.ball_px

        # 通道优先级：**梯度为主，灰度为辅**。真实白管实测（tools/compare_illum.py，
        # 以「无球帧 − 有球帧」的差分剖面质心为真值）：管左段被黑色异物遮挡时，
        # 灰度偏 −16.42cm 直接失效，梯度仍 +0.30cm；干净帧两路都够用
        # （−0.16 / +0.24cm，要求 ≤1cm）。机理：ROI 仅约半个球径高，大面积均匀
        # 遮挡把整条带填满后纵向差分≈0，而球必有曲面明暗过渡 —— 梯度天然免疫
        # 遮挡、管端跳变与沿轴照明梯度。两路一致时取灰度（干净帧精度略高）。
        degraded = False
        if grad_ok and gray_ok and agree:
            sub_u, source = u_g, "grad+gray"
        elif grad_ok:
            sub_u, source = u_e, "grad"
        elif gray_ok:
            # 仅灰度可用属降级路径：实测它单独工作时会被异物骗，故一并标疑
            sub_u, source, degraded = u_g, "gray", True
        else:
            sub_u, source = u_e, "none"
        u_px = float(cols[0]) + sub_u

        # 阈上宽度偏离球径太多 = 整条槽被阴影盖住或强光斑，非真球。两通道量纲
        # 不同：灰度是球的暗斑宽（实测约 1.3×球径），梯度只含球的上下边缘
        # （实测约 0.75×球径），故分别设范围。
        if source in ("grad", "none"):
            plausible = 0.25 * self.ball_px <= w_e <= 1.8 * self.ball_px
        else:
            plausible = 0.4 * self.ball_px <= w_g <= 2.5 * self.ball_px
        found = bool((gray_ok or grad_ok) and plausible)

        # 物理速度门控：跳变超单帧极限即可疑。失锁越久允许跳得越远。
        lost_n = self._stats["lost_run"]
        jump = None if self._last_u is None else abs(u_px - self._last_u)
        over_jump = bool(jump is not None and found and
                         jump > self.max_jump_px * (1 + lost_n))
        ambiguous = bool(second > 0.7 or over_jump or degraded or
                         (gray_ok and grad_ok and not agree))

        y_norm = self._vertical_center(band, int(round(sub_u))) if found else None
        u_est, v_px_s, obs_valid = self._observe(u_px, found, over_jump)
        x_cm = float(np.polyval(self.poly, u_px))
        x_est_cm = float(np.polyval(self.poly, u_est))
        # 速度换算走 poly 的导数，二次映射下各处标度不同，不能只乘常数
        dxdu = (float(np.polyval(np.polyder(self.poly), u_est))
                if len(self.poly) > 1 else 0.0)
        v_cm_s = v_px_s * dxdu
        r = self._bookkeep(found, x_cm, u_px, c_g, c_e, w_g, pol, source,
                           ambiguous, over_jump, second, y_norm, jump, gray_ok,
                           thr_g, thr_e, x_est_cm, v_cm_s, obs_valid)
        self._collect_cal(u_px, found, ambiguous)
        self._log_csv(r)
        return self._annotate(frame, r), r

    def _collect_cal(self, u_px, found, ambiguous):
        """多帧标定采样：只收 found 且非 ambiguous 的帧，满额后取中位数落点。

        同时报告样本离散度 —— 它就是该点的标定不确定度，若 std 偏大说明球
        没停稳或该位置检测不可靠，应当重采而不是硬拟合。
        """
        if self._cal_pending is None:
            return
        if not found or ambiguous:
            return
        self._cal_pending[2].append(float(u_px))
        got, need = len(self._cal_pending[2]), self._cal_pending[1]
        if got < need:
            return
        us = np.array(self._cal_pending[2])
        u_med = float(np.median(us))
        x_cm = self._cal_pending[0]
        self._cal_points.append((u_med, x_cm))
        self._cal_pending = None
        print(f"[ball_groove] 标定点 {len(self._cal_points)}: x={x_cm:+.2f}cm "
              f"u={u_med:.2f}px  (n={got} std={us.std():.2f}px "
              f"极差={us.max() - us.min():.2f}px)")

    def _bookkeep(self, found, x_cm, u_px, c_g, c_e, w_g, pol, source,
                  ambiguous, over_jump, second, y_norm, jump, gray_ok,
                  thr_g=0.0, thr_e=0.0, x_est_cm=0.0, v_cm_s=0.0,
                  obs_valid=False):
        st = self._stats
        st["frames"] += 1
        now = self._now()
        if found:
            st["found"] += 1
            st["lost_run"] = 0
            self._lost_since = None
            self._last_u = u_px
            if y_norm is not None:
                self._y_hist.append(y_norm)
            if jump is not None:
                self._jump_hist.append(jump)
        else:
            st["lost_run"] += 1
            st["max_lost_run"] = max(st["max_lost_run"], st["lost_run"])
            if self._lost_since is None:
                self._lost_since = now
        # percentile 每 30 帧算一次，不挂在每帧路径上
        if st["frames"] % 30 == 0 and self._y_hist:
            self._agg = {
                "y_std": round(float(np.std(self._y_hist)), 4),
                "y_med": round(float(np.median(self._y_hist)), 3),
                "jump_p95": (round(float(np.percentile(self._jump_hist, 95)), 2)
                             if self._jump_hist else 0.0),
                "jump_max": (round(float(np.max(self._jump_hist)), 2)
                             if self._jump_hist else 0.0),
            }
        state = "ok" if found else ("low_contrast" if not gray_ok else "bad_width")
        r = {
            "found": found, "x_cm": round(x_cm, 3), "u_px": round(u_px, 2),
            # 观测器输出：x_est 平滑位置、v 速度、dir 运动方向。obs 为 False 时
            # 这三项不可用（失锁超过 predict_ms）。⚠️ poly 未标定时 x/v 的**厘米
            # 值无意义**，只有 u_px 是可信的原始测量。
            "x_est_cm": round(x_est_cm, 3), "v_cm_s": round(v_cm_s, 2),
            "obs": obs_valid,
            "dir": ("?" if not obs_valid else
                    ("0" if abs(v_cm_s) < self.still_cm_s
                     else ("+" if v_cm_s > 0 else "-"))),
            "contrast": round(c_g, 1), "grad": round(c_e, 1),
            "thr_c": round(thr_g, 1), "thr_e": round(thr_e, 1),
            "width_px": round(w_g, 1), "polarity": pol, "source": source,
            "ambiguous": ambiguous, "over_jump": over_jump,
            "second_ratio": round(second, 2),
            "y_norm": None if y_norm is None else round(y_norm, 3),
            "bg": self._g0 is not None,
            "lost_run": st["lost_run"], "max_lost_run": st["max_lost_run"],
            "found_rate": round(100.0 * st["found"] / max(1, st["frames"]), 1),
            "lost_ms": 0 if self._lost_since is None
                       else int((self._now() - self._lost_since) * 1000),
            "state": state,
        }
        r.update(self._agg)
        self._last = r
        return r

    # ---------- CSV 逐帧记录 ----------

    def _log_csv(self, r):
        """调试设施，默认关。字段沿用第三方实现的设计便于对照。
        服务于外环 PD 整定与「误差绝对值 ≤1cm」的验收取证。"""
        if self._csv is None:
            return
        self._csv.write(
            f"{self._stats['frames']},{time.time():.4f},{int(r['found'])},"
            f"{r['u_px']},{r['x_cm']},{r['contrast']},{r['grad']},"
            f"{r['width_px']},{r['y_norm'] if r['y_norm'] is not None else ''},"
            f"{r['source']},{int(r['ambiguous'])},{r['state']}\n")
        self._csv_rows += 1
        if self._csv_rows % 60 == 0:
            self._csv.flush()

    def _csv_open(self, tag=""):
        self._csv_close()
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        p = LOG_DIR / (f"groove_{datetime.now().strftime('%m%d_%H%M%S')}"
                       f"{('_' + tag) if tag else ''}.csv")
        self._csv = p.open("w", encoding="utf-8")
        self._csv.write("frame,time_s,found,u_px,x_cm,contrast,grad,width_px,"
                        "y_norm,source,ambiguous,state\n")
        self._csv_rows = 0
        print(f"[ball_groove] CSV 记录开始 {p}")

    def _csv_close(self):
        if self._csv is not None:
            self._csv.close()
            print(f"[ball_groove] CSV 记录结束（{self._csv_rows} 行）")
            self._csv = None

    # ---------- 叠加绘制 ----------

    def _annotate(self, frame, r):
        """叠加物一律画在 ROI 之外。题面第 1 项 6 分要求「完整清晰看到钢球
        滚动」，把标记压在球上等于自己扣分。"""
        if not self.draw_overlay:
            return frame
        cols = self._gather_index(frame.shape[0])[1]
        half = int(self.roi["h"]) // 2
        u0, u1 = int(cols[0]), int(cols[-1])
        y0, y1 = self.roi["y0"], self.roi["y1"]
        for sign in (-1, 1):
            cv2.line(frame, (u0, int(y0 + sign * half)), (u1, int(y1 + sign * half)),
                     (0, 200, 255), 1)
        H, W = frame.shape[:2]
        uncal = not self._cal_points          # poly 未标定 → 厘米值无意义
        if r.get("found"):
            u = int(round(r["u_px"]))
            t = (u - u0) / max(1, u1 - u0)
            yc = int(y0 + (y1 - y0) * t)
            # 指向球的三角，尖端停在 ROI 带外 4px —— 不压住球与槽
            tip = yc - half - 4
            cv2.drawMarker(frame, (u, tip - 10), (0, 255, 0),
                           cv2.MARKER_TRIANGLE_DOWN, 22, 3)
            cv2.line(frame, (u, tip - 2), (u, tip), (0, 255, 0), 2)
            lab = (f"u={r['u_px']:.1f}px" if uncal
                   else f"x={r['x_cm']:+.2f}cm")
            cv2.putText(frame, lab, (max(4, u - 70), tip - 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 底部位置尺：球在 ROI 内的相对位置，不遮挡管子。速度用箭头长度表示。
        bx0, bx1, by = int(W * 0.08), int(W * 0.92), H - 34
        cv2.line(frame, (bx0, by), (bx1, by), (120, 120, 120), 2)
        for i in range(5):
            x = bx0 + (bx1 - bx0) * i // 4
            cv2.line(frame, (x, by - 7), (x, by + 7), (120, 120, 120), 1)
        cv2.line(frame, ((bx0 + bx1) // 2, by - 11),
                 ((bx0 + bx1) // 2, by + 11), (180, 180, 60), 2)   # 中心 O
        if r.get("found"):
            t = min(1.0, max(0.0, (r["u_px"] - u0) / max(1, u1 - u0)))
            px = bx0 + int((bx1 - bx0) * t)
            cv2.drawMarker(frame, (px, by), (0, 255, 0),
                           cv2.MARKER_TRIANGLE_UP, 24, 3)
            v = r.get("v_cm_s", 0.0)
            if r.get("obs") and abs(v) >= self.still_cm_s:
                L = int(min(90, 6 * abs(v)))
                x2 = px + (L if v > 0 else -L)
                cv2.arrowedLine(frame, (px, by + 20), (x2, by + 20),
                                (0, 200, 255), 2, tipLength=0.3)
                cv2.putText(frame, f"{v:+.1f}cm/s", (px - 40, by + 44),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

        col = (0, 255, 0) if r.get("found") else (0, 0, 255)
        cv2.putText(frame, f"{r.get('state', '?')}  {r.get('source', '')}"
                           f"  dir={r.get('dir', '?')}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        cv2.putText(frame, f"c={r.get('contrast', 0)}/{r.get('thr_c', 0)} "
                           f"e={r.get('grad', 0)}/{r.get('thr_e', 0)} "
                           f"{'BG' if r.get('bg') else 'no-bg'}"
                           f"{' AMBIG' if r.get('ambiguous') else ''}"
                           f"{' JUMP' if r.get('over_jump') else ''}"
                           f"{'  [UNCAL]' if uncal else ''}",
                    (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1)
        cv2.putText(frame, f"rate={r.get('found_rate', 0)}%  "
                           f"lost_max={r.get('max_lost_run', 0)}  "
                           f"y_std={r.get('y_std', '-')}",
                    (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
        return frame

    # ---------- 标定命令 ----------

    def on_key(self, key):
        """roi:y0,y1,h,ul,ur | bg[:n] | cal:<x_cm> | caln:<x_cm>[,n] | calst
        fit | clear | stats | reset
        pol:auto|dark|bright | mc:<灰度阈> | mg:<梯度阈> | jump:<px>
        alpha:<0~1> | csv:on[,tag]|off | save | load | overlay:0|1"""
        try:
            cmd, _, arg = key.partition(':')
            if cmd == 'roi':
                y0, y1, h, ul, ur = (float(v) for v in arg.split(','))
                self.roi = {"y0": y0, "y1": y1, "h": int(h),
                            "ul": int(ul), "ur": int(ur)}
                self._g0 = self._e0 = None   # ROI 变了背景曲线立即失效
                print(f"[ball_groove] roi={self.roi}（背景已失效，请重跑 bg）")
            elif cmd == 'bg':
                self._bg_n = int(arg) if arg else 30
                self._bg_left, self._bg_acc = self._bg_n, None
            elif cmd == 'cal':
                if not self._last.get("found"):
                    print("[ball_groove] 当前未锁定，标定点未记录")
                else:
                    self._cal_points.append((self._last["u_px"], float(arg)))
                    print(f"[ball_groove] 标定点 {len(self._cal_points)}: "
                          f"u={self._last['u_px']:.2f} → x={float(arg):+.2f}cm")
            elif cmd == 'caln':
                p = arg.split(',')
                n = int(p[1]) if len(p) > 1 else 15
                self._cal_pending = [float(p[0]), max(1, n), []]
                print(f"[ball_groove] 采集标定点 x={float(p[0]):+.2f}cm，"
                      f"需 {n} 帧有效样本…")
            elif cmd == 'calst':
                if self._cal_pending is None:
                    print(f"[ball_groove] 无采集任务；已有 "
                          f"{len(self._cal_points)} 个标定点")
                else:
                    print(f"[ball_groove] 采集中 x={self._cal_pending[0]:+.2f}"
                          f"cm: {len(self._cal_pending[2])}/"
                          f"{self._cal_pending[1]} 帧")
            elif cmd == 'fit':
                self._fit()
            elif cmd == 'clear':
                self._cal_points.clear()
            elif cmd == 'stats':
                self._print_stats()
            elif cmd == 'reset':
                self._stats = {"frames": 0, "found": 0, "lost_run": 0,
                               "max_lost_run": 0}
                self._y_hist.clear()
                self._jump_hist.clear()
                self._agg = {}
                self._last_u = None
                print("[ball_groove] 统计已复位")
            elif cmd == 'pol':
                self.polarity = arg
            elif cmd == 'mc':
                self.min_contrast = float(arg)
            elif cmd == 'mg':
                self.min_grad = float(arg)
            elif cmd == 'jump':
                self.max_jump_px = float(arg)
            elif cmd == 'ball':
                self.ball_px = float(arg)
                self.smooth_px = max(3, int(self.ball_px / 3) | 1)
                print(f"[ball_groove] ball_px={self.ball_px} "
                      f"smooth_px={self.smooth_px}")
            elif cmd == 'edge':
                self.edge_px = float(arg)
            elif cmd == 'alpha':
                # beta 跟着走临界阻尼，不单独暴露 —— 两者独立可调易调出振荡
                a = min(0.98, max(0.05, float(arg)))
                self.alpha, self.beta = a, a * a / (2 - a)
                print(f"[ball_groove] alpha={a:.2f} beta={self.beta:.3f}")
            elif cmd == 'csv':
                sub, _, tag = arg.partition(',')
                self._csv_open(tag) if sub == 'on' else self._csv_close()
            elif cmd == 'overlay':
                self.draw_overlay = arg == '1'
            elif cmd == 'save':
                self.save()
            elif cmd == 'load':
                self.load()
        except Exception as e:
            print(f"[ball_groove] 命令 '{key}' 失败: {e}")

    def _print_stats(self):
        st = self._stats
        print(f"[ball_groove] frames={st['frames']} found={st['found']} "
              f"rate={100.0 * st['found'] / max(1, st['frames']):.2f}% "
              f"longest_lost_run={st['max_lost_run']}")
        if self._y_hist:
            print(f"  槽内垂直位置 median/std="
                  f"{np.median(self._y_hist):.4f}/{np.std(self._y_hist):.4f}"
                  f"（std 大或 median 偏离 0.5 → 查支架/ROI）")
        if self._jump_hist:
            print(f"  帧间跳变 p95/max="
                  f"{np.percentile(self._jump_hist, 95):.2f}/"
                  f"{np.max(self._jump_hist):.2f}px（门限 {self.max_jump_px}）")

    def _fit(self):
        """标定点拟合 x=f(u)。110° 广角边缘有桶形畸变，点足够时比较一次/
        二次残差再定阶，不盲目上高阶。"""
        if len(self._cal_points) < 2:
            print("[ball_groove] 标定点不足（至少 2 点）")
            return
        u = np.array([p[0] for p in self._cal_points])
        x = np.array([p[1] for p in self._cal_points])
        best = None
        for deg in (1, 2):
            if len(u) < deg + 1:
                continue
            c = np.polyfit(u, x, deg)
            rms = float(np.sqrt(np.mean((np.polyval(c, u) - x) ** 2)))
            print(f"[ball_groove] deg={deg} rms={rms:.4f}cm")
            # 二次仅在把残差压掉 30% 以上时才采用，避免少点数过拟合
            if best is None or rms < best[1] * 0.7:
                best = (c, rms)
        self.poly = [float(v) for v in best[0]]
        print(f"[ball_groove] poly={self.poly} rms={best[1]:.4f}cm（{len(u)} 点）")

    # ---------- 配置持久化 ----------

    def save(self):
        CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
        data = {"roi": self.roi, "poly": self.poly, "polarity": self.polarity,
                "min_contrast": self.min_contrast, "min_grad": self.min_grad,
                "snr_k": self.snr_k, "edge_px": self.edge_px,
                "edge_scale_bg": self.edge_scale_bg,
                "edge_scale": self.edge_scale,
                "max_jump_px": self.max_jump_px, "ball_px": self.ball_px,
                "smooth_px": self.smooth_px, "alpha": self.alpha,
                "cal_points": self._cal_points,
                "g0": None if self._g0 is None else
                      [round(float(v), 2) for v in self._g0],
                "e0": None if self._e0 is None else
                      [round(float(v), 3) for v in self._e0]}
        CONFIG_PATH.write_text(json.dumps(data, indent=2), encoding='utf-8')
        print(f"[ball_groove] 已保存 {CONFIG_PATH}")

    def load(self):
        if not CONFIG_PATH.exists():
            return
        try:
            d = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
            for k in ("roi", "poly", "polarity", "min_contrast", "min_grad",
                      "snr_k", "edge_px", "edge_scale", "edge_scale_bg", "max_jump_px",
                      "ball_px", "smooth_px"):
                if k in d:
                    setattr(self, k, d[k])
            if "alpha" in d:            # beta 是派生量，必须跟着重算
                self.alpha = float(d["alpha"])
                self.beta = self.alpha ** 2 / (2 - self.alpha)
            self._cal_points = [tuple(p) for p in d.get("cal_points", [])]
            g0, e0 = d.get("g0"), d.get("e0")
            self._g0 = None if g0 is None else np.array(g0, dtype=np.float32)
            self._e0 = None if e0 is None else np.array(e0, dtype=np.float32)
            self._idx_key = None
            print(f"[ball_groove] 已载入 {CONFIG_PATH}"
                  f"{'（含空槽背景）' if self._g0 is not None else ''}")
        except Exception as e:
            print(f"[ball_groove] 载入配置失败: {e}")
