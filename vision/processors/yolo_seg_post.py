"""YOLOv8 实例分割的纯 NumPy/OpenCV 后处理。

本模块不依赖 hobot_dnn，PC 端 ONNX 回归与 X5 BPU 处理器共用同一套
letterbox、NMS、掩膜恢复和坐标映射，避免两端算法悄悄分叉。
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class LetterboxMeta:
    original_height: int
    original_width: int
    input_height: int
    input_width: int
    resized_height: int
    resized_width: int
    ratio: float
    pad_top: int
    pad_left: int


@dataclass
class SegDetection:
    score: float
    box: np.ndarray
    mask: np.ndarray | None


def letterbox_frame(frame: np.ndarray, size: int = 640) -> tuple[np.ndarray, LetterboxMeta]:
    """按 Ultralytics 的居中 letterbox 规则缩放到正方形。"""
    height, width = frame.shape[:2]
    ratio = min(size / width, size / height)
    resized_width = int(round(width * ratio))
    resized_height = int(round(height * ratio))
    if (resized_width, resized_height) != (width, height):
        resized = cv2.resize(frame, (resized_width, resized_height),
                             interpolation=cv2.INTER_LINEAR)
    else:
        resized = frame

    pad_width = size - resized_width
    pad_height = size - resized_height
    left = int(round(pad_width / 2 - 0.1))
    right = int(round(pad_width / 2 + 0.1))
    top = int(round(pad_height / 2 - 0.1))
    bottom = int(round(pad_height / 2 + 0.1))
    padded = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
    )
    meta = LetterboxMeta(
        original_height=height,
        original_width=width,
        input_height=size,
        input_width=size,
        resized_height=resized_height,
        resized_width=resized_width,
        ratio=ratio,
        pad_top=top,
        pad_left=left,
    )
    return padded, meta


def letterbox_bgr2nv12(
    frame: np.ndarray, size: int = 640,
) -> tuple[np.ndarray, LetterboxMeta]:
    """融合 letterbox 与 BGR->NV12，避免转换灰色 padding 区域。"""
    height, width = frame.shape[:2]
    ratio = min(size / width, size / height)
    resized_width = int(round(width * ratio))
    resized_height = int(round(height * ratio))
    pad_width = size - resized_width
    pad_height = size - resized_height
    left = int(round(pad_width / 2 - 0.1))
    top = int(round(pad_height / 2 - 0.1))
    meta = LetterboxMeta(
        original_height=height,
        original_width=width,
        input_height=size,
        input_width=size,
        resized_height=resized_height,
        resized_width=resized_width,
        ratio=ratio,
        pad_top=top,
        pad_left=left,
    )

    # NV12 的色度平面以 2x2 为单位。奇数边界退回通用路径，保持语义正确。
    if any(value % 2 for value in (
        size, resized_width, resized_height, left, top,
    )):
        padded, _ = letterbox_frame(frame, size)
        area = size * size
        i420 = cv2.cvtColor(padded, cv2.COLOR_BGR2YUV_I420).reshape(-1)
        nv12 = np.empty_like(i420)
        nv12[:area] = i420[:area]
        chroma = i420[area:].reshape(2, area // 4)
        nv12[area::2] = chroma[0]
        nv12[area + 1::2] = chroma[1]
        return nv12, meta

    if (resized_width, resized_height) == (width, height):
        resized = frame
    else:
        resized = cv2.resize(
            frame, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR,
        )
    source_area = resized_width * resized_height
    i420 = cv2.cvtColor(resized, cv2.COLOR_BGR2YUV_I420).reshape(-1)
    source_y = i420[:source_area].reshape(resized_height, resized_width)
    source_chroma = i420[source_area:].reshape(2, resized_height // 2, resized_width // 2)

    target_area = size * size
    nv12 = np.empty(target_area * 3 // 2, dtype=np.uint8)
    target_y = nv12[:target_area].reshape(size, size)
    target_uv = nv12[target_area:].reshape(size // 2, size)
    target_y.fill(114)
    target_uv.fill(128)
    target_y[top:top + resized_height, left:left + resized_width] = source_y
    uv_region = target_uv[
        top // 2:top // 2 + resized_height // 2,
        left:left + resized_width,
    ]
    uv_region[:, 0::2] = source_chroma[0]
    uv_region[:, 1::2] = source_chroma[1]
    return nv12, meta


def prepare_rgb_nchw(frame: np.ndarray, size: int = 640) -> tuple[np.ndarray, LetterboxMeta]:
    """生成 ONNX/量化校准使用的 RGB NCHW float32 输入。"""
    padded, meta = letterbox_frame(frame, size)
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1), dtype=np.float32) / 255.0
    return tensor[None], meta


def _prediction_matrix(output: np.ndarray, mask_dim: int) -> np.ndarray:
    array = np.asarray(output).squeeze()
    if array.ndim != 2:
        raise ValueError(f"检测输出维度应为 2，实际为 {array.shape}")
    expected_channels = 5 + mask_dim
    if array.shape[0] == expected_channels:
        return array.T.astype(np.float32, copy=False)
    if array.shape[1] == expected_channels:
        return array.astype(np.float32, copy=False)
    raise ValueError(f"检测输出找不到 {expected_channels} 个通道: {array.shape}")


def _prototype_tensor(output: np.ndarray, mask_dim: int) -> np.ndarray:
    array = np.asarray(output).squeeze()
    if array.ndim != 3:
        raise ValueError(f"掩膜原型输出维度应为 3，实际为 {array.shape}")
    if array.shape[0] == mask_dim:
        return array.astype(np.float32, copy=False)
    if array.shape[-1] == mask_dim:
        return array.transpose(2, 0, 1).astype(np.float32, copy=False)
    raise ValueError(f"掩膜原型输出找不到 {mask_dim} 个通道: {array.shape}")


def _as_chw(output: np.ndarray, channels: int) -> np.ndarray | None:
    array = np.asarray(output)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3:
        return None
    if array.shape[0] == channels:
        return array.astype(np.float32, copy=False)
    if array.shape[-1] == channels:
        return array.transpose(2, 0, 1).astype(np.float32, copy=False)
    return None


def _raw_head_outputs(outputs: list[np.ndarray] | tuple[np.ndarray, ...],
                      input_size: int, mask_dim: int,
                      score_threshold: float | None = None
                      ) -> tuple[np.ndarray, np.ndarray]:
    proto_size = input_size // 4
    prototypes = []
    heads: dict[tuple[int, int], dict[int, np.ndarray]] = {}
    for output in outputs:
        matched = False
        for channels in (64, 1, mask_dim):
            chw = _as_chw(output, channels)
            if chw is None:
                continue
            height, width = chw.shape[1:]
            if channels == mask_dim and (height, width) == (proto_size, proto_size):
                prototypes.append(chw)
            else:
                heads.setdefault((height, width), {})[channels] = chw
            matched = True
            break
        if not matched:
            raise ValueError(f"无法识别原始检测头输出: {np.asarray(output).shape}")

    if len(prototypes) != 1:
        raise ValueError(f"原始检测头应有 1 个 proto，实际为 {len(prototypes)}")
    predictions = []
    bins = np.arange(16, dtype=np.float32).reshape(1, 16, 1)
    for stride in (8, 16, 32):
        size = input_size // stride
        level = heads.get((size, size), {})
        if set(level) != {64, 1, mask_dim}:
            raise ValueError(f"{size}x{size} 原始头通道不完整: {sorted(level)}")

        class_logits = level[1].reshape(-1)
        scores = 1.0 / (1.0 + np.exp(-np.clip(class_logits, -30.0, 30.0)))
        if score_threshold is None:
            indices = np.arange(scores.size)
        else:
            indices = np.flatnonzero(scores >= score_threshold)
        if indices.size == 0:
            continue

        # 分类头比 DFL 解码便宜得多。实时路径先筛候选，避免每帧对
        # 绝大多数低分位置执行 4x16 bin softmax 和后续数组拼接。
        scores = scores[indices]
        box_logits = level[64].reshape(4, 16, -1)[:, :, indices]
        shifted = box_logits - box_logits.max(axis=1, keepdims=True)
        probabilities = np.exp(shifted)
        probabilities /= probabilities.sum(axis=1, keepdims=True)
        distances = (probabilities * bins).sum(axis=1)

        anchor_x = (indices % size).astype(np.float32) + 0.5
        anchor_y = (indices // size).astype(np.float32) + 0.5
        left, top, right, bottom = distances
        x1 = anchor_x - left
        y1 = anchor_y - top
        x2 = anchor_x + right
        y2 = anchor_y + bottom
        boxes = np.stack(
            ((x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1), axis=1
        ) * stride

        coefficients = level[mask_dim].reshape(mask_dim, -1).T[indices]
        predictions.append(np.concatenate((boxes, scores[:, None], coefficients), axis=1))

    if not predictions:
        return np.empty((0, 5 + mask_dim), dtype=np.float32), prototypes[0]
    return np.concatenate(predictions, axis=0), prototypes[0]


def _raw_detect_head_outputs(
    outputs: list[np.ndarray] | tuple[np.ndarray, ...],
    input_size: int,
    score_threshold: float | None = None,
) -> np.ndarray:
    heads: dict[tuple[int, int], dict[int, np.ndarray]] = {}
    for output in outputs:
        matched = False
        for channels in (64, 1):
            chw = _as_chw(output, channels)
            if chw is None:
                continue
            heads.setdefault(chw.shape[1:], {})[channels] = chw
            matched = True
            break
        if not matched:
            raise ValueError(f"无法识别 detection 原始头输出: {np.asarray(output).shape}")

    predictions = []
    bins = np.arange(16, dtype=np.float32).reshape(1, 16, 1)
    for stride in (8, 16, 32):
        size = input_size // stride
        level = heads.get((size, size), {})
        if set(level) != {64, 1}:
            raise ValueError(f"{size}x{size} detection 原始头通道不完整: {sorted(level)}")

        logits = level[1].reshape(-1)
        scores = 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))
        indices = (
            np.arange(scores.size)
            if score_threshold is None
            else np.flatnonzero(scores >= score_threshold)
        )
        if indices.size == 0:
            continue

        scores = scores[indices]
        box_logits = level[64].reshape(4, 16, -1)[:, :, indices]
        shifted = box_logits - box_logits.max(axis=1, keepdims=True)
        probabilities = np.exp(shifted)
        probabilities /= probabilities.sum(axis=1, keepdims=True)
        distances = (probabilities * bins).sum(axis=1)

        anchor_x = (indices % size).astype(np.float32) + 0.5
        anchor_y = (indices // size).astype(np.float32) + 0.5
        left, top, right, bottom = distances
        boxes = np.stack((
            (anchor_x - left + anchor_x + right) / 2,
            (anchor_y - top + anchor_y + bottom) / 2,
            left + right,
            top + bottom,
        ), axis=1) * stride
        predictions.append(np.concatenate((boxes, scores[:, None]), axis=1))

    if not predictions:
        return np.empty((0, 5), dtype=np.float32)
    return np.concatenate(predictions, axis=0)


def _raw_detect_head_outputs_quantized(
    outputs: list[np.ndarray] | tuple[np.ndarray, ...],
    scales: list[float] | tuple[float, ...],
    input_size: int,
    score_threshold: float,
) -> np.ndarray:
    if len(outputs) != len(scales):
        raise ValueError("量化输出与 scale 数量不一致")
    heads: dict[tuple[int, int], dict[int, tuple[np.ndarray, float]]] = {}
    for output, scale in zip(outputs, scales):
        array = np.asarray(output)
        if array.ndim == 4 and array.shape[0] == 1:
            array = array[0]
        if array.ndim != 3:
            raise ValueError(f"量化 detection 输出维度非法: {array.shape}")
        channels = array.shape[0]
        if channels not in (64, 1):
            raise ValueError(f"量化 detection 输出通道非法: {array.shape}")
        heads.setdefault(array.shape[1:], {})[channels] = (array, float(scale))

    predictions = []
    bins = np.arange(16, dtype=np.float32).reshape(1, 16, 1)
    for stride in (8, 16, 32):
        size = input_size // stride
        level = heads.get((size, size), {})
        if set(level) != {64, 1}:
            raise ValueError(f"{size}x{size} 量化 detection 头不完整: {sorted(level)}")

        class_values, class_scale = level[1]
        logits = class_values.reshape(-1).astype(np.float32) * class_scale
        scores = 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))
        indices = np.flatnonzero(scores >= score_threshold)
        if indices.size == 0:
            continue

        scores = scores[indices]
        box_values, box_scale = level[64]
        box_logits = box_values.reshape(4, 16, -1)[:, :, indices]
        box_logits = box_logits.astype(np.float32) * box_scale
        shifted = box_logits - box_logits.max(axis=1, keepdims=True)
        probabilities = np.exp(shifted)
        probabilities /= probabilities.sum(axis=1, keepdims=True)
        distances = (probabilities * bins).sum(axis=1)

        anchor_x = (indices % size).astype(np.float32) + 0.5
        anchor_y = (indices // size).astype(np.float32) + 0.5
        left, top, right, bottom = distances
        boxes = np.stack((
            anchor_x + (right - left) / 2,
            anchor_y + (bottom - top) / 2,
            left + right,
            top + bottom,
        ), axis=1) * stride
        predictions.append(np.concatenate((boxes, scores[:, None]), axis=1))

    if not predictions:
        return np.empty((0, 5), dtype=np.float32)
    return np.concatenate(predictions, axis=0)


def split_outputs(outputs: list[np.ndarray] | tuple[np.ndarray, ...],
                  mask_dim: int = 32, input_size: int = 640,
                  score_threshold: float | None = None,
                  ) -> tuple[np.ndarray, np.ndarray]:
    """兼容旧 2 输出和 X5 原始 10 输出，返回预测矩阵与 proto。"""
    if len(outputs) == 10:
        return _raw_head_outputs(outputs, input_size, mask_dim, score_threshold)
    if len(outputs) != 2:
        raise ValueError(f"YOLOv8-seg 应有 2 或 10 个输出，实际为 {len(outputs)}")
    squeezed = [np.asarray(output).squeeze() for output in outputs]
    pred_index = next((index for index, value in enumerate(squeezed) if value.ndim == 2), None)
    proto_index = next((index for index, value in enumerate(squeezed) if value.ndim == 3), None)
    if pred_index is None or proto_index is None:
        raise ValueError(f"无法区分检测与原型输出: {[value.shape for value in squeezed]}")
    return (_prediction_matrix(outputs[pred_index], mask_dim),
            _prototype_tensor(outputs[proto_index], mask_dim))


def _xywh_to_xyxy(boxes: np.ndarray) -> np.ndarray:
    result = np.empty_like(boxes, dtype=np.float32)
    result[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    result[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    result[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    result[:, 3] = boxes[:, 1] + boxes[:, 3] / 2
    return result


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float,
         max_det: int) -> list[int]:
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size and len(keep) < max_det:
        current = int(order[0])
        keep.append(current)
        if order.size == 1:
            break
        rest = order[1:]
        inter_x1 = np.maximum(x1[current], x1[rest])
        inter_y1 = np.maximum(y1[current], y1[rest])
        inter_x2 = np.minimum(x2[current], x2[rest])
        inter_y2 = np.minimum(y2[current], y2[rest])
        intersection = (np.maximum(0.0, inter_x2 - inter_x1) *
                        np.maximum(0.0, inter_y2 - inter_y1))
        union = areas[current] + areas[rest] - intersection
        iou = intersection / np.maximum(union, 1e-7)
        order = rest[iou <= iou_threshold]
    return keep


def _restore_box(box: np.ndarray, meta: LetterboxMeta) -> np.ndarray:
    restored = box.astype(np.float32, copy=True)
    restored[[0, 2]] = (restored[[0, 2]] - meta.pad_left) / meta.ratio
    restored[[1, 3]] = (restored[[1, 3]] - meta.pad_top) / meta.ratio
    restored[[0, 2]] = np.clip(restored[[0, 2]], 0, meta.original_width - 1)
    restored[[1, 3]] = np.clip(restored[[1, 3]], 0, meta.original_height - 1)
    return restored


def _restore_mask(coefficients: np.ndarray, prototypes: np.ndarray,
                  input_box: np.ndarray, meta: LetterboxMeta,
                  threshold: float) -> np.ndarray:
    channels, mask_height, mask_width = prototypes.shape
    logits = coefficients @ prototypes.reshape(channels, -1)
    mask = 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))
    mask = mask.reshape(mask_height, mask_width)

    x1 = int(np.floor(input_box[0] * mask_width / meta.input_width))
    y1 = int(np.floor(input_box[1] * mask_height / meta.input_height))
    x2 = int(np.ceil(input_box[2] * mask_width / meta.input_width))
    y2 = int(np.ceil(input_box[3] * mask_height / meta.input_height))
    crop = np.zeros_like(mask)
    x1, x2 = np.clip((x1, x2), 0, mask_width)
    y1, y2 = np.clip((y1, y2), 0, mask_height)
    if x2 > x1 and y2 > y1:
        crop[y1:y2, x1:x2] = mask[y1:y2, x1:x2]

    full = cv2.resize(crop, (meta.input_width, meta.input_height),
                      interpolation=cv2.INTER_LINEAR)
    content = full[
        meta.pad_top:meta.pad_top + meta.resized_height,
        meta.pad_left:meta.pad_left + meta.resized_width,
    ]
    restored = cv2.resize(content, (meta.original_width, meta.original_height),
                          interpolation=cv2.INTER_LINEAR)
    return restored > threshold


def decode_yolov8_seg(outputs: list[np.ndarray] | tuple[np.ndarray, ...],
                      meta: LetterboxMeta, score_threshold: float = 0.25,
                      iou_threshold: float = 0.6, mask_threshold: float = 0.5,
                      max_det: int = 100, mask_dim: int = 32,
                      restore_masks: bool = True) -> list[SegDetection]:
    predictions, prototypes = split_outputs(
        outputs, mask_dim, meta.input_width, score_threshold
    )
    if len(predictions) == 0:
        return []
    scores = predictions[:, 4]
    candidates = scores >= score_threshold
    if not np.any(candidates):
        return []

    selected = predictions[candidates]
    selected_scores = scores[candidates]
    input_boxes = _xywh_to_xyxy(selected[:, :4])
    keep = _nms(input_boxes, selected_scores, iou_threshold, max_det)

    detections = []
    for index in keep:
        input_box = input_boxes[index]
        detections.append(SegDetection(
            score=float(selected_scores[index]),
            box=_restore_box(input_box, meta),
            mask=(
                _restore_mask(selected[index, 5:5 + mask_dim], prototypes,
                              input_box, meta, mask_threshold)
                if restore_masks else None
            ),
        ))
    return detections


def decode_yolov8_detect(
    outputs: list[np.ndarray] | tuple[np.ndarray, ...],
    meta: LetterboxMeta,
    score_threshold: float = 0.25,
    iou_threshold: float = 0.4,
    max_det: int = 100,
) -> list[SegDetection]:
    if len(outputs) != 6:
        raise ValueError(f"YOLOv8-detect raw-head 应有 6 个输出，实际为 {len(outputs)}")
    predictions = _raw_detect_head_outputs(
        outputs, meta.input_width, score_threshold
    )
    if len(predictions) == 0:
        return []

    input_boxes = _xywh_to_xyxy(predictions[:, :4])
    keep = _nms(input_boxes, predictions[:, 4], iou_threshold, max_det)
    return [
        SegDetection(
            score=float(predictions[index, 4]),
            box=_restore_box(input_boxes[index], meta),
            mask=None,
        )
        for index in keep
    ]


def decode_yolov8_detect_quantized(
    outputs: list[np.ndarray] | tuple[np.ndarray, ...],
    scales: list[float] | tuple[float, ...],
    meta: LetterboxMeta,
    score_threshold: float = 0.25,
    iou_threshold: float = 0.4,
    max_det: int = 100,
) -> list[SegDetection]:
    """直接从 INT8 raw heads 解码，只反量化超过分类阈值的框。"""
    if len(outputs) != 6:
        raise ValueError(f"YOLOv8-detect 量化 raw-head 应有 6 个输出，实际为 {len(outputs)}")
    predictions = _raw_detect_head_outputs_quantized(
        outputs, scales, meta.input_width, score_threshold,
    )
    if len(predictions) == 0:
        return []
    input_boxes = _xywh_to_xyxy(predictions[:, :4])
    keep = _nms(input_boxes, predictions[:, 4], iou_threshold, max_det)
    return [
        SegDetection(
            score=float(predictions[index, 4]),
            box=_restore_box(input_boxes[index], meta),
            mask=None,
        )
        for index in keep
    ]
