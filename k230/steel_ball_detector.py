"""Local steel236 inference using the Yahboom firmware's native YOLO11 runtime."""

import math

MODEL_PATH = "/sdcard/kmodel/steel_ball_236_nncase2.9.kmodel"
MODEL_INPUT_SIZE = [320, 320]
CONF_THRESHOLD = 0.50
# Start with the whole image; restrict this to the real platform after mounting.
ROI = (0, 0, 640, 480)


def select_ball(result, width=640, height=480, roi=ROI):
    # Native libs.YOLO returns [xywh_boxes, class_ids, scores], not Nx6 rows.
    if result is None or len(result) != 3:
        return None
    boxes, classes, scores = result
    if not (len(boxes) == len(classes) == len(scores)):
        return None
    rx, ry, rw, rh = roi
    best = None
    best_score = -1.0
    for box, class_id, score in zip(boxes, classes, scores):
        try:
            x, y, w, h = (float(value) for value in box)
            score = float(score)
            if class_id != 0 or not all(math.isfinite(v) for v in (x, y, w, h, score)):
                continue
            cx, cy = x + w / 2, y + h / 2
            if not (w > 0 and h > 0 and CONF_THRESHOLD <= score <= 1
                    and 0 <= cx < width and 0 <= cy < height
                    and rx <= cx < rx + rw and ry <= cy < ry + rh):
                continue
        except (TypeError, ValueError, OverflowError):
            continue
        if score > best_score:
            best_score = score
            best = (cx, cy, int(round(score * 100)), x, y, w, h)
    return best


class SteelBallDetector:
    def __init__(self, width=640, height=480):
        from libs.YOLO import YOLO11
        self.width, self.height = width, height
        self.last_box = None
        self.model = YOLO11(
            task_type="detect", mode="video", kmodel_path=MODEL_PATH,
            labels=["steel"], rgb888p_size=[width, height],
            model_input_size=MODEL_INPUT_SIZE, display_size=[width, height],
            conf_thresh=CONF_THRESHOLD, nms_thresh=0.45,
            max_boxes_num=10, debug_mode=0,
        )
        self.model.config_preprocess()

    def detect(self, frame):
        selected = select_ball(self.model.run(frame.to_numpy_ref()), self.width, self.height)
        self.last_box = selected[3:] if selected is not None else None
        return selected[:3] if selected is not None else None

    def deinit(self):
        self.model.deinit()
