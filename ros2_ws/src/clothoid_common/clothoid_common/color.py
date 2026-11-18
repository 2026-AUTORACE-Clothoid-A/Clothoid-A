"""Current-image color segmentation shared by detectors and diagnostic viewers.

Optional blue glare suppression precedes channel-wise RGB gamma
round(255*(input/255)**gamma) and CLAHE on 8-bit Lab L. Apply before blur/BEV. Classify Lab using
OpenCV float CIELab (L 0..100, signed a/b), RGB differences in signed arithmetic.
No temporal prediction, interpolation of missing lines, or camera controls.
"""
from dataclasses import asdict, dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class ColorParams:
    color_method: str = 'hsv'
    gamma_enabled: bool = False
    gamma: float = 1.2
    clahe_enabled: bool = False
    clahe_clip_limit: float = 2.0
    clahe_grid_size: int = 8
    lab_yellow_b_min: float = 2.0
    lab_yellow_a_abs_max: float = 25.0
    lab_yellow_l_min: float = 40.0
    lab_white_chroma_max: float = 16.0
    lab_white_l_min: float = 80.0
    rgb_yellow_difference_min: int = 5
    rgb_yellow_v_min: int = 90
    rgb_white_spread_max: int = 30
    rgb_white_v_min: int = 190
    blue_glare_strength: float = 0.0

    def __post_init__(self):
        if self.color_method not in ('hsv', 'lab', 'rgb'):
            raise ValueError('color_method must be hsv, lab or rgb')
        for name in ('gamma_enabled', 'clahe_enabled'):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f'{name} must be boolean')
        bounds = {'gamma': (.1, 4.), 'blue_glare_strength': (0., 1.),
                  'clahe_clip_limit': (.1, 16.), 'clahe_grid_size': (2, 32),
                  'lab_yellow_b_min': (0., 127.), 'lab_yellow_a_abs_max': (0., 128.),
                  'lab_yellow_l_min': (0., 100.), 'lab_white_chroma_max': (0., 200.),
                  'lab_white_l_min': (0., 100.), 'rgb_yellow_difference_min': (0, 255),
                  'rgb_yellow_v_min': (0, 255), 'rgb_white_spread_max': (0, 255),
                  'rgb_white_v_min': (0, 255)}
        integers = {'clahe_grid_size', 'rgb_yellow_difference_min', 'rgb_yellow_v_min',
                    'rgb_white_spread_max', 'rgb_white_v_min'}
        for name, (low, high) in bounds.items():
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value) or not low <= value <= high or
                    (name in integers and not isinstance(value, int))):
                raise ValueError(f'{name} must be {low}..{high}' + (' (integer)' if name in integers else ''))


COLOR_NAMES = tuple(ColorParams.__dataclass_fields__)


def color_defaults():
    return asdict(ColorParams())


def color_params(value):
    return ColorParams(**{name: value(name) for name in COLOR_NAMES})


def validate_hsv(lower, upper, white_h_min, white_h_max, white_s_max, white_v_min):
    if len(lower) != 3 or len(upper) != 3:
        raise ValueError('HSV bounds require three channels')
    for lo, hi, maximum in zip(lower, upper, (179, 255, 255)):
        if not isinstance(lo, (int, np.integer)) or not isinstance(hi, (int, np.integer)) or not 0 <= lo <= hi <= maximum:
            raise ValueError('HSV bounds must be ordered within H 0..179, S/V 0..255')
    if not 0 <= white_h_min <= white_h_max <= 179:
        raise ValueError('white hue must satisfy 0 <= white_h_min <= white_h_max <= 179')
    if not 0 <= white_s_max <= 255 or not 0 <= white_v_min <= 255:
        raise ValueError('white S/V must be 0..255')


class ColorPipeline:
    def __init__(self, params=None):
        self.p = params or ColorParams()
        levels = np.arange(256)
        self._glare_weights = None
        if self.p.blue_glare_strength:
            # Dark floor receives the full correction. Fade it out as both R and G brighten.
            self._glare_weights = (self.p.blue_glare_strength
                                   * np.clip((135 - levels) / 65, 0, 1)).astype(np.float32)
            self._glare_red = ((levels + 1) // 3).astype(np.uint8)
        self.lut = (np.rint(255*(levels/255.)**self.p.gamma).astype(np.uint8)
                    if self.p.gamma_enabled else None)
        self.clahe = (cv2.createCLAHE(clipLimit=self.p.clahe_clip_limit,
                      tileGridSize=(self.p.clahe_grid_size, self.p.clahe_grid_size))
                      if self.p.clahe_enabled else None)

    def preprocess(self, bgr):
        image = bgr
        if self._glare_weights is not None:
            blue, green, red = cv2.split(image)
            # Blue dominates the stage-light reflection. Preserve the first six levels as noise margin.
            excess = cv2.subtract(cv2.subtract(blue, cv2.max(green, red)), 6)
            weights = cv2.LUT(cv2.min(green, red), self._glare_weights)
            correction = cv2.multiply(excess, weights, dtype=cv2.CV_8U)
            # The violet spill contributes some red too. Keep at least the original green level.
            red_correction = cv2.min(cv2.subtract(red, green), cv2.LUT(correction, self._glare_red))
            image = cv2.merge((cv2.subtract(blue, correction), green, cv2.subtract(red, red_correction)))
        if self.lut is not None:
            image = cv2.LUT(image, self.lut)
        if self.clahe is not None:
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            lab[..., 0] = self.clahe.apply(lab[..., 0])
            image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        return image

    def warp(self, bgr, matrix, size):
        return cv2.warpPerspective(cv2.GaussianBlur(self.preprocess(bgr), (5, 5), 0),
                                   matrix, size, flags=cv2.INTER_LINEAR)

    def masks(self, warped, *, hsv_lower=(15, 20, 160), hsv_upper=(40, 255, 255),
              white_h_min=0, white_h_max=179, white_s_max=50, white_v_min=190,
              exclude_yellow=True):
        p = self.p
        if p.color_method == 'hsv':
            hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
            yellow = cv2.inRange(hsv, np.array(hsv_lower), np.array(hsv_upper)) > 0
            white = ((hsv[..., 0] >= white_h_min) & (hsv[..., 0] <= white_h_max) &
                     (hsv[..., 1] <= white_s_max) & (hsv[..., 2] >= white_v_min))
        elif p.color_method == 'lab':
            lab = cv2.cvtColor(warped.astype(np.float32)/255., cv2.COLOR_BGR2LAB)
            l, a, b = lab.transpose(2, 0, 1)
            yellow = (b >= p.lab_yellow_b_min) & (np.abs(a) <= p.lab_yellow_a_abs_max) & (l >= p.lab_yellow_l_min)
            white = (np.hypot(a, b) <= p.lab_white_chroma_max) & (l >= p.lab_white_l_min)
        else:
            b, g, r = warped.astype(np.int16).transpose(2, 0, 1)
            v = np.maximum(np.maximum(r, g), b)
            yellow = (np.minimum(r, g)-b >= p.rgb_yellow_difference_min) & (v >= p.rgb_yellow_v_min)
            white = (v-np.minimum(np.minimum(r, g), b) <= p.rgb_white_spread_max) & (v >= p.rgb_white_v_min)
        return yellow, white & ~yellow if exclude_yellow else white
