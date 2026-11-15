"""Augmented-reality overlay on the phone video: everything is drawn ON THE REAL FLOOR.

The floor plane comes from the tag itself: in every phone frame the tag (ID 3, 154 mm black square, flat on the floor)
is detected, and its four corners give the homography floor (metres, tag frame) -> image. So the overlay stays on
the floor even with a hand-held phone. Floor frame: origin = tag centre, v = towards the robot's start, u = across.

Drawn: spotlight on the test corridor (rest of the room blurred and darkened), a virtual ruler beside the real tape,
START 1.20 m / STOP 0.50 m lines, the EKF and wheels-only paths, the EKF footprint ("ghost") around the real robot,
and a flash on each camera fix.
"""
import json
import math

import cv2
import numpy as np

TAG = 0.154
OBJ = np.array([[-TAG / 2, -TAG / 2], [TAG / 2, -TAG / 2], [TAG / 2, TAG / 2], [-TAG / 2, TAG / 2]], np.float32)
BLUE, GREEN, ORANGE, PURPLE, GREY, WHITE = (255, 163, 77), (151, 220, 61), (71, 181, 255), (255, 140, 180), \
    (190, 180, 165), (248, 243, 238)                      # BGR, same palette as the dashboard
RULER = (77, 214, 255)
from textdraw import draw_text                           # noqa: E402


def smooth_corners(corners, win=7):
    c = np.array([np.array(x, float) if x is not None else np.full((4, 2), np.nan) for x in corners])
    out = c.copy()
    h = win // 2
    for i in range(len(c)):
        seg = c[max(0, i - h):i + h + 1]
        out[i] = np.nanmean(seg, axis=0)
    return out


class PhoneAR:
    def __init__(self, corners_json, d, sync_offset, start=1.20, front=0.16, stop=0.50, scale=2,
                 tape_u=0.16, u0=0.0):
        J = json.loads(open(corners_json).read())
        self.corners = smooth_corners(J['corners'])
        self.fps = J['fps']
        self.d, self.off = d, sync_offset
        self.start, self.front, self.stop, self.k = start, front, stop, scale
        self.tape_u = tape_u
        # path axis: from the robot's start centre towards the tag centre
        c0 = np.array([u0, start + front])
        self.a = -c0 / np.linalg.norm(c0)             # heading at start (towards the tag)
        self.left = np.array([-self.a[1], self.a[0]])  # robot's left
        self.c0 = c0

    # ---------------------------------------------------------------- geometry
    def H(self, fi):
        H, _ = cv2.findHomography(OBJ, self.corners[fi].astype(np.float32))
        return H

    def to_img(self, H, pts):
        p = cv2.perspectiveTransform(np.asarray(pts, np.float32).reshape(-1, 1, 2), H).reshape(-1, 2)
        return (p * self.k * 16).astype(np.int32)        # sub-pixel, scaled canvas

    def ekf_floor(self, x, y):
        """EKF (x forward from the start centre, y left) -> floor (u, v)."""
        return self.c0 + np.outer(x, self.a) + np.outer(y, self.left) if np.ndim(x) else \
            self.c0 + x * self.a + y * self.left

    def line_at(self, dist, half=0.42):
        """Floor segment across the path at `dist` from the tag centre (chassis-front distance)."""
        p = -self.a * dist                             # point on the axis, `dist` from the tag towards the start
        return [p - self.left * half, p + self.left * half]

    # ---------------------------------------------------------------- drawing
    def _poly(self, img, H, pts, col, th, closed=False):
        cv2.polylines(img, [self.to_img(H, pts)], closed, col, th, cv2.LINE_AA, shift=4)

    def _text(self, img, H, pt, txt, col, size=0.9, th=2, dx=0, dy=0):
        u, v = (self.to_img(H, [pt])[0] / 16).astype(int)
        draw_text(img, (u + dx, v + dy), txt, col, size=int(34 * size), weight='Heavy' if th >= 3 else 'Bold')

    def render(self, frame_bgr, fi, t, i, closeup=False):
        k = self.k
        img = cv2.resize(frame_bgr, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC)
        H = self.H(fi)
        d = self.d
        # spotlight: corridor around the path stays sharp, the room is blurred and darkened
        corridor = [(-0.48, -0.30), (0.48, -0.30), (0.48, 1.70), (-0.48, 1.70)]
        mask = np.zeros(img.shape[:2], np.uint8)
        cv2.fillPoly(mask, [self.to_img(H, corridor)], 255, cv2.LINE_AA, shift=4)
        mask = cv2.GaussianBlur(mask, (61, 61), 0).astype(np.float32)[:, :, None] / 255.0
        bg = cv2.GaussianBlur(img, (0, 0), 18) * 0.35
        img = (img * mask + bg * (1 - mask)).astype(np.uint8)
        if closeup:
            return img
        # virtual ruler beside the real tape: 0 at the start line, 120 cm at the tag centre (like the real tape)
        side = self.left * (self.tape_u + 0.09)       # ruler runs parallel to the path, just outside the tape
        for cm in range(0, 121, 5):
            base = -self.a * (self.start - cm / 100.0) + side
            ln = 0.05 if cm % 10 == 0 else 0.025
            self._poly(img, H, [base, base + self.left * ln], RULER, 3 if cm % 10 == 0 else 2)
            if cm % 20 == 0:
                self._text(img, H, base + self.left * 0.06, f'{cm}', RULER, 0.95, 2, dx=4, dy=10)
        self._poly(img, H, [-self.a * self.start + side, side], RULER, 2)
        self._text(img, H, side + self.left * 0.05 + self.a * 0.10, 'virtual ruler (cm)', RULER, 0.8, 2)
        # start / stop lines
        for dist, col, lab in [(self.start, GREY, 'START 1.20 m'), (self.stop, PURPLE, 'STOP 0.50 m')]:
            a, b = self.line_at(dist)
            self._poly(img, H, [a, b], col, 4)
            self._text(img, H, -self.a * dist - self.left * 0.10, lab, col, 1.25, 3, dx=-120, dy=-14)
        if t < 0:
            return img
        # paths (robot centre): wheels only (green) and EKF (blue)
        xo, yo = d['xo'][:i + 1], d['yo'][:i + 1]
        xe, ye = d['x'][:i + 1], d['y'][:i + 1]
        if len(xe) > 1:
            self._poly(img, H, self.ekf_floor(xo, yo), GREEN, 4)
            self._poly(img, H, self.ekf_floor(xe, ye), BLUE, 5)
        # EKF footprint ("ghost") at the estimated pose
        x, y, th = d['x'][i], d['y'][i], d['heading'][i]
        c = self.ekf_floor(x, y)
        hd = math.atan2(self.a[1], self.a[0]) + th
        f, l = np.array([math.cos(hd), math.sin(hd)]), np.array([-math.sin(hd), math.cos(hd)])
        box = [c + f * 0.16 + l * 0.12, c + f * 0.16 - l * 0.12, c - f * 0.16 - l * 0.12, c - f * 0.16 + l * 0.12]
        self._poly(img, H, box, BLUE, 4, closed=True)
        front = c + f * self.front
        self._poly(img, H, [front - l * 0.12, front + l * 0.12], (255, 220, 160), 5)
        # camera fix flash: front of the robot -> tag centre
        for kk in range(max(0, i - 5), i + 1):
            if d['tag_new'][kk] == 1 and d['tag_used'][kk] == 1:
                age = d['t'][i] - d['t'][kk]
                if age < 0.25:
                    ov = img.copy()
                    self._poly(ov, H, [front, (0, 0)], ORANGE, 4)
                    a = max(0.0, 0.85 - 3.0 * age)
                    cv2.addWeighted(ov, a, img, 1 - a, 0, dst=img)
        # estimate label next to the ghost
        self._text(img, H, c - self.a * 0.26, f"EKF  d = {d['d_est'][i]:.3f} m", (255, 220, 160), 1.35, 3, dx=-170)
        return img
