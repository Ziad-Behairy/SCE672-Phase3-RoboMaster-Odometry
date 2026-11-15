"""Anti-aliased Lato text on numpy images (OpenCV's Hershey fonts look dated in a video)."""
from functools import lru_cache

import numpy as np
from PIL import Image, ImageDraw, ImageFont

LATO = '/usr/share/fonts/truetype/lato/Lato-{}.ttf'


@lru_cache(maxsize=64)
def font(size, weight='Bold'):
    return ImageFont.truetype(LATO.format(weight), int(size))


def draw_text(img, xy, text, color, size=28, weight='Bold', shadow=True, anchor='ls'):
    """Draw `text` in place on a uint8 HxWx3 array. Colour tuple is in the array's own channel order.
    anchor: PIL anchor, 'ls' = left/baseline, 'ms' = middle/baseline."""
    pil = Image.fromarray(img)
    d = ImageDraw.Draw(pil)
    f = font(size, weight)
    x, y = int(xy[0]), int(xy[1])
    if shadow:
        d.text((x, y), text, font=f, fill=(0, 0, 0), anchor=anchor, stroke_width=max(2, int(size / 9)),
               stroke_fill=(0, 0, 0))
    d.text((x, y), text, font=f, fill=tuple(int(c) for c in color), anchor=anchor)
    img[:] = np.asarray(pil)
    return img
