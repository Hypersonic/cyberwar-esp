#!/usr/bin/env python3

from PIL import Image
from itertools import product
from math import sqrt
from random import shuffle, randint


FILE_HEADER = r"""
// ===== AUTOGENERATED FILE =====
"""


FILE_FOOTER = r"""
"""


def get_vga_color(r, g, b):
    r = int(round(r / 255))
    g = int(round(g / 255))
    b = int(round(b / 255))
    if (r, g, b) == (0, 0, 0): # black (ish)
        return 0
    if (r, g, b) == (1, 1, 1): # white (ish)
        return 0x1f
    raise ValueError('No implemented color for {}, {}, {}'.format(r, g, b))


def dist(a, b):
    ax, ay = a
    bx, by = b
    return sqrt((ax - bx) ** 2 + (ay - by) ** 2)


def order_for_render(map_pixels):
    """
    Basically, keep jumping to the closest thing. Looks kinda like a CRT beam!
    """
    last_x, last_y = 0, 0
    ordered = []
    while map_pixels:
        closest = min(map_pixels, key=lambda px: dist((px[0], px[1]), (last_x, last_y)))
        map_pixels.remove(closest)
        ordered.append(closest)
        last_x, last_y = closest[0], closest[1]
    return ordered


im = Image.open('./world.png')


map_pixels = []
for x, y in product(range(im.width), range(im.height)):
    r, g, b = im.getpixel((x, y))
    try:
        color = get_vga_color(r, g, b)
    except:
        raise ValueError('at {} {}'.format(x, y))
    if color != 0: # don't put black pixels, they're in the screen clear
        map_pixels.append((
            x,
            y,
            color
        ))

map_pixels = order_for_render(map_pixels)


print(FILE_HEADER)
for x, y, color in map_pixels:
    print('{{{}, {}}},'.format(x, y))
print(FILE_FOOTER)