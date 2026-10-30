# Copyright 2026 Pavan Patil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Arrange the frames saved by record_demo.py into a short portrait video for a social post.

    python3 docs/demo/record_demo.py --goals 4 0 --raw-dir /path/to/raw --output /tmp/x.mp4
    python3 docs/demo/make_linkedin_video.py --raw-dir /path/to/raw --output post.mp4

The result is 1080x1350 (4:5), about 45 seconds, with no sound, so every message is a caption:
a hook, how the system works, the two docks played with a speed ramp (fast while it searches
and approaches, real time for the last centimetres) with live numbers, the test results, and
an end card with the repository address. Every number on the cards comes from docs/test_plan.md.
Needs PIL, OpenCV, NumPy and ffmpeg.
"""

import argparse
import json
import os
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

W, H = 1080, 1350
FPS = 30           # output frame rate
SRC_FPS = 15       # rate the raw frames were saved at, per simulated second
BG = (9, 14, 26)
BG2 = (22, 36, 62)
ACCENT = (56, 189, 248)
GREEN = (74, 222, 128)
AMBER = (251, 191, 36)
RED = (248, 113, 113)
WHITE = (245, 247, 250)
GREY = (160, 172, 190)
STATE_NAMES = {0: 'IDLE', 1: 'SEARCH', 2: 'APPROACH', 3: 'FINAL', 4: 'RECOVER', 5: 'DOCKED',
               6: 'FAILED'}
STATE_COLORS = {1: AMBER, 2: ACCENT, 3: GREEN, 4: RED, 5: GREEN}
# How many times faster than real time each controller state is played; the last centimetres
# of a dock run in real time so the careful final approach can be seen.
SPEEDS = {None: 2.0, 0: 1.0, 1: 12.0, 2: 10.0, 3: 1.0, 4: 12.0, 5: 1.0}
HOLD_FRAMES = 33   # about 2.2 s of the finished dock stays on screen
FONT_DIRS = ['/usr/share/fonts/truetype/lato', '/usr/share/fonts/truetype/dejavu']
FONT_FILES = {'heavy': ['Lato-Heavy.ttf', 'DejaVuSans-Bold.ttf'],
              'bold': ['Lato-Bold.ttf', 'DejaVuSans-Bold.ttf'],
              'regular': ['Lato-Regular.ttf', 'DejaVuSans.ttf']}
REPO = 'github.com/pavanmp140102/ArUco_ROS_Nav'
_fonts = {}
_cache = {}


def font(weight, size):
    """Return a font of the given weight and size, falling back to what the machine has."""
    key = (weight, size)
    if key not in _fonts:
        for name in FONT_FILES[weight]:
            for directory in FONT_DIRS:
                path = os.path.join(directory, name)
                if os.path.exists(path):
                    _fonts[key] = ImageFont.truetype(path, size)
                    break
            if key in _fonts:
                break
        else:
            _fonts[key] = ImageFont.load_default()
    return _fonts[key]


def mix(a, b, t):
    """Blend colour a towards colour b by t, which is how text fades in on the dark cards."""
    t = max(0.0, min(1.0, t))
    return tuple(int(round(x + (y - x) * t)) for x, y in zip(a, b))


def gradient():
    """Return the dark vertical gradient used behind every card."""
    column = np.linspace(0, 1, H)[:, None, None]
    top, bottom = np.array(BG, float), np.array(BG2, float)
    return (top + (bottom - top) * column).repeat(W, axis=1).astype(np.uint8)


def draw_text(d, xy, text, weight, size, fill, anchor='la', bg=BG, alpha=1.0):
    """Draw text that fades in by blending its colour from the background."""
    if alpha > 0:
        d.text(xy, text, font=font(weight, size), fill=mix(bg, fill, alpha), anchor=anchor)


def fade(t, start, length=0.45):
    """Return 0 before start and rise to 1 over length seconds, for staggered reveals."""
    return max(0.0, min(1.0, (t - start) / length))


def read_log(raw_dir):
    """Return the per-frame state log written by record_demo.py."""
    with open(os.path.join(raw_dir, 'log.jsonl')) as f:
        return [json.loads(line) for line in f]


def goal_span(rows, marker):
    """Return the first frame of a goal and the frame where its result appears."""
    first = next(r['frame'] for r in rows if r['goal'] == marker and r['state'] == 1)
    done = next(r['frame'] for r in rows if r['goal'] == marker and r['result'])
    return first, done


def card_hook(raw_dir, t):
    """Frame of the opening card: the claim, over a dimmed and blurred shot of the world."""
    if 'hook_bg' not in _cache:
        shot = Image.open(os.path.join(raw_dir, 'overview_00040.jpg')).convert('RGB')
        shot = shot.resize((int(shot.width * H / shot.height), H)).filter(
            ImageFilter.GaussianBlur(9))
        left = (shot.width - W) // 2
        shot = np.asarray(shot.crop((left, 0, left + W, H)), float) * 0.28
        _cache['hook_bg'] = np.clip(shot + np.array(BG, float) * 0.5, 0, 255).astype(np.uint8)
    img = Image.fromarray(_cache['hook_bg'])
    d = ImageDraw.Draw(img)
    bg = (30, 45, 68)
    draw_text(d, (W // 2, 520), 'A robot that docks', 'heavy', 100, WHITE, 'mm', bg, fade(t, 0.1))
    draw_text(d, (W // 2, 640), 'using only a camera.', 'heavy', 100, ACCENT, 'mm', bg,
              fade(t, 0.7))
    draw_text(d, (W // 2, 790), 'No map. No GPS. No odometry.', 'regular', 52, GREY, 'mm', bg,
              fade(t, 1.5))
    return np.asarray(img)


def card_pipeline(marker_png, t):
    """Frame of the 'how it works' card: four steps that appear one after another."""
    img = Image.fromarray(gradient())
    d = ImageDraw.Draw(img)
    draw_text(d, (W // 2, 120), 'How it works', 'heavy', 76, WHITE, 'mm', BG, fade(t, 0))
    steps = [('Camera', 'sees an ArUco marker'),
             ('Detector', 'works out where the marker is'),
             ('Controller', 'steers, slows down, lines up'),
             ('Wheels', 'stop 10 cm from the marker')]
    for i, (title, detail) in enumerate(steps):
        a = fade(t, 0.4 + i * 0.75)
        if a <= 0:
            continue
        y = 235 + i * 258
        box = mix(BG2, (36, 56, 92), a)
        d.rounded_rectangle((70, y, W - 70, y + 205), 30, fill=box, outline=mix(BG2, ACCENT, a),
                            width=3)
        d.ellipse((105, y + 55, 195, y + 145), fill=mix(BG2, ACCENT, a))
        draw_text(d, (150, y + 100), str(i + 1), 'heavy', 58, BG, 'mm', ACCENT, a)
        draw_text(d, (235, y + 72), title, 'heavy', 62, WHITE, 'lm', box, a)
        draw_text(d, (235, y + 142), detail, 'regular', 40, GREY, 'lm', box, a)
        if i == 0:
            icon = marker_png.resize((150, 150), Image.NEAREST)
            img.paste(icon, (W - 70 - 40 - 150, y + 28))
        if i < 3:
            draw_text(d, (W // 2, y + 232), 'v', 'bold', 34, ACCENT, 'mm', BG, a)
    return np.asarray(img)


def card_numbers(t):
    """Frame of the results card, with figures taken from the project's test plan."""
    img = Image.fromarray(gradient())
    d = ImageDraw.Draw(img)
    draw_text(d, (W // 2, 170), 'Tested end to end', 'heavy', 76, WHITE, 'mm', BG, fade(t, 0))
    rows = [('12 / 12', 'scenarios passed,\n3 full runs in a row'),
            ('6 mm', 'worst stopping error\nagainst the 10 cm target'),
            ('1 mm', 'spread over 30\nrepeat docks')]
    for i, (big, small) in enumerate(rows):
        a = fade(t, 0.4 + i * 0.6)
        y = 330 + i * 300
        draw_text(d, (80, y), big, 'heavy', 130, ACCENT, 'lm', BG2, a)
        draw_text(d, (560, y - 5), small, 'regular', 46, WHITE, 'lm', BG2, a)
    return np.asarray(img)


def card_end(t):
    """Frame of the closing card: the project's name, what it is, and where to find it."""
    img = Image.fromarray(gradient())
    d = ImageDraw.Draw(img)
    draw_text(d, (W // 2, 420), 'dockbot', 'heavy', 150, ACCENT, 'mm', BG, fade(t, 0.1))
    draw_text(d, (W // 2, 560), 'Autonomous docking', 'bold', 62, WHITE, 'mm', BG, fade(t, 0.5))
    draw_text(d, (W // 2, 635), 'with ArUco markers', 'bold', 62, WHITE, 'mm', BG, fade(t, 0.5))
    draw_text(d, (W // 2, 780), 'ROS 2 Jazzy  |  Gazebo Harmonic  |  C++17', 'regular', 40, GREY,
              'mm', BG, fade(t, 0.9))
    a = fade(t, 1.3)
    d.rounded_rectangle((70, 880, W - 70, 990), 28, outline=mix(BG2, ACCENT, a), width=3)
    draw_text(d, (W // 2, 935), REPO, 'bold', 41, WHITE, 'mm', BG2, a)
    draw_text(d, (W // 2, 1070), 'Open source, Apache-2.0', 'regular', 38, GREY, 'mm', BG, a)
    return np.asarray(img)


def caption_for(marker, row, finished):
    """Return the two caption lines and their colour for the controller's current state."""
    s = row['state']
    if finished:
        return 'Docked', 'front face %.1f cm from the marker' % finished['gap_cm'], GREEN
    if row['goal'] is None:
        if marker == 4:
            return '5 markers in the room', 'Task: dock at marker 4', WHITE
        return 'Next: marker 0', 'Marker 1 is right beside it', WHITE
    if s == 1:
        return 'Looking for marker %d' % marker, 'Only the camera. No map.', AMBER
    if s == 2:
        if marker == 0:
            return 'Picks marker 0', 'not its neighbour', ACCENT
        return 'Marker spotted: steering to it', 'Position worked out from the image', ACCENT
    if s == 4:
        return 'Not lined up well enough?', 'It backs off and tries again', RED
    if s == 3:
        return 'Final approach', 'slow and straight', GREEN
    return 'Docking to marker %d' % marker, '', WHITE


class Footage:
    """Draw frames of one dock: third-person view, the robot's camera, captions and numbers."""

    def __init__(self, raw_dir, rows, marker):
        """Remember where the frames are and which goal this footage is about."""
        self.raw_dir, self.rows, self.marker = raw_dir, rows, marker
        self.template = Image.fromarray(gradient())
        self.last_caption, self.caption_since = None, -99.0

    def frame(self, idx, out_t, speed, finished):
        """Return one output frame for source frame idx at output time out_t seconds."""
        row = self.rows[idx]
        img = self.template.copy()
        d = ImageDraw.Draw(img)
        over = Image.open(os.path.join(self.raw_dir, 'overview_%05d.jpg' % idx)).convert('RGB')
        img.paste(over.crop((0, 140, W, 860)), (0, 150))
        # A caption stays for at least a second, so a flicker between searching and approaching
        # does not flash it; the final approach, a recovery and the result switch at once.
        wanted = caption_for(self.marker, row, finished)
        urgent = finished or row['state'] in (3, 4)
        if self.last_caption is None or (wanted != self.last_caption and (
                urgent or out_t - self.caption_since > 1.0)):
            self.last_caption, self.caption_since = wanted, out_t
        line1, line2, color = self.last_caption
        line1, line2, color = self.last_caption
        d.rectangle((0, 0, W, 150), fill=BG)
        d.rectangle((0, 144, W, 150), fill=color)
        d.text((40, 52), line1, font=font('heavy', 56), fill=WHITE, anchor='lm')
        d.text((40, 110), line2, font=font('regular', 36), fill=GREY, anchor='lm')
        if speed > 1.05:
            d.rounded_rectangle((W - 170, 168, W - 20, 224), 14, fill=(0, 0, 0))
            d.text((W - 95, 197), '%gx' % speed, font=font('heavy', 34), fill=WHITE, anchor='mm')
        d.text((20, 898), "ROBOT'S CAMERA", font=font('bold', 28), fill=GREY, anchor='lm')
        robot = Image.open(os.path.join(self.raw_dir, 'robot_%05d.jpg' % idx)).convert('RGB')
        img.paste(robot, (20, 925))
        d.rounded_rectangle((20, 925, 660, 1285), 6, outline=mix(BG2, color, 0.8), width=3)
        self.stats(d, row, finished, color)
        d.text((W // 2, 1322), 'dockbot  |  ROS 2  |  Gazebo  |  C++', font=font('regular', 28),
               fill=GREY, anchor='mm')
        return np.asarray(img)

    def stats(self, d, row, finished, color):
        """Draw the controller state chip and the live distance, offset and heading."""
        x = 690
        name = 'DOCKED' if finished else STATE_NAMES.get(row['state'], '')
        if finished or row['state'] in STATE_COLORS:
            chip = GREEN if finished else STATE_COLORS[row['state']]
            d.rounded_rectangle((x, 925, W - 20, 995), 16, fill=chip)
            d.text(((x + W - 20) // 2, 960), name, font=font('heavy', 44), fill=BG, anchor='mm')
        live = row['state'] in (1, 2, 3, 4) and not finished
        if finished:  # the controller's own numbers read zero once docked, so show the result
            lines = [('%.1f cm' % finished['gap_cm'], 'front face to marker'),
                     ('10 cm', 'target distance')]
        else:
            lines = []
            for label, key, fmt in [('to the stop point', 'range_cm', '%.0f cm'),
                                    ('off to the side', 'lateral_cm', '%+.1f cm'),
                                    ('heading error', 'heading_deg', '%+.1f deg')]:
                value = row[key]
                lines.append((fmt % value if live and value is not None else '-', label))
        for i, (shown, label) in enumerate(lines):
            y = 1047 + i * 84
            d.text((x, y), shown, font=font('heavy', 52), fill=WHITE, anchor='lm')
            d.text((x, y + 38), label, font=font('regular', 30), fill=GREY, anchor='lm')


def run_frames(raw_dir, rows, marker, start, done):
    """Yield the output frames of one dock, speeding up and slowing down with its state."""
    footage = Footage(raw_dir, rows, marker)
    end = min(done + HOLD_FRAMES, len(rows) - 1)
    pos, n = float(start), 0
    while pos <= end:
        idx = int(pos)
        row = rows[idx]
        finished = rows[idx]['result'] if idx >= done else None
        speed = SPEEDS[5] if finished else SPEEDS.get(row['state'], 1.0)
        if row['goal'] is None:
            speed = SPEEDS[None]
        yield footage.frame(idx, n / FPS, speed, finished)
        pos += speed * SRC_FPS / FPS  # source frames to move on for one output frame
        n += 1


def with_fades(frames, seconds=0.3):
    """Fade a clip in from, and out to, the card background so the cuts feel soft."""
    frames = list(frames)
    k = int(seconds * FPS)
    back = np.array(BG, float)
    for i, f in enumerate(frames):
        a = min(1.0, (i + 1) / k, (len(frames) - i) / k)
        yield (back + (f.astype(float) - back) * a).astype(np.uint8) if a < 1 else f


def cards(fn, seconds, *args):
    """Yield the frames of a card whose appearance depends on time."""
    for i in range(int(seconds * FPS)):
        yield fn(*args, i / FPS)


def main():
    """Build the video from the raw frames and write it with ffmpeg."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--raw-dir', required=True)
    parser.add_argument('--output', default='docs/demo/dockbot_linkedin.mp4')
    args = parser.parse_args()
    rows = read_log(args.raw_dir)
    here = os.path.dirname(os.path.abspath(__file__))
    marker_png = Image.open(os.path.join(
        here, '..', '..', 'src', 'dockbot_gazebo', 'models', 'aruco_marker_4', 'materials',
        'textures', 'marker_4.png')).convert('RGB')
    first4, done4 = goal_span(rows, 4)
    first0, done0 = goal_span(rows, 0)
    parts = [
        with_fades(cards(card_hook, 3.0, args.raw_dir)),
        with_fades(cards(card_pipeline, 4.6, marker_png)),
        with_fades(run_frames(args.raw_dir, rows, 4, max(0, first4 - 6), done4)),
        with_fades(run_frames(args.raw_dir, rows, 0, max(0, first0 - 12), done0)),
        with_fades(cards(card_numbers, 3.6)),
        with_fades(cards(card_end, 4.0)),
    ]
    out_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(out_dir, exist_ok=True)
    ff = subprocess.Popen(
        ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
         '-s', '%dx%d' % (W, H), '-r', str(FPS), '-i', '-', '-c:v', 'libx264',
         '-pix_fmt', 'yuv420p', '-crf', '20', '-preset', 'slow', '-movflags', '+faststart',
         args.output], stdin=subprocess.PIPE)
    count = 0
    for part in parts:
        for frame in part:
            ff.stdin.write(frame.tobytes())
            count += 1
    ff.stdin.close()
    ff.wait()
    print('wrote %s: %d frames, %.1f s' % (args.output, count, count / FPS))


if __name__ == '__main__':
    main()
