#!/usr/bin/env python3
"""Scene 3 of the demo video: the filmed run, phone video (with AR) + 3D + robot's eye + numbers + plot + terminal.

    env -u PYTHONPATH ~/phase3_video_venv/bin/python video_tools/build_scene3.py --from 17.0 --to 37.0

Inputs (all from the filmed demo run): ~/Videos/phase3/run.mp4, results/video_run/{trial_11_*, frames/,
phone_tag_corners.json, sync.json} and the launch log. Output: results/video/live.mp4
"""
import argparse
import glob
import json
import multiprocessing as mp
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'video_tools'))

G = {}


def init(cfg):
    import cv2
    import make_video as mv
    from phone_ar import PhoneAR
    d = mv.load_trial(sorted(glob.glob(str(ROOT / 'results/video_run/trial_11_*')))[-1])
    off = json.loads((ROOT / 'results/video_run/sync.json').read_text())['video_minus_data_s']
    t0 = d['summary']['t_start_unix']
    G.update(cfg=cfg, mv=mv, d=d, off=off, cv2=cv2,
             ar=PhoneAR(str(ROOT / 'results/video_run/phone_tag_corners.json'), d, off, u0=cfg['u0'],
                        tape_u=cfg['tape_u']),
             cap=cv2.VideoCapture(cfg['phone']),
             log=mv.load_log(cfg['log'], t0), frames=mv.load_frames(str(ROOT / 'results/video_run/frames'), t0))
    G['d']['summary']['trial'] = 'demo'


def one(job):
    n, fi = job
    cv2, mv, d, cfg = G['cv2'], G['mv'], G['d'], G['cfg']
    G['cap'].set(1, fi)
    ok, im = G['cap'].read()
    tv = fi / cfg['fps']
    t = tv - G['off']
    i = mv.idx_at(d, max(t, 0))
    closeup = cfg['zoom'][0] <= tv <= cfg['zoom'][1]
    ph = G['ar'].render(im, fi, t, i, closeup=closeup)
    ph = cv2.cvtColor(ph, cv2.COLOR_BGR2RGB)
    img = mv.render(d, t, phone_img=ph, frames=G['frames'], log=G['log'], tape=cfg['tape'], portrait=True)
    cv2.imwrite(os.path.join(cfg['tmp'], f'{n:05d}.png'), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--phone', default=os.path.expanduser('~/Videos/phase3/run.mp4'))
    ap.add_argument('--log', default='')
    ap.add_argument('--from', dest='t0', type=float, default=17.0)
    ap.add_argument('--to', dest='t1', type=float, default=37.0)
    ap.add_argument('--zoom', type=float, nargs=2, default=[28.9, 34.6], help='phone close-up on the tape (s)')
    ap.add_argument('--tape', type=float, default=0.478)
    ap.add_argument('--u0', type=float, default=0.041)
    ap.add_argument('--tape-u', type=float, default=0.13)
    ap.add_argument('--every', type=int, default=1, help='render every Nth phone frame (preview)')
    ap.add_argument('--workers', type=int, default=max(1, os.cpu_count() // 2))
    ap.add_argument('--out', default=str(ROOT / 'results/video/live.mp4'))
    a = ap.parse_args()
    import cv2
    import imageio_ffmpeg
    fps = cv2.VideoCapture(a.phone).get(5)
    tmp = Path(a.out).with_suffix('') .as_posix() + '_frames'
    os.makedirs(tmp, exist_ok=True)
    for f in glob.glob(tmp + '/*.png'):
        os.remove(f)
    cfg = dict(phone=a.phone, log=a.log, fps=fps, zoom=a.zoom, tape=a.tape, u0=a.u0, tape_u=a.tape_u, tmp=tmp)
    fis = list(range(int(a.t0 * fps), int(a.t1 * fps), a.every))
    with mp.get_context('spawn').Pool(a.workers, initializer=init, initargs=(cfg,)) as pool:
        for k, _ in enumerate(pool.imap_unordered(one, list(enumerate(fis)), chunksize=4)):
            if k % 50 == 0:
                print(f'{k}/{len(fis)}', flush=True)
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    subprocess.run([ff, '-y', '-loglevel', 'error', '-framerate', f'{fps / a.every:.3f}', '-i', f'{tmp}/%05d.png',
                    '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '18', '-preset', 'slow', a.out], check=True)
    print(a.out)


if __name__ == '__main__':
    main()
