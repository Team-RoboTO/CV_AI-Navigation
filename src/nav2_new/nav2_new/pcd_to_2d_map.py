#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


def load_ascii_pcd(path):
    with open(path, 'r', errors='ignore') as f:
        lines = f.readlines()
    fields = None
    data_start = None
    for i, line in enumerate(lines):
        if line.startswith('FIELDS'):
            fields = line.strip().split()[1:]
        if line.startswith('DATA'):
            if 'ascii' not in line:
                raise RuntimeError('Binary PCD not supported. Convert with: pcl_convert_pcd_ascii_binary input.pcd output_ascii.pcd 0')
            data_start = i + 1
            break
    if fields is None or data_start is None:
        raise RuntimeError('Invalid PCD file')
    xyz_idx = [fields.index('x'), fields.index('y'), fields.index('z')]
    pts = []
    for line in lines[data_start:]:
        vals = line.strip().split()
        if len(vals) <= max(xyz_idx):
            continue
        pts.append([float(vals[j]) for j in xyz_idx])
    return np.asarray(pts, dtype=np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('--out', default='/root/nav2_ws/maps/robomaster_map')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--z-min', type=float, default=-0.10)
    ap.add_argument('--z-max', type=float, default=0.30)
    ap.add_argument('--padding', type=float, default=1.0)
    ap.add_argument('--min-hits', type=int, default=3)
    ap.add_argument('--inflate', type=int, default=1)
    args = ap.parse_args()

    pts = load_ascii_pcd(args.pcd)
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = pts[(pts[:, 2] >= args.z_min) & (pts[:, 2] <= args.z_max)]
    if len(pts) == 0:
        raise RuntimeError('No points left after z filtering. Change --z-min/--z-max.')

    min_xy = pts[:, :2].min(axis=0) - args.padding
    max_xy = pts[:, :2].max(axis=0) + args.padding
    size_xy = np.ceil((max_xy - min_xy) / args.resolution).astype(int)
    width, height = int(size_xy[0]), int(size_xy[1])

    ij = np.floor((pts[:, :2] - min_xy) / args.resolution).astype(int)
    ij[:, 0] = np.clip(ij[:, 0], 0, width - 1)
    ij[:, 1] = np.clip(ij[:, 1], 0, height - 1)
    rows = height - 1 - ij[:, 1]
    cols = ij[:, 0]

    hits = np.zeros((height, width), dtype=np.uint16)
    for r, c in zip(rows, cols):
        if hits[r, c] < 65535:
            hits[r, c] += 1

    img = np.full((height, width), 254, dtype=np.uint8)
    img[hits >= args.min_hits] = 0

    for _ in range(args.inflate):
        occ = img == 0
        grown = occ.copy()
        grown[:-1, :] |= occ[1:, :]
        grown[1:, :] |= occ[:-1, :]
        grown[:, :-1] |= occ[:, 1:]
        grown[:, 1:] |= occ[:, :-1]
        img[grown] = 0

    out_base = Path(args.out)
    out_base.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = out_base.with_suffix('.pgm')
    yaml_path = out_base.with_suffix('.yaml')
    Image.fromarray(img).save(pgm_path)
    with open(yaml_path, 'w') as f:
        yaml.safe_dump({
            'image': pgm_path.name,
            'mode': 'trinary',
            'resolution': args.resolution,
            'origin': [float(min_xy[0]), float(min_xy[1]), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
        }, f, sort_keys=False)
    print(f'Saved: {pgm_path}')
    print(f'Saved: {yaml_path}')
    print(f'Points used: {len(pts)}')
    print(f'Map size: {width} x {height}')


if __name__ == '__main__':
    main()
