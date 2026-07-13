#!/usr/bin/env python3
# Copyright (C) 2025 Kartik Vadhawana
#
# Render a KITTI Velodyne .bin LiDAR frame with matplotlib: ground points
# (brown), obstacle points (gray), plus an intensity-colored view. Exports a
# PNG. Ground/obstacle split uses a simple height threshold that mirrors the
# C++ RANSAC extractor's intent.
#
#   python visualize_pointcloud.py --bin 0000000000.bin --out frame_0000.png
#   python visualize_pointcloud.py --demo --out frame_0000.png

import argparse
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless / CI-safe
import matplotlib.pyplot as plt


def load_bin(path):
    """KITTI Velodyne: flat float32 (x, y, z, intensity)."""
    pts = np.fromfile(path, dtype=np.float32)
    if pts.size % 4 != 0:
        raise ValueError(f"{path}: byte count not a multiple of 16")
    return pts.reshape(-1, 4)


def demo_cloud(n_ground=8000, n_obstacle=2000, ground_z=-1.7, seed=7):
    rng = np.random.default_rng(seed)
    r = rng.uniform(3, 60, n_ground)
    a = rng.uniform(0, 2 * np.pi, n_ground)
    ground = np.column_stack([r * np.cos(a), r * np.sin(a),
                              ground_z + rng.uniform(-0.03, 0.03, n_ground),
                              rng.uniform(0.05, 0.95, n_ground)])
    r2 = rng.uniform(3, 60, n_obstacle)
    a2 = rng.uniform(0, 2 * np.pi, n_obstacle)
    obs = np.column_stack([r2 * np.cos(a2), r2 * np.sin(a2),
                           rng.uniform(-1.0, 3.0, n_obstacle),
                           rng.uniform(0.05, 0.95, n_obstacle)])
    return np.vstack([ground, obs])


def render(pts, out_path, ground_threshold=-1.4):
    x, y, z, inten = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
    ground_mask = z < ground_threshold
    ratio = float(ground_mask.mean()) if pts.shape[0] else 0.0

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    ax1.scatter(x[ground_mask], y[ground_mask], s=0.5, c="saddlebrown", label="ground")
    ax1.scatter(x[~ground_mask], y[~ground_mask], s=0.5, c="dimgray", label="obstacle")
    ax1.set_title(f"Ground / obstacle (ground ratio {ratio:.2f})")
    ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
    ax1.set_aspect("equal"); ax1.legend(markerscale=10, loc="upper right")

    sc = ax2.scatter(x, y, s=0.5, c=inten, cmap="viridis")
    ax2.set_title("Intensity")
    ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)")
    ax2.set_aspect("equal")
    fig.colorbar(sc, ax=ax2, label="intensity")

    fig.suptitle(f"KITTI LiDAR frame — {pts.shape[0]} points")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return ratio


def main():
    ap = argparse.ArgumentParser(description="Visualize a KITTI LiDAR frame.")
    ap.add_argument("--bin", help="path to a KITTI Velodyne .bin frame")
    ap.add_argument("--demo", action="store_true", help="synthesize a demo cloud")
    ap.add_argument("--out", default="frame_0000.png")
    args = ap.parse_args()

    pts = demo_cloud() if (args.demo or not args.bin) else load_bin(args.bin)
    ratio = render(pts, args.out)
    print(f"wrote {args.out} ({os.path.getsize(args.out)} bytes): "
          f"{pts.shape[0]} points, ground ratio {ratio:.2f}")


if __name__ == "__main__":
    main()
