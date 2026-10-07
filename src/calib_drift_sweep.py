"""Topic A - Đo độ nhạy của LiDAR-camera projection với calibration drift.

Ý tưởng: lấy các điểm LiDAR nằm trong 3D box GT (tính bằng calib ĐÚNG), rồi chiếu chúng
lên ảnh bằng calib BỊ LỆCH. Nếu calib đúng, gần như mọi điểm đó phải rơi vào 2D box GT.
Metric chính: in_box_pct = % điểm của object rơi đúng vào 2D box của chính nó.

Thay đổi đúng 1 yếu tố mỗi lần (yaw, pitch, hoặc dịch ty), giữ nguyên dữ liệu và mọi thứ khác.

Chạy:
    python -m src.calib_drift_sweep --data-root data/kitti_mini
    python -m src.calib_drift_sweep --data-root data/nuscenes_mini_subset --max-frames 20
    python -m src.calib_drift_sweep --help
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from starter.datasets import list_frames, load_frame
from starter.projection import cam_to_image, perturb_extrinsic, velo_to_cam

CLASSES = {"Car", "Van", "Truck", "Pedestrian", "Cyclist", "Person_sitting", "Tram",
           "car", "truck", "bus", "pedestrian", "bicycle", "motorcycle"}
# Mỗi dòng: (tên perturbation, roll, pitch, yaw, (tx, ty, tz))
LEVELS = [("none", 0, 0, 0, (0, 0, 0))]
LEVELS += [(f"yaw_{d}deg", 0, 0, d, (0, 0, 0)) for d in (0.5, 1.0, 2.0, 3.0)]
LEVELS += [(f"pitch_{d}deg", 0, d, 0, (0, 0, 0)) for d in (0.5, 1.0, 2.0, 3.0)]
LEVELS += [(f"ty_{int(c)}cm", 0, 0, 0, (0, c / 100, 0)) for c in (2, 5, 10)]


def points_in_box3d(pts_cam: np.ndarray, obj) -> np.ndarray:
    """Mask điểm (rectified camera frame) nằm trong 3D box KITTI của obj."""
    h, w, l = obj.dimensions
    d = pts_cam - obj.location                      # gốc toạ độ = tâm đáy box
    c, s = np.cos(obj.rotation_y), np.sin(obj.rotation_y)
    x_loc = c * d[:, 0] - s * d[:, 2]               # xoay ngược quanh trục y
    z_loc = s * d[:, 0] + c * d[:, 2]
    return (np.abs(x_loc) <= l / 2) & (np.abs(z_loc) <= w / 2) & (d[:, 1] <= 0) & (d[:, 1] >= -h)


def dist_bin(z: float) -> str:
    return "near<15m" if z < 15 else ("mid15-30m" if z < 30 else "far>30m")


def run(data_root: str, max_frames: int, min_pts: int) -> pd.DataFrame:
    rows = []
    for fid in list_frames(data_root)[:max_frames]:
        fr = load_frame(data_root, fid)
        pts = fr["points"][:, :3]
        shape = fr["image"].shape
        cam_true = velo_to_cam(pts, fr["calib"])
        objs = []
        for o in fr["labels"]:
            if o.type not in CLASSES or o.location[2] <= 0:
                continue
            m = points_in_box3d(cam_true, o)
            if m.sum() >= min_pts:
                objs.append((o, m))
        for name, r, p, y, t in LEVELS:
            calib = perturb_extrinsic(fr["calib"], r, p, y, t)
            cam = velo_to_cam(pts, calib)
            uv_all = np.full((len(pts), 2), np.nan)
            uv, _, mask = cam_to_image(cam, calib.P2, shape)
            uv_all[mask] = uv
            fov_pct = 100 * mask.mean()
            for o, m in objs:
                x1, y1, x2, y2 = o.bbox
                u, v = uv_all[m, 0], uv_all[m, 1]
                inb = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)   # NaN -> False
                rows.append(dict(dataset=Path(data_root).name, frame=fid, perturb=name,
                                 obj=o.type, dist_m=round(float(o.location[2]), 1),
                                 dist_bin=dist_bin(o.location[2]), n_obj_pts=int(m.sum()),
                                 in_box_pct=100 * inb.mean(), fov_pct=fov_pct))
    return pd.DataFrame(rows)


def main() -> None:
    ap = argparse.ArgumentParser(description="Sweep calibration drift, đo %% điểm object rơi đúng 2D box")
    ap.add_argument("--data-root", default="data/kitti_mini")
    ap.add_argument("--max-frames", type=int, default=20)
    ap.add_argument("--min-pts", type=int, default=20, help="bỏ object có ít hơn N điểm LiDAR")
    ap.add_argument("--out-dir", default="results")
    args = ap.parse_args()

    df = run(args.data_root, args.max_frames, args.min_pts)
    name = Path(args.data_root).name
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    df.to_csv(out / f"calib_drift_{name}_per_object.csv", index=False)
    order = [lv[0] for lv in LEVELS]
    summ = (df.groupby("perturb")
              .agg(n_obj=("in_box_pct", "size"), in_box_pct=("in_box_pct", "mean"),
                   fov_pct=("fov_pct", "mean"))
              .reindex(order).round(1))
    by_dist = df.pivot_table(index="perturb", columns="dist_bin", values="in_box_pct",
                             aggfunc="mean").reindex(order).round(1)
    summ = summ.join(by_dist)
    summ.to_csv(out / f"calib_drift_{name}.csv")
    print(summ.to_string())
    print(f"-> {out / f'calib_drift_{name}.csv'}")


if __name__ == "__main__":
    main()
