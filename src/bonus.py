"""Bonus B1 + B3.

B1: So sánh 2 cấu hình metric trên cùng dữ liệu, cùng mức lệch yaw:
    v1 = % TẤT CẢ điểm của object rơi vào 2D box (metric gốc, bị lỗi với xe bị cắt mép ảnh)
    v2 = chỉ xét object truncated < 0.5 và chỉ tính trên điểm đã chiếu vào được ảnh (sửa failure case)
B3: Đo latency projection (velo_to_cam + cam_to_image) trên 1 frame KITTI:
    bỏ 5 lần chạy đầu (warm-up), đo 50 lần, báo p50/p95.

Chạy:  python -m src.bonus
"""
from __future__ import annotations

import platform
import time
from pathlib import Path

import numpy as np
import pandas as pd

from starter.datasets import list_frames, load_frame
from starter.projection import cam_to_image, perturb_extrinsic, project_velo_to_image, velo_to_cam
from src.calib_drift_sweep import CLASSES, points_in_box3d

YAWS = [0.0, 0.5, 1.0, 2.0, 3.0]


def compare_metrics(data_root: str = "data/kitti_mini") -> pd.DataFrame:
    rows = []
    for fid in list_frames(data_root):
        fr = load_frame(data_root, fid)
        pts = fr["points"][:, :3]
        cam_true = velo_to_cam(pts, fr["calib"])
        objs = [(o, points_in_box3d(cam_true, o)) for o in fr["labels"]
                if o.type in CLASSES and o.location[2] > 0]
        objs = [(o, m) for o, m in objs if m.sum() >= 20]
        for yaw in YAWS:
            calib = perturb_extrinsic(fr["calib"], yaw_deg=yaw)
            uv_all = np.full((len(pts), 2), np.nan)
            uv, _, mask = cam_to_image(velo_to_cam(pts, calib), calib.P2, fr["image"].shape)
            uv_all[mask] = uv
            for o, m in objs:
                x1, y1, x2, y2 = o.bbox
                u, v = uv_all[m, 0], uv_all[m, 1]
                inb = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
                in_img = mask[m]
                rows.append(dict(frame=fid, yaw_deg=yaw, trunc=o.truncated,
                                 v1=100 * inb.mean(),
                                 v2=100 * inb[in_img].mean() if in_img.sum() >= 10 else np.nan))
    df = pd.DataFrame(rows)
    v2_df = df[df.trunc < 0.5]
    out = pd.DataFrame({
        "v1_mean_pct": df.groupby("yaw_deg").v1.mean(),
        "v1_n_obj": df.groupby("yaw_deg").v1.size(),
        "v1_obj_below_85pct": df.assign(b=df.v1 < 85).groupby("yaw_deg").b.mean() * 100,
        "v2_mean_pct": v2_df.groupby("yaw_deg").v2.mean(),
        "v2_n_obj": v2_df.groupby("yaw_deg").v2.count(),
        "v2_obj_below_85pct": v2_df.assign(b=v2_df.v2 < 85).groupby("yaw_deg").b.mean() * 100,
    }).round(1)
    return out


def latency(data_root: str = "data/kitti_mini", frame: str = "000011",
            warmup: int = 5, runs: int = 50) -> pd.DataFrame:
    fr = load_frame(data_root, frame)
    times = []
    for i in range(warmup + runs):
        t0 = time.perf_counter()
        project_velo_to_image(fr["points"], fr["calib"], fr["image"].shape)
        dt = (time.perf_counter() - t0) * 1000
        if i >= warmup:
            times.append(dt)
    t = np.array(times)
    return pd.DataFrame([dict(frame=frame, n_points=len(fr["points"]), runs=runs, warmup_dropped=warmup,
                              p50_ms=round(float(np.percentile(t, 50)), 2),
                              p95_ms=round(float(np.percentile(t, 95)), 2),
                              mean_ms=round(float(t.mean()), 2),
                              cpu=platform.processor() or platform.machine(),
                              python=platform.python_version(), os=platform.platform())])


if __name__ == "__main__":
    Path("results").mkdir(exist_ok=True)
    cmp = compare_metrics()
    cmp.to_csv("results/bonus_b1_metric_v1_vs_v2.csv")
    print(cmp.to_string())
    lat = latency()
    lat.to_csv("results/bonus_b3_latency.csv", index=False)
    print(lat.T.to_string())
