"""Vẽ biểu đồ kết quả sweep + ảnh failure case cho REPORT.

Chạy (sau calib_drift_sweep):  python -m src.make_figures
"""
from __future__ import annotations

from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from starter.datasets import load_frame
from starter.projection import cam_to_image, draw_box2d, overlay_points, perturb_extrinsic, velo_to_cam
from src.calib_drift_sweep import points_in_box3d

FIG = Path("results/figures")


def plot_sweep() -> None:
    df = pd.read_csv("results/calib_drift_kitti_mini.csv", index_col=0)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for ax, kind in zip(axes, ["yaw", "pitch"]):
        rows = ["none"] + [f"{kind}_{d}deg" for d in (0.5, 1.0, 2.0, 3.0)]
        x = [0, 0.5, 1, 2, 3]
        for col in ["near<15m", "mid15-30m", "far>30m", "in_box_pct"]:
            label = "tất cả object" if col == "in_box_pct" else col
            ax.plot(x, df.loc[rows, col], marker="o", lw=2.5 if col == "in_box_pct" else 1.5,
                    ls="-" if col == "in_box_pct" else "--", label=label)
        ax.set_title(f"KITTI mini: lệch {kind}")
        ax.set_xlabel(f"{kind} drift (độ)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("% điểm object rơi đúng 2D box")
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(FIG / "calib_drift_sweep.png", dpi=120)


def overlay_obj_points(fr, calib, color_box=(0, 255, 0)):
    """Vẽ toàn bộ điểm + 2D box GT với calib cho trước."""
    uv, depth, _ = cam_to_image(velo_to_cam(fr["points"][:, :3], calib), calib.P2, fr["image"].shape)
    vis = overlay_points(fr["image"], uv, depth)
    for o in fr["labels"]:
        if o.type != "DontCare":
            vis = draw_box2d(vis, o.bbox, color_box, o.type)
    return vis


def demo_drift() -> None:
    fr = load_frame("data/kitti_mini", "000011")
    a = overlay_obj_points(fr, fr["calib"])
    b = overlay_obj_points(fr, perturb_extrinsic(fr["calib"], yaw_deg=2.0))
    cv2.putText(a, "calib dung", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(b, "yaw lech 2 do", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.imwrite(str(FIG / "demo_000011_yaw0_vs_yaw2.png"), np.vstack([a, b]))


def failure_truncated() -> None:
    """Car bị cắt ở mép trái ảnh (truncated=0.98): calib ĐÚNG nhưng metric báo chỉ 6% điểm trong box."""
    fr = load_frame("data/kitti_mini", "000011")
    cam = velo_to_cam(fr["points"][:, :3], fr["calib"])
    car = [o for o in fr["labels"] if o.type == "Car" and o.truncated > 0.9][0]
    m = points_in_box3d(cam, car)
    uv, depth, mask = cam_to_image(cam[m], fr["calib"].P2, fr["image"].shape)
    vis = overlay_points(fr["image"], uv, depth, radius=2)
    vis = draw_box2d(vis, car.bbox, (0, 255, 0), f"Car trunc={car.truncated:.2f}")
    x1, y1, x2, y2 = car.bbox
    inb = (uv[:, 0] >= x1) & (uv[:, 0] <= x2) & (uv[:, 1] >= y1) & (uv[:, 1] <= y2)
    txt = [f"diem LiDAR trong 3D box: {m.sum()}",
           f"chieu vao duoc anh: {mask.sum()} ({100 * mask.mean():.0f}%)",
           f"roi dung 2D box: {inb.sum()} ({100 * inb.sum() / m.sum():.1f}%) -> metric bao SAI"]
    cv2.rectangle(vis, (410, 5), (1000, 100), (0, 0, 0), -1)   # nền đen cho chữ dễ đọc
    for i, t in enumerate(txt):
        cv2.putText(vis, t, (420, 30 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imwrite(str(FIG / "fail_01_truncated_car_metric.png"), vis)
    print("\n".join(txt))


if __name__ == "__main__":
    FIG.mkdir(parents=True, exist_ok=True)
    plot_sweep()
    demo_drift()
    failure_truncated()
    print(f"-> {FIG}")
