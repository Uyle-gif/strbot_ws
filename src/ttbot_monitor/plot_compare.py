#!/usr/bin/env python3
import argparse
import os
import pandas as pd
import matplotlib.pyplot as plt


def load_csv(path: str) -> pd.DataFrame:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Không tìm thấy file: {path}")
    return pd.read_csv(path)


def pick_reference_path(ref_path, mpc_ref, gmpc_ref):
    if ref_path is not None:
        return load_csv(ref_path)

    if os.path.exists(gmpc_ref):
        return load_csv(gmpc_ref)

    if os.path.exists(mpc_ref):
        return load_csv(mpc_ref)

    raise FileNotFoundError(
        "Không tìm thấy file reference. Hãy truyền --ref hoặc đảm bảo có mpc_ref.csv / gmpc_ref.csv"
    )


def main():
    parser = argparse.ArgumentParser(description="So sánh quỹ đạo MPC và GMPC trên cùng một hình")
    parser.add_argument("--mpc_traj", type=str, default="mpc_traj.csv")
    parser.add_argument("--gmpc_traj", type=str, default="gmpc_traj.csv")
    parser.add_argument("--mpc_ref", type=str, default="mpc_ref.csv")
    parser.add_argument("--gmpc_ref", type=str, default="gmpc_ref.csv")
    parser.add_argument("--ref", type=str, default=None)
    parser.add_argument("--output", type=str, default="mpc_vs_gmpc.png")
    parser.add_argument("--title", type=str, default="Trajectory Tracking Comparison")
    args = parser.parse_args()

    mpc = load_csv(args.mpc_traj)
    gmpc = load_csv(args.gmpc_traj)
    ref = pick_reference_path(args.ref, args.mpc_ref, args.gmpc_ref)

    required_cols = {"x", "y"}
    if not required_cols.issubset(mpc.columns):
        raise ValueError(f"{args.mpc_traj} phải có cột x,y")
    if not required_cols.issubset(gmpc.columns):
        raise ValueError(f"{args.gmpc_traj} phải có cột x,y")
    if not required_cols.issubset(ref.columns):
        raise ValueError("File reference phải có cột x,y")

    mpc_x = mpc["x"].to_numpy()
    mpc_y = mpc["y"].to_numpy()
    gmpc_x = gmpc["x"].to_numpy()
    gmpc_y = gmpc["y"].to_numpy()
    ref_x = ref["x"].to_numpy()
    ref_y = ref["y"].to_numpy()

    plt.figure(figsize=(8, 8))
    plt.plot(ref_x, ref_y, "g--", linewidth=2.0, label="Reference Path")
    plt.plot(mpc_x, mpc_y, "r-", linewidth=2.0, label="MPC")
    plt.plot(gmpc_x, gmpc_y, "b-", linewidth=2.0, label="GMPC")

    plt.scatter(ref_x[0], ref_y[0], c="k", marker="o", s=60, label="Start")
    plt.scatter(ref_x[-1], ref_y[-1], c="k", marker="x", s=60, label="End")

    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title(args.title)
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(args.output, dpi=300, bbox_inches="tight")
    print(f"Đã lưu hình so sánh: {args.output}")
    plt.show()


if __name__ == "__main__":
    main()