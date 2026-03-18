#!/usr/bin/env python3
import matplotlib.pyplot as plt
import csv
import os

def read_ref_csv(filename):
    x_vals, y_vals = [], []
    if not os.path.exists(filename):
        print(f"Không tìm thấy file: {filename}")
        return x_vals, y_vals
        
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        next(reader, None) 
        for row in reader:
            if not row: continue
            x_vals.append(float(row[0]))
            y_vals.append(float(row[1]))
    return x_vals, y_vals

def read_traj_csv(filename):
    x_vals, y_vals = [], []
    if not os.path.exists(filename):
        print(f"Không tìm thấy file: {filename}")
        return x_vals, y_vals
        
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if not row: continue
            x_vals.append(float(row[1]))
            y_vals.append(float(row[2]))
    return x_vals, y_vals

def main():
    # SỬA TÊN FILE Ở ĐÂY CHO KHỚP VỚI FILE CỦA BẠN
    ref_file = "mpc_ref_circle.csv"
    traj_file = "mpc_traj_circle.csv"

    ref_x, ref_y = read_ref_csv(ref_file)
    traj_x, traj_y = read_traj_csv(traj_file)

    fig, ax = plt.subplots(figsize=(8, 9)) # Chỉnh form đồ thị hơi chữ nhật đứng một chút (12x14)

    if ref_x and ref_y:
        ax.plot(ref_y, ref_x, 'k--', linewidth=1.5, label='Reference Path')
        ax.plot(ref_y[0], ref_x[0], 'ko', markerfacecolor='none', markersize=8, markeredgewidth=1.5, label='Start')
        ax.plot(ref_y[-1], ref_x[-1], 'kx', markersize=10, markeredgewidth=2, label='End')

    if traj_x and traj_y:
        ax.plot(traj_y, traj