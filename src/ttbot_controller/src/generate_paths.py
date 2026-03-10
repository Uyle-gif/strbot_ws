import math
import csv

def save_to_csv(filename, points):
    """Hàm lưu danh sách tọa độ vào file CSV"""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y'])
        writer.writerows(points)
    print(f"Đã tạo: {filename} ({len(points)} điểm).")

def generate_circle(r=6.0, cx=6.0, cy=0.0, step_m=0.1):
    """Tạo hình tròn bắt đầu tại (0,0), scale 1.5 lần"""
    points = []
    circumference = 2 * math.pi * r
    total_steps = int(circumference / step_m)
    active_steps = int(total_steps * 0.98) # Bỏ điểm cuối để tránh dính đầu
    
    for i in range(active_steps):
        theta = math.pi + (i / total_steps) * 2 * math.pi
        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)
        points.append([round(x, 4), round(y, 4)])
    return points

def generate_rounded_square(side=9.0, radius=1.0, step_m=0.1):
    """Tạo hình vuông bo góc, scale 1.5 lần"""
    points = []
    # Chiều dài đoạn thẳng giữa 2 góc bo
    straight_len = side - 2 * radius
    steps_straight = int(straight_len / step_m)
    steps_arc = int((math.pi * radius / 2) / step_m)

    # 1. Cạnh đáy (Phải)
    for i in range(steps_straight):
        points.append([radius + i * step_m, 0.0])
    # Góc bo 1 (Dưới - Phải)
    for i in range(steps_arc):
        a = -math.pi/2 + (i/steps_arc) * (math.pi/2)
        points.append([side - radius + radius*math.cos(a), radius + radius*math.sin(a)])
    
    # 2. Cạnh phải (Lên)
    for i in range(steps_straight):
        points.append([side, radius + i * step_m])
    # Góc bo 2 (Trên - Phải)
    for i in range(steps_arc):
        a = 0 + (i/steps_arc) * (math.pi/2)
        points.append([side - radius + radius*math.cos(a), side - radius + radius*math.sin(a)])

    # 3. Cạnh trên (Trái)
    for i in range(steps_straight):
        points.append([side - radius - i * step_m, side])
    # Góc bo 3 (Trên - Trái)
    for i in range(steps_arc):
        a = math.pi/2 + (i/steps_arc) * (math.pi/2)
        points.append([radius + radius*math.cos(a), side - radius + radius*math.sin(a)])

    # 4. Cạnh trái (Xuống)
    for i in range(steps_straight):
        points.append([0.0, side - radius - i * step_m])
    # Góc bo 4 (Dưới - Trái) - Cắt sớm để tách điểm đầu cuối
    for i in range(int(steps_arc * 0.8)):
        a = math.pi + (i/steps_arc) * (math.pi/2)
        points.append([radius + radius*math.cos(a), radius + radius*math.sin(a)])

    return [[round(p[0], 4), round(p[1], 4)] for p in points]

def generate_figure_8(scale=9.0, step_rad=0.05):
    """Tạo hình số 8, scale 1.5 lần"""
    points = []
    t = 0.0
    while t <= (2 * math.pi - 0.12):
        x = scale * math.sin(t)
        y = scale * math.sin(t) * math.cos(t)
        points.append([round(x, 4), round(y, 4)])
        t += step_rad
    return points

if __name__ == "__main__":
    # Đã scale 1.5 lần so với mặc định ban đầu
    save_to_csv("path_circle.csv", generate_circle(r=6.0, cx=6.0))
    # Hình vuông cạnh 9m, bán kính bo góc 1.5m (1.5m giúp GMPC ôm cua mượt nhất)
    save_to_csv("path_square.csv", generate_rounded_square(side=9.0, radius=1.5))
    save_to_csv("path_8.csv", generate_figure_8(scale=9.0))