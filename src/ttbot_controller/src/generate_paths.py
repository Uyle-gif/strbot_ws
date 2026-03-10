import math
import csv

def save_to_csv(filename, points):
    """Hàm lưu danh sách tọa độ vào file CSV"""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y'])
        writer.writerows(points)
    print(f"Đã tạo: {filename} ({len(points)} điểm).")
    print(f"  -> Điểm đầu: {points[0]}")
    print(f"  -> Điểm cuối: {points[-1]}")

def generate_square(side=13.5, start_x=0.0, start_y=0.0, step_m=0.1):
    """Tạo hình vuông: Đi ngang sang phải trước (Scale 1.5 lần nữa = 13.5m)"""
    points = []
    num_steps = int(side / step_m)
    
    # Cạnh 1: X tăng (Đi ngang)
    for i in range(num_steps):
        points.append([start_x + i * step_m, start_y])
        
    # Cạnh 2: Y tăng (Đi lên)
    for i in range(num_steps):
        points.append([start_x + side, start_y + i * step_m])
        
    # Cạnh 3: X giảm (Sang trái)
    for i in range(num_steps):
        points.append([start_x + side - i * step_m, start_y + side])
        
    # Cạnh 4: Y giảm (Đi xuống) - Ngắt sớm để tránh dính điểm đầu
    for i in range(num_steps - 5):
        points.append([start_x, start_y + side - i * step_m])
        
    return [[round(p[0], 4), round(p[1], 4)] for p in points]

def generate_circle(r=9.0, cx=9.0, cy=0.0, step_m=0.1):
    """Tạo hình tròn: Bắt đầu tại (0,0) (Scale 1.5 lần nữa R=9.0m)"""
    points = []
    circumference = 2 * math.pi * r
    total_steps = int(circumference / step_m)
    active_steps = int(total_steps * 0.98) 
    
    for i in range(active_steps):
        theta = math.pi + (i / total_steps) * 2 * math.pi
        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)
        points.append([round(x, 4), round(y, 4)])
    return points

def generate_figure_8(scale=13.5, step_rad=0.05):
    """Hình số 8: Bắt đầu từ (0,0) (Scale 1.5 lần nữa = 13.5m)"""
    points = []
    t = 0.0
    while t <= (2 * math.pi - 0.12):
        x = scale * math.sin(t)
        y = scale * math.sin(t) * math.cos(t)
        points.append([round(x, 4), round(y, 4)])
        t += step_rad
    return points

if __name__ == "__main__":
    # Cập nhật kích thước scale lên 1.5 lần so với file cũ
    save_to_csv("path_square.csv", generate_square(side=13.5))
    save_to_csv("path_circle.csv", generate_circle(r=9.0, cx=9.0))
    save_to_csv("path_8.csv", generate_figure_8(scale=13.5))