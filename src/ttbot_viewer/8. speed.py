import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time
from collections import deque

class StepResponseGenerator(Node):
    def __init__(self):
        super().__init__('step_response_generator')
        
        # 1. PUBLISHER & SUBSCRIBERS
        self.cmd_pub = self.create_publisher(TwistStamped, '/ackermann_controller/cmd_vel', 10)
        
        # Subscribe cả 2 motor
        self.fb1_sub = self.create_subscription(Float32, '/motor1_feedback', self.fb1_callback, 10)
        self.fb2_sub = self.create_subscription(Float32, '/motor2_feedback', self.fb2_callback, 10)
        
        # --- CẤU HÌNH KỊCH BẢN TEST ---
        self.test_steps = [0.0, 0.5, 0.7, 1.5, 1.3, 0.5, 0.0, -0.7, -0.3, 0.0]
        self.step_duration = 5.0 
        # ------------------------------

        self.start_time = time.time()
        
        # Bộ nhớ đệm (Window ~60s)
        self.t_data = deque(maxlen=3000)
        self.sp_data = deque(maxlen=3000)
        self.fb1_data = deque(maxlen=3000) # Data Motor 1
        self.fb2_data = deque(maxlen=3000) # Data Motor 2
        
        self.current_sp = 0.0
        self.current_fb1 = 0.0
        self.current_fb2 = 0.0
        
        print(f"--> BẮT ĐẦU TEST (Window: 60s)")

    def fb1_callback(self, msg):
        self.current_fb1 = msg.data

    def fb2_callback(self, msg):
        self.current_fb2 = msg.data

    def update_control_loop(self):
        t_now = time.time() - self.start_time
        
        # Logic tính Setpoint
        step_idx = int((t_now / self.step_duration) % len(self.test_steps))
        self.current_sp = self.test_steps[step_idx]
        
        # Gửi lệnh
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(self.current_sp)
        msg.twist.angular.z = 0.0 
        self.cmd_pub.publish(msg)
        
        # Lưu dữ liệu
        self.t_data.append(t_now)
        self.sp_data.append(self.current_sp)
        self.fb1_data.append(self.current_fb1)
        self.fb2_data.append(self.current_fb2)

def animate(i, generator, line_sp, line_fb1, line_fb2, ax):
    generator.update_control_loop()
    
    # Cập nhật dữ liệu cho các đường
    line_sp.set_data(generator.t_data, generator.sp_data)
    line_fb1.set_data(generator.t_data, generator.fb1_data)
    line_fb2.set_data(generator.t_data, generator.fb2_data)
    
    if len(generator.t_data) > 0:
        current_t = generator.t_data[-1]
        
        # Logic trục X hiển thị 60s
        WINDOW_SIZE = 60.0
        if current_t < WINDOW_SIZE:
            ax.set_xlim(0, WINDOW_SIZE)
        else:
            ax.set_xlim(current_t - WINDOW_SIZE, current_t)
        
        # Scale trục Y (tính toán dựa trên cả SP, FB1 và FB2)
        all_y = list(generator.sp_data) + list(generator.fb1_data) + list(generator.fb2_data)
        if all_y:
            curr_min = min(all_y)
            curr_max = max(all_y)
            y_min = min(0, curr_min)
            y_max = max(0, curr_max)
            span = y_max - y_min
            if span == 0: span = 1.0
            margin = span * 0.1
            ax.set_ylim(y_min - margin, y_max + margin)
            
    return line_sp, line_fb1, line_fb2

def main():
    rclpy.init()
    generator = StepResponseGenerator()
    thread = threading.Thread(target=rclpy.spin, args=(generator,), daemon=True)
    thread.start()

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlabel("Thời gian (s)")
    ax.set_ylabel("Vận tốc (m/s)")
    ax.set_title("Step Response: Setpoint vs Motor 1 & Motor 2")
    ax.grid(True, linestyle='--', alpha=0.6)
    
    # Khởi tạo các đường plot với màu sắc khác nhau
    line_sp, = ax.plot([], [], 'r--', label='Setpoint', linewidth=1.0)        # Đỏ nét đứt
    line_fb1, = ax.plot([], [], 'g-', label='Motor Left', linewidth=0.8) # Xanh lá
    line_fb2, = ax.plot([], [], 'b-', label='Motor Right', linewidth=0.8) # Xanh dương
    
    ax.legend(loc='upper left')

    ani = FuncAnimation(fig, animate, fargs=(generator, line_sp, line_fb1, line_fb2, ax), interval=30)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = TwistStamped()
        generator.cmd_pub.publish(stop_msg)
        generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()