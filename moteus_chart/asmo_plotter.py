import matplotlib.pyplot as plt
import matplotlib.animation as animation
import json
import zmq
import collections 
import time 
import math

# --- CẤU HÌNH BIỂU ĐỒ (3 Subplots) ---
# Tạo 3 đồ thị xếp dọc: Position, Velocity, Error
fig, (ax_pos, ax_vel, ax_err) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

# 1. Subplot VỊ TRÍ (Position)
ax_pos.set_ylabel("Position (rad)")
ax_pos.set_title("So sánh: ASMO vs Encoder")
ax_pos.grid(True)
line_pos_ref, = ax_pos.plot([], [], 'k--', label='Ref (Lệnh)', alpha=0.6)
line_pos_act, = ax_pos.plot([], [], 'r-', label='Act (Thực tế)')
line_pos_est, = ax_pos.plot([], [], 'b:', label='Est (Ước lượng)', linewidth=2)
ax_pos.legend(loc='upper right', fontsize='small')

# 2. Subplot VẬN TỐC (Velocity)
ax_vel.set_ylabel("Velocity (rad/s)")
ax_vel.grid(True)
line_vel_ref, = ax_vel.plot([], [], 'k--', label='Ref', alpha=0.6)
line_vel_act, = ax_vel.plot([], [], 'g-', label='Act')
line_vel_est, = ax_vel.plot([], [], 'm:', label='Est', linewidth=2)
ax_vel.legend(loc='upper right', fontsize='small')

# 3. Subplot SAI SỐ (Error = Est - Act) [MỚI THÊM]
ax_err.set_ylabel("Error")
ax_err.set_xlabel("Time (s)")
ax_err.set_title("Sai số ước lượng (Est - Act)")
ax_err.grid(True)
# Sai số vị trí (màu cam)
line_err_pos, = ax_err.plot([], [], 'orange', label='Pos Error (rad)')
# Sai số vận tốc (màu tím)
line_err_vel, = ax_err.plot([], [], 'purple', label='Vel Error (rad/s)', alpha=0.7)
ax_err.legend(loc='upper right', fontsize='small')

# --- CẤU HÌNH ZMQ ---
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5556") 
socket.setsockopt_string(zmq.SUBSCRIBE, "") 

# --- BỘ ĐỆM DỮ LIỆU ---
max_points = 500  # Giảm bớt số điểm để vẽ mượt hơn nếu cần
times = collections.deque(maxlen=max_points)

# Dữ liệu gốc
pos_ref = collections.deque(maxlen=max_points)
pos_act = collections.deque(maxlen=max_points)
pos_est = collections.deque(maxlen=max_points)

vel_ref = collections.deque(maxlen=max_points)
vel_act = collections.deque(maxlen=max_points)
vel_est = collections.deque(maxlen=max_points)

# Dữ liệu sai số (Error)
err_pos = collections.deque(maxlen=max_points)
err_vel = collections.deque(maxlen=max_points)

def init_plot():
    lines = [line_pos_ref, line_pos_act, line_pos_est, 
             line_vel_ref, line_vel_act, line_vel_est,
             line_err_pos, line_err_vel]
    for line in lines:
        line.set_data([], [])
    return lines

def update_plot(frame):
    has_new_data = False
    
    # Đọc hết hàng đợi ZMQ
    while True:
        try:
            message = socket.recv_string(flags=zmq.NOBLOCK)
            data = json.loads(message)
            
            t = data.get('time', 0)
            
            # Lấy dữ liệu
            p_ref = data.get('command_position', 0)
            p_act = data.get('actual_position', 0)
            p_est = data.get('asmo_theta_hat', 0)
            
            v_ref = data.get('command_velocity', 0)
            v_act = data.get('actual_velocity', 0)
            v_est = data.get('asmo_w_hat', 0)
            
            # Tính sai số (Error = Estimated - Actual)
            e_p = p_est - p_act
            e_v = v_est - v_act

            # Append vào deque
            times.append(t)
            pos_ref.append(p_ref)
            pos_act.append(p_act)
            pos_est.append(p_est)
            
            vel_ref.append(v_ref)
            vel_act.append(v_act)
            vel_est.append(v_est)
            
            err_pos.append(e_p)
            err_vel.append(e_v)
            
            has_new_data = True
            
        except zmq.Again:
            break
        except Exception as e:
            print(f"Error: {e}")
            break

    if has_new_data and len(times) > 1:
        # Cập nhật dữ liệu cho các đường vẽ
        line_pos_ref.set_data(times, pos_ref)
        line_pos_act.set_data(times, pos_act)
        line_pos_est.set_data(times, pos_est)
        
        line_vel_ref.set_data(times, vel_ref)
        line_vel_act.set_data(times, vel_act)
        line_vel_est.set_data(times, vel_est)
        
        line_err_pos.set_data(times, err_pos)
        line_err_vel.set_data(times, err_vel)

        # Autoscale trục X
        ax_pos.set_xlim(times[0], times[-1])
        
        # Autoscale trục Y (Pos)
        y_min, y_max = min(pos_act), max(pos_act)
        margin = (y_max - y_min) * 0.1 if y_max != y_min else 1.0
        ax_pos.set_ylim(y_min - margin, y_max + margin)

        # Autoscale trục Y (Vel)
        v_min, v_max = min(vel_act), max(vel_act)
        margin_v = (v_max - v_min) * 0.1 if v_max != v_min else 1.0
        ax_vel.set_ylim(v_min - margin_v, v_max + margin_v)
        
        # Autoscale trục Y (Error)
        # Gộp cả 2 list sai số để tìm min/max chung
        all_err = list(err_pos) + list(err_vel)
        if all_err:
            e_min, e_max = min(all_err), max(all_err)
            margin_e = (e_max - e_min) * 0.1 if e_max != e_min else 0.5
            ax_err.set_ylim(e_min - margin_e, e_max + margin_e)

    return (line_pos_ref, line_pos_act, line_pos_est, 
            line_vel_ref, line_vel_act, line_vel_est,
            line_err_pos, line_err_vel)

# Chạy animation (interval=30ms ~ 33fps)
ani = animation.FuncAnimation(fig, update_plot, init_func=init_plot, 
                              interval=30, blit=True)

plt.tight_layout()
plt.show()