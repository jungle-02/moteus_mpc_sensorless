import matplotlib.pyplot as plt
import matplotlib.animation as animation
import json
import zmq
import collections 
import time

# --- CAU HINH ---
ZMQ_HOST = "localhost" # IP cua Docker (thuong la localhost neu map port)
ZMQ_PORT = 5556
MAX_POINTS = 500       # So diem hien thi tren do thi
# ----------------

# Cau hinh bieu do Matplotlib
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlabel("Thoi gian (s)")
ax.set_ylabel("Dien tro Uoc luong (Ohm)")
ax.set_title("Nhan dang tham so RLS thoi gian thuc (R_hat)")
ax.grid(True)

# Duong bieu dien R_hat
line_r, = ax.plot([], [], 'g-', linewidth=2, label='Estimated R (Ohm)')
ax.legend()

# Text hien thi gia tri hien tai
text_info = ax.text(0.02, 0.95, "", transform=ax.transAxes, 
                    bbox=dict(facecolor='white', alpha=0.8))

# Bo dem du lieu (Rolling window)
times = collections.deque(maxlen=MAX_POINTS)
r_values = collections.deque(maxlen=MAX_POINTS)

# Ket noi ZeroMQ Subscriber
print(f"Connecting to {ZMQ_HOST}:{ZMQ_PORT}...")
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect(f"tcp://{ZMQ_HOST}:{ZMQ_PORT}") 
socket.setsockopt_string(zmq.SUBSCRIBE, "") # Dang ky nhan tat ca thong tin

def init_plot():
    line_r.set_data([], [])
    return line_r, text_info

def update_plot(frame):
    new_data_received = False
    
    # Doc HET du lieu trong buffer de tranh tre (lag)
    while True:
        try:
            message = socket.recv_string(flags=zmq.NOBLOCK)
            data = json.loads(message)
            
            times.append(data['time'])
            r_values.append(data['r_hat'])
            
            # Cap nhat text hien thi nhiet do neu co
            temp = data.get('temp', 0)
            text_info.set_text(f"R_hat: {data['r_hat']:.4f} Ω\nTemp: {temp:.1f} °C")
            
            new_data_received = True
        except zmq.Again:
            break # Het du lieu
        except Exception as e:
            print(f"Loi nhan du lieu: {e}")
            break

    if new_data_received and times:
        line_r.set_data(list(times), list(r_values))

        # Tu dong chinh truc X (truot theo thoi gian)
        ax.set_xlim(times[0], times[-1] + 1)
        
        # Tu dong chinh truc Y (Zoom vao gia tri R)
        if len(r_values) > 10:
            current_r = r_values[-1]
            if current_r > 0.01:
                # Zoom +/- 20% quanh gia tri hien tai
                min_y = min(list(r_values)[-100:]) * 0.95
                max_y = max(list(r_values)[-100:]) * 1.05
                # Gioi han toi thieu de khong bi scale qua nho
                ax.set_ylim(min_y, max_y)
            else:
                ax.set_ylim(0, 0.1) # Mac dinh neu R=0

    return line_r, text_info

# Chay animation
print("Dang ve do thi... Hay chay Docker script!")
ani = animation.FuncAnimation(fig, update_plot, init_func=init_plot, interval=50, blit=False) 
plt.show()