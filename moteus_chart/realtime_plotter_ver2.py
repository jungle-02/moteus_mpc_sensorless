import matplotlib.pyplot as plt
import matplotlib.animation as animation
import json
import zmq
import collections 
import time 
import math

# --- Cau hinh bieu do Matplotlib ---
fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(10, 8), sharex=True) # 2 subplot, chia se truc x

# Cau hinh subplot VI TRI
ax_pos.set_ylabel("Vi tri (rad)")
ax_pos.set_title("Bieu do dieu khien vi tri dong co thoi gian thuc")
ax_pos.grid(True)
line_command_pos, = ax_pos.plot([], [], 'b-', label='Vi tri lenh')
line_actual_pos, = ax_pos.plot([], [], 'r-', label='Vi tri thuc te')
ax_pos.legend()
ax_pos.set_ylim(-2 * math.pi * 1.1, 2 * math.pi * 1.1) # Gioi han Y ban dau cho vi tri

# Cau hinh subplot VAN TOC
ax_vel.set_xlabel("Thoi gian (s)")
ax_vel.set_ylabel("Van toc (rad/s)")
ax_vel.set_title("Bieu do van toc dong co thoi gian thuc") # Tieu de rieng cho subplot van toc
ax_vel.grid(True)
line_command_vel, = ax_vel.plot([], [], 'm-', label='Van toc lenh')
line_actual_vel, = ax_vel.plot([], [], 'g-', label='Van toc thuc te')
ax_vel.legend()
ax_vel.set_ylim(-5, 5) # Gioi han Y ban dau cho van toc, dieu chinh theo van toc du kien

# --- Cau hinh ZeroMQ ---
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5556") 
socket.setsockopt_string(zmq.SUBSCRIBE, "") 

# --- Cau hinh bo dem du lieu ---
max_points = 1000 # So diem toi da tren bieu do
times = collections.deque(maxlen=max_points)
command_positions = collections.deque(maxlen=max_points)
actual_positions = collections.deque(maxlen=max_points)
command_velocities = collections.deque(maxlen=max_points)
actual_velocities = collections.deque(maxlen=max_points)

# Bien de theo doi trang thai hien thi cua cac duong
is_data_available = False

# --- Ham khoi tao (duoc goi mot lan khi animation bat dau) ---
def init_plot():
    # An tat ca cac duong khi khoi tao
    line_command_pos.set_data([], [])
    line_actual_pos.set_data([], [])
    line_command_vel.set_data([], [])
    line_actual_vel.set_data([], [])
    
    # Tra ve tat ca cac doi tuong Line ma animation se quan ly
    return line_command_pos, line_actual_pos, line_command_vel,line_actual_vel

# --- Ham cap nhat bieu do ---
def update_plot(frame):
    global is_data_available
    new_data_received_in_frame = False
    
    while True:
        try:
            message = socket.recv_string(flags=zmq.NOBLOCK) 
            data = json.loads(message)
            
            # Cap nhat cac hang doi du lieu
            times.append(data['time'])
            command_positions.append(data['command_position'])
            actual_positions.append(data['actual_position'])
            command_velocities.append(data['command_velocity'])
            actual_velocities.append(data['actual_velocity'])
            new_data_received_in_frame = True
            
        except zmq.Again:
            # Khong con tin nhan nao trong buffer
            break
        except json.JSONDecodeError as e:
            print(f"Loi phan tich JSON: {e} - Tin nhan: {message}")
            break
        except KeyError as e:
            print(f"Loi KeyError: Thieu khoa '{e}' trong du lieu - Du lieu: {data}")
            break
        except Exception as e:
            print(f"Loi khong xac dinh khi nhan hoac parse du lieu: {e}")
            break

    # Cap nhat trang thai 'is_data_available'
    # Neu co du lieu moi trong frame nay HOAC da co du lieu tu truoc
    if new_data_received_in_frame or len(times) > 0:
        is_data_available = True
    else:
        is_data_available = False

    if is_data_available and times:
        # Hien thi cac duong neu co du lieu
        line_command_pos.set_visible(True)
        line_actual_pos.set_visible(True)
        line_command_vel.set_visible(True)
        line_actual_vel.set_visible(True)

        # Cap nhat du lieu cho cac duong bieu do VI TRI
        line_command_pos.set_data(list(times), list(command_positions))
        line_actual_pos.set_data(list(times), list(actual_positions))

        # Cap nhat du lieu cho duong bieu do VAN TOC
        line_command_vel.set_data(list(times), list(command_velocities))
        line_actual_vel.set_data(list(times), list(actual_velocities))

        # Tu dong dieu chinh gioi han truc x (chia se giua 2 subplot)
        if len(times) > 1:
            ax_pos.set_xlim(times[0], times[-1])
        
        # Tu dong dieu chinh gioi han truc Y cho VI TRI
        # Chi tu dong dieu chinh neu co du du lieu, neu khong giu nguyen gioi han ban dau
        if len(actual_positions) > 0:
            min_pos_y = min(min(command_positions), min(actual_positions)) - 0.1
            max_pos_y = max(max(command_positions), max(actual_positions)) + 0.1
            ax_pos.set_ylim(min_pos_y, max_pos_y)
        
        # Tu dong dieu chinh gioi han truc Y cho VAN TOC
        if len(actual_velocities) > 0:
            min_vel_y = min(min(command_velocities), min(actual_velocities)) - 0.1
            max_vel_y = max(max(command_velocities), max(actual_velocities)) + 0.1
            ax_vel.set_ylim(min_vel_y, max_vel_y)
    else:
        # An tat ca cac duong neu khong co du lieu
        line_command_pos.set_visible(False)
        line_actual_pos.set_visible(False)
        line_command_vel.set_visible(False)
        line_actual_vel.set_visible(False)
        
    # Tra ve tat ca cac doi tuong Line ma animation se quan ly
    return line_command_pos, line_actual_pos, line_command_vel,line_actual_vel

# Tao doi tuong Animation
ani = animation.FuncAnimation(fig, update_plot, init_func=init_plot, interval=20, blit=True) 

plt.tight_layout() # Tu dong dieu chinh khoang cach giua cac subplot
plt.show()