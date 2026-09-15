import matplotlib.pyplot as plt
import matplotlib.animation as animation
import json
import zmq
import collections 
import time # Them import time

# Cau hinh bieu do Matplotlib
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlabel("Thoi gian (s)")
ax.set_ylabel("Van toc (rad/s)")
ax.set_title("Bieu do van toc dong co thoi gian thuc")
ax.grid(True)

line_command, = ax.plot([], [], 'b-', label='Van toc lenh')
line_actual, = ax.plot([], [], 'r-', label='Van toc thuc te')
ax.legend()

max_points = 1000
times = collections.deque(maxlen=max_points)
command_velocities = collections.deque(maxlen=max_points)
actual_velocities = collections.deque(maxlen=max_points)

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5556") 
socket.setsockopt_string(zmq.SUBSCRIBE, "") 

# Luu tru thoi gian bat dau cua bieu do (tren host) de lam moc
# Dieu nay giup truc thoi gian cua bieu do luon bat dau tu 0 hoac duong
plot_start_time = time.time()

# Ham khoi tao (duoc goi mot lan khi animation bat dau)
def init_plot():
    line_command.set_data([], [])
    line_actual.set_data([], [])
    return line_command, line_actual

# Ham cap nhat bieu do
def update_plot(frame):
    new_data_received = False
    # Lay tat ca tin nhan co san trong buffer ZeroMQ de cap nhat nhanh nhat
    while True:
        try:
            message = socket.recv_string(flags=zmq.NOBLOCK) 
            data = json.loads(message)
            
            # Tinh thoi gian tuong doi so voi thoi gian bat dau cua bieu do tren host
            # Dieu nay giup truc thoi gian luon la duong va lien tuc
            relative_time = data['time'] 
            
            times.append(relative_time)
            command_velocities.append(data['command_velocity'])
            actual_velocities.append(data['actual_velocity'])
            new_data_received = True
            
        except zmq.Again:
            # Khong con tin nhan nao trong buffer, thoat vong lap nhan
            break
        except Exception as e:
            print(f"Loi khi nhan hoac parse du lieu: {e}")
            break

    # Chi cap nhat bieu do neu co du lieu moi
    if new_data_received and times: 
        # Cap nhat du lieu cho cac duong bieu do
        line_command.set_data(list(times), list(command_velocities))
        line_actual.set_data(list(times), list(actual_velocities))

        # Tu dong dieu chinh gioi han truc x va y
        # Dam bao truc X luon hien thi mot khoang thoi gian co dinh
        if len(times) > 1:
            # Dieu chinh xlim de hien thi max_points diem cuoi cung
            ax.set_xlim(times[0], times[-1])
        
        # Dieu chinh gioi han truc Y
        min_y = min(min(command_velocities), min(actual_velocities)) - 0.1
        max_y = max(max(command_velocities), max(actual_velocities)) + 0.1
        ax.set_ylim(min_y, max_y)
    
    return line_command, line_actual

# Tao doi tuong Animation
# interval: thoi gian cho giua cac lan goi ham update_plot (ms)
# blit=True: toi uu hoa viec ve, chi ve lai nhung phan thay doi.
#            Neu gap loi hien thi, thu dat blit=False.
ani = animation.FuncAnimation(fig, update_plot, init_func=init_plot, interval=20, blit=False) 

plt.show()