import sys
import time
import math
import json
import zmq

def run_bell_velocity_test():
    # --- 1. Thiet lap ZeroMQ Publisher (Ket noi voi Backend) ---
    context = zmq.Context()
    pub_socket = context.socket(zmq.PUB)
    pub_socket.connect("tcp://localhost:5557")

    time.sleep(0.5) # Doi ZMQ thiet lap handshake

    # Chi gui 1 log duy nhat bao hieu script da chay thanh cong
    start_msg = "OK. Bell shape script loaded."
    pub_socket.send_string(json.dumps({"type": "log", "msg": start_msg}))
    print(f">> {start_msg}")

    # --- 2. Cau hinh tham so quy dao hinh chuong (Raised Cosine) ---
    V_max = 30.0     # Dinh van toc (rad/s)
    T_move = 5.0     # Thoi gian de thuc hien xong 1 nhip hinh chuong (s)
    dt = 0.01        # Chu ky dieu khien (s) - 100Hz
    
    # --- 3. Vong lap dieu khien voi thoi gian ly tuong ---
    t_simulated = 0.0     
    
    while True:
        loop_start_time = time.time() # Ghi lai thoi gian bat dau vong lap thuc te
        
        # Tang bien thoi gian "ly tuong" len dt
        t_simulated += dt 
        
        # --- TINH TOAN VAN TOC HINH CHUONG ---
        # Neu van nam trong khoang thoi gian thuc hien (0 den T_move)
        if t_simulated <= T_move:
            velocity_command = (V_max / 2.0) * (1.0 - math.cos((2 * math.pi * t_simulated) / T_move))
        else:
            # Sau khi di het hinh chuong thi giu van toc dung lai o 0 (giong tren do thi)
            velocity_command = 0.0 
        
        # Gui lenh dieu khien qua Backend
        torque_limit = 1.0 
        cmd_str = f"d pos nan {velocity_command:.4f} {torque_limit:.2f}"
        
        command_payload = {
            "type": "raw",
            "cmd": cmd_str,
            "silent": True
        }
        pub_socket.send_string(json.dumps(command_payload))

        # Tinh toan thoi gian can ngu de duy tri chu ky dt chinh xac
        time_spent_in_loop = time.time() - loop_start_time
        sleep_duration = dt - time_spent_in_loop

        if sleep_duration > 0:
            time.sleep(sleep_duration)

if __name__ == "__main__":
    try:
        run_bell_velocity_test()
    except KeyboardInterrupt:
        print("Script manually interrupted.")
        ctx = zmq.Context()
        sock = ctx.socket(zmq.PUB)
        sock.connect("tcp://localhost:5557")
        # Gui lenh dung dong co khi nhan Stop Script hoac Ctrl+C
        sock.send_string(json.dumps({"type": "d_stop"}))
        sock.send_string(json.dumps({"type": "log", "msg": "Bell script stopped."}))
        sock.close()
        ctx.term()
        sys.exit(0)