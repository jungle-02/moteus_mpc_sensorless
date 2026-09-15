import sys
import time
import math
import json
import zmq
import random

def main():
    print("Running random position control with stops... (Ctrl+C to stop)")

    # --- 1. Thiet lap ZeroMQ ---
    context = zmq.Context()
    
    # Socket PUB de gui lenh DIEU KHIEN (Port 5557)
    pub_socket = context.socket(zmq.PUB)
    pub_socket.connect("tcp://localhost:5557")
    
    # Socket SUB de DOC phan hoi (Port 5556)
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect("tcp://localhost:5556")
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

    time.sleep(0.5) # Doi ket noi ZMQ on dinh
    pub_socket.send_string(json.dumps({"type": "log", "msg": "OK. Random Position script loaded."}))

    # --- 2. Parameters for position control ---
    position_tolerance = 0.05 
    move_speed_limit = 5.0    
    accel_limit = 0.1        # [MOI] Gia toc de chay/dung muot hon (rad/s^2)
    torque_limit = 0.1        

    min_stop_duration = 3.0 
    max_stop_duration = 7.0 
    min_random_pos = -2 * math.pi 
    max_random_pos = 2 * math.pi  

    # KHONG CAN dt = 0.02 NUA. Script se chay theo nhip cua Backend.

    t_simulated = 0.0
    current_mode = "MOVING_TO_RANDOM_POS" 
    target_position = random.uniform(min_random_pos, max_random_pos)
    stop_start_time = 0.0
    stop_duration = 0.0
    actual_position = 0.0 

    start_msg = f"Bat dau di chuyen den vi tri ngau nhien dau tien: {target_position:.2f} rad"
    print(start_msg)
    pub_socket.send_string(json.dumps({"type": "log", "msg": start_msg}))

    # --- DONG BO LAN DAU TU BACKEND ---
    print("Waiting for Backend sync...")
    while True:
        msg = sub_socket.recv_string()
        data = json.loads(msg)
        if 'time' in data and 'actual_position' in data:
            t_simulated = data['time']
            actual_position = data['actual_position']
            break

    # --- 3. Vong lap dieu khien (Event-Driven) ---
    while True:
        # CHO BACKEND
        msg = sub_socket.recv_string()
        data = json.loads(msg)
        
        if 'time' in data:
            t_simulated = data['time'] # Dung clock cua Backend lam chuan tuyet doi
        if 'actual_position' in data:
            actual_position = data['actual_position']

        # Xa sach buffer neu bi tre
        while True:
            try:
                msg_extra = sub_socket.recv_string(flags=zmq.NOBLOCK)
                data_extra = json.loads(msg_extra)
                if 'time' in data_extra: 
                    t_simulated = data_extra['time']
                if 'actual_position' in data_extra: 
                    actual_position = data_extra['actual_position']
            except zmq.Again:
                break 

        # Logic State Machine
        if current_mode == "MOVING_TO_RANDOM_POS":
            if abs(actual_position - target_position) < position_tolerance:
                current_mode = "STOPPED"
                stop_start_time = t_simulated
                stop_duration = random.uniform(min_stop_duration, max_stop_duration)
                
                msg = f"Da den vi tri {target_position:.2f} rad. Dung trong {stop_duration:.2f}s"
                print(msg)
                pub_socket.send_string(json.dumps({"type": "log", "msg": msg}))

        elif current_mode == "STOPPED":
            if (t_simulated - stop_start_time) >= stop_duration:
                current_mode = "MOVING_TO_RANDOM_POS"
                target_position = random.uniform(min_random_pos, max_random_pos)
                
                msg = f"Het thoi gian dung. Bat dau di chuyen den vi tri moi: {target_position:.2f} rad"
                print(msg)
                pub_socket.send_string(json.dumps({"type": "log", "msg": msg}))

        # Gui lenh
        cmd_str = f"d pos {target_position:.4f} 0.0 {torque_limit:.2f} v{move_speed_limit:.4f} a{accel_limit:.1f}"
        
        pub_socket.send_string(json.dumps({
            "type": "raw", 
            "cmd": cmd_str,
            "silent": True
        }))
        
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Script manually interrupted.")
        ctx = zmq.Context()
        pub = ctx.socket(zmq.PUB)
        pub.connect("tcp://localhost:5557")
        pub.send_string(json.dumps({"type": "d_stop"}))
        pub.send_string(json.dumps({"type": "log", "msg": "Random Position script stopped."}))
        pub.close()
        ctx.term()
        sys.exit(0)