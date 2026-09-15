import sys
import time
import math
import json
import zmq

def run_sine_velocity_test():
    # --- 1. Thiet lap ZeroMQ Publisher (Ket noi voi Backend) ---
    context = zmq.Context()
    pub_socket = context.socket(zmq.PUB)
    pub_socket.connect("tcp://localhost:5557")

    time.sleep(0.5) # Doi ZMQ thiet lap handshake

    # Chi gui 1 log duy nhat bao hieu script da chay thanh cong
    start_msg = f"OK. Sine script loaded."
    pub_socket.send_string(json.dumps({"type": "log", "msg": start_msg}))
    print(f">> {start_msg}")

    # --- 2. Cau hinh tham so song Sine ---
    A = 15.0      # bien do van toc (rad/s)
    f = 0.1      # tan so (Hz)
    dt = 0.01    # chu ky dieu khien (s) - 100Hz
    
    # --- 3. Vong lap dieu khien voi thoi gian ly tuong ---
    t_simulated = 0.0     
    
    while True:
        loop_start_time = time.time() # Ghi lai thoi gian bat dau vong lap thuc te
        
        # Tang bien thoi gian "ly tuong" len dt
        t_simulated += dt 
        
        # Van toc lenh duoc tinh dua tren thoi gian "ly tuong" (de dam bao song sin muot)
        velocity_command = A * math.sin(2 * math.pi * f * t_simulated)
        
        # Gui lenh dieu khien qua Backend (pos = nan de backend tu chay Velocity mode)
        torque_limit = 0.1
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
        run_sine_velocity_test()
    except KeyboardInterrupt:
        print("Script manually interrupted.")
        ctx = zmq.Context()
        sock = ctx.socket(zmq.PUB)
        sock.connect("tcp://localhost:5557")
        # Gui lenh dung dong co khi nhan Stop Script hoac Ctrl+C
        sock.send_string(json.dumps({"type": "d_stop"}))
        sock.send_string(json.dumps({"type": "log", "msg": "Sine script stopped."}))
        sock.close()
        ctx.term()
        sys.exit(0)