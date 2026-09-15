import asyncio
import moteus
import time
import json
import zmq
import math
import random 

# --- CẤU HÌNH (CONFIG) ---
ZMQ_PORT = 5556
TARGET_ID = 1
RUN_DURATION = 60.0   # Thời gian chạy (giây)
FREQ_HZ = 0.5         # Tần số dao động (Hz)
AMPLITUDE_POS = 2.0   # Biên độ (Vòng)
DT = 0.02             # Chu kỳ (s) - 50Hz

async def main():
    print(f"Connecting to Moteus ID {TARGET_ID}...")
    c = moteus.Controller(TARGET_ID)
    
    # Reset lỗi và dừng động cơ trước khi chạy
    await c.set_stop()
    
    # Khởi tạo ZMQ Publisher
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:{ZMQ_PORT}")
    print(f"ZMQ Publisher running on port {ZMQ_PORT}...")
    print("Running SINE PROFILE (Encoder: REAL | Est: SIMULATED)...")
    
    start_time = time.time()
    t_simulated = 0.0

    try:
        while True: 
            loop_start = time.time()
            
            # Kiểm tra thời gian dừng
            if (time.time() - start_time) > RUN_DURATION:
                print("\nDuration finished.")
                break

            t_simulated += DT

            # --- A. TÍNH TOÁN QUỸ ĐẠO (Trajectory) ---
            # Sine Wave
            pos_cmd = AMPLITUDE_POS * math.sin(2 * math.pi * FREQ_HZ * t_simulated)
            vel_ff = AMPLITUDE_POS * (2 * math.pi * FREQ_HZ) * math.cos(2 * math.pi * FREQ_HZ * t_simulated)

            # --- B. GỬI LỆNH & LẤY DỮ LIỆU THỰC (Logic Code 1) ---
            state = await c.set_position(
                position=pos_cmd,   
                velocity=vel_ff,    
                accel_limit=20.0,   
                velocity_limit=20.0,
                watchdog_timeout=math.nan, # An toàn cho script Python (từ Code 2)
                query=True           # Bắt buộc True để lấy dữ liệu Encoder về
            )

            # --- C. XỬ LÝ DỮ LIỆU ---
            
            # 1. Dữ liệu Lệnh (Command) -> Đổi sang Rad
            ref_pos_rad = pos_cmd * 2 * math.pi
            ref_vel_rad = vel_ff * 2 * math.pi

            # 2. Dữ liệu Encoder Thực (Actual - Từ Code 1)
            # Moteus trả về Vòng (turns) -> Nhân 2pi ra Rad
            act_pos_rad = state.values.get(moteus.Register.POSITION, 0.0) * 2 * math.pi
            act_vel_rad = state.values.get(moteus.Register.VELOCITY, 0.0) * 2 * math.pi

            # 3. Dữ liệu Ước lượng (Est - Từ Code 2)
            # Tạo nhiễu giả lập (Fake Noise) để test giao diện/thuật toán observer
            fake_noise_vel = random.uniform(-0.8, 0.8)  
            fake_noise_pos = random.uniform(-0.05, 0.05)

            est_vel_rad = act_vel_rad + fake_noise_vel
            est_pos_rad = act_pos_rad + fake_noise_pos

            # --- D. GỬI ZMQ (JSON Payload) ---
            payload = {
                'time': t_simulated,
                
                # Command
                'command_position': ref_pos_rad,
                'command_velocity': ref_vel_rad,
                
                # Actual (Encoder)
                'actual_position': act_pos_rad,
                'actual_velocity': act_vel_rad,
                
                # Estimated (Fake/Simulated)
                'asmo_theta_hat': est_pos_rad,   
                'asmo_w_hat': est_vel_rad,       
                
                # Debug
                'fault': state.values.get(moteus.Register.FAULT, 0)
            }
            
            socket.send_string(json.dumps(payload))

            # Log hiển thị
            print(f"Ref:{ref_vel_rad:5.1f} | Act:{act_vel_rad:5.1f} | Est:{est_vel_rad:5.1f}", end='\r')

            # --- E. GIỮ NHỊP LOOP ---
            elapsed = time.time() - loop_start
            sleep_time = DT - elapsed
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        await c.set_stop()
        socket.close()
        context.term()
        print("\nDisconnected.")

if __name__ == '__main__':
    asyncio.run(main())