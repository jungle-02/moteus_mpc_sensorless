import asyncio
import moteus
import time
import json
import zmq
import math

# --- CAU HINH ---
ZMQ_PORT = 5556
TARGET_ID = 1
RUN_DURATION = 60.0   # Chay 60s 
FREQ_HZ = 0.5         # Tan so dao dong
AMPLITUDE_POS = 2.0   # Quay qua lai +/- 2 vong 
DT = 0.02             # Chu ky gui lenh (50Hz)

async def main():
    print(f"Connecting to Moteus ID {TARGET_ID}...")
    c = moteus.Controller(TARGET_ID)
    
    # 1. Khoi tao Stream (De doc bien ASMO tu Firmware)
    stream = moteus.Stream(c, ["servo_stats"])
    
    # 2. Reset loi va dung dong co (Quan trong)
    await c.set_stop()
    
    # 3. Khoi tao ZMQ Publisher
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:{ZMQ_PORT}")
    print(f"ZMQ Publisher running on port {ZMQ_PORT}...")
    print("Running SINE POSITION profile... (Ctrl+C to stop)")
    
    start_time = time.time()
    t_simulated = 0.0

    try:
        while True: # Chay lien tuc cho den khi het gio hoac Ctrl+C
            loop_start = time.time()
            
            # Kiem tra thoi gian chay
            if (time.time() - start_time) > RUN_DURATION:
                print("\nDuration finished.")
                break

            t_simulated += DT

            # --- A. TINH TOAN QUY DAO (HINH SIN) ---
            # Vi tri dat (Ref Position)
            pos_cmd = AMPLITUDE_POS * math.sin(2 * math.pi * FREQ_HZ * t_simulated)
            
            # Van toc dat (Ref Velocity - Feedforward)
            # Dao ham cua sin la cos
            vel_ff = AMPLITUDE_POS * (2 * math.pi * FREQ_HZ) * math.cos(2 * math.pi * FREQ_HZ * t_simulated)

            # --- B. GUI LENH DIEU KHIEN ---
            state = await c.set_position(
                position=pos_cmd,   
                velocity=vel_ff,    
                accel_limit=20.0,   # Gioi han gia toc
                velocity_limit=20.0,
                query=True          # Lay phan hoi chuan ngay lap tuc
            )

            # --- C. DOC DU LIEU ASMO TU STREAM ---
            asmo_w = 0.0
            asmo_theta = 0.0
            fault_code = 0
            
            try:
                # Doc du lieu stream 
                stream_data = await asyncio.wait_for(stream.read_data("servo_stats"), timeout=0.005)
                
                if stream_data:
                    # Lay du lieu uoc luong tu ASMO 
                    asmo_w = getattr(stream_data, 'asmo_w_hat', 0.0)
                    asmo_theta = getattr(stream_data, 'asmo_theta_hat', 0.0)
                    fault_code = getattr(stream_data, 'fault', 0)
            except:
                pass # Bo qua neu mat goi tin stream

            # --- D. XU LY DU LIEU DE GUI VE ---
            # Lay du lieu thuc te tu Encoder (Register chuan)
            # Moteus tra ve Vong (turns) va Vong/s (turns/s) -> Doi sang Rad va Rad/s
            act_pos_rad = state.values.get(moteus.Register.POSITION, 0.0) * 2 * math.pi
            act_vel_rad = state.values.get(moteus.Register.VELOCITY, 0.0) * 2 * math.pi
            
            # Du lieu lenh (doi sang Rad de cung don vi ve)
            ref_pos_rad = pos_cmd * 2 * math.pi
            ref_vel_rad = vel_ff * 2 * math.pi

            # Du lieu ASMO
            est_vel_rad = asmo_w 
            est_pos_rad = asmo_theta

            # Dong goi JSON
            payload = {
                'time': t_simulated,
                
                'command_position': ref_pos_rad,
                'actual_position': act_pos_rad,
                'asmo_theta_hat': est_pos_rad,
                
                'command_velocity': ref_vel_rad,
                'actual_velocity': act_vel_rad,
                'asmo_w_hat': est_vel_rad,
                
                'fault': fault_code
            }
            
            socket.send_string(json.dumps(payload))

            # Log ra terminal
            print(f"Time:{t_simulated:.1f}s | Ref:{ref_vel_rad:.1f} | Act:{act_vel_rad:.1f} | Est:{est_vel_rad:.1f} | Fault:{fault_code}", end='\r')

            # --- E. DUY TRI THOI GIAN THUC ---
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