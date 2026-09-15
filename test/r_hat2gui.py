import asyncio
import moteus
import time
import json
import zmq
import math
import os

# --- CONFIGURATION ---
ZMQ_PORT = 5556
TARGET_ID = 1
RUN_DURATION = 30.0  # Seconds to run before auto-stopping

BASE_RESISTANCE_OHMS = 0.05529112076079028  # (Ohm) - R0 - Dien tro goc luc 30°C
# -----------------------------------------------------------------

TEMP_BASE_C = 35.0           # (Celsius) - T0 - Nhiet do goc
TEMP_COEFF_COPPER = 0.00393  # (alpha) - He so nhiet cua dong

async def main():
    print(f"Connecting to Moteus ID {TARGET_ID}...")
    
    # --- BUOC 1: KET NOI CONTROLLER CO BAN ---
    # Chi ket noi doi tuong Controller, khong khoi tao Stream voi.
    c = moteus.Controller(TARGET_ID)
    
    # --- BUOC 2: GUI LENH STOP CO BAN ---
    # Hy vong rang lenh nay (giong code sine wave) se chay duoc
    # va "reset" lai controller, lam no ngung auto-stream.
    print("Sending initial stop command...")
    await c.set_stop()
    await asyncio.sleep(0.1) # Cho mot chut de lenh duoc thuc thi

    # --- BUOC 3: KHOI TAO STREAM ---
    # Bay gio, chung ta moi thu khoi tao stream.
    print("Initializing data stream...")
    stream = moteus.Stream(c, ["servo_stats"])
    
    # Gui lai lenh 'd stop' qua stream de chac chan
    await stream.command(b"d stop")
    
    
    # Initialize ZeroMQ
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://0.0.0.0:{ZMQ_PORT}") 
    
    print(f"ZMQ Publisher running on port {ZMQ_PORT}...")
    
    # --- SEND COMMAND ONCE ---
    # Lenh nay yeu cau giu vi tri 0, van toc 0, nhung ap 1.0 Nm feedforward
    # (va gioi han dong 10A). Rat tot de lam nong dong co.
    cmd_string = "d pos 0.0 0.0 1.0 c10.0"
    print(f"Sending command ONCE: '{cmd_string}'")
    await stream.command(cmd_string.encode('latin1'))

    print(f"Monitoring data for {RUN_DURATION} seconds...")
    print("Press Ctrl+C to stop early.")

    start_time = time.time()

    try:
        # Loop for RUN_DURATION seconds, reading data
        while (time.time() - start_time) < RUN_DURATION:
            
            # Read feedback data
            try:
                data = await stream.read_data("servo_stats")
            except Exception:
                await asyncio.sleep(0.01)
                continue
            
            if data:
                # Get data from firmware
                r_hat = getattr(data, 'rls_R_hat', 0.0)
                
                # --- TINH TOAN NHIET DO TU R_HAT ---
                temp_estimated = 0.0
                if r_hat <= 0.0 or BASE_RESISTANCE_OHMS <= 0.0:
                    # Neu R_hat hoac R0 chua co, gia dinh nhiet do co ban
                    temp_estimated = TEMP_BASE_C 
                else:
                    # Ap dung cong thuc: T = T0 + [ ((R/R0) - 1) / alpha ]
                    temp_estimated = TEMP_BASE_C + \
                        (((r_hat / BASE_RESISTANCE_OHMS) - 1.0) / TEMP_COEFF_COPPER)
                # ------------------------------------

                # temp = getattr(data, 'motor_temp_C', 0.0) 
                
                torque_meas = getattr(data, 'torque_Nm', 0.0)
                velocity = getattr(data, 'velocity', 0.0)
                
                t_simulated = time.time() - start_time

                # Pack JSON for sending
                payload = {
                    'time': t_simulated,
                    'r_hat': r_hat,
                    'temp': temp_estimated,  # <-- SU DUNG GIA TRI TU TINH
                    'torque': torque_meas,
                    'velocity': velocity
                }
                
                socket.send_string(json.dumps(payload))
                
                # Print log (Da them Temp)
                remaining = RUN_DURATION - t_simulated
                print(f"Time: {t_simulated:.1f}s (Stop in {remaining:.1f}s) | R_hat: {r_hat:.4f} | Temp: {temp_estimated:.1f}C | Torque: {torque_meas:.2f} Nm", end='\r')

            await asyncio.sleep(0.02)
        
        print(f"\n{RUN_DURATION} second duration finished.")

    except KeyboardInterrupt:
        print("\nStopping early (Ctrl+C)...")
    finally:
        # This block executes on normal exit (30s) or Ctrl+C
        print("Sending 'd stop'...")
        await stream.command(b"d stop")
        socket.close()
        context.term()
        print("Disconnected.")

if __name__ == '__main__':
    if "CAN_INTERFACE" not in os.environ:
        os.environ["CAN_INTERFACE"] = "sockercan"
        os.environ["CAN_CHANNEL"] = "can0"

    asyncio.run(main())