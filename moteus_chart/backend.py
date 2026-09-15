import asyncio
import json
import time
import sys
import os
import zmq
import zmq.asyncio
import moteus
from moteus import transport_factory
import math
import re

class HardwareBackend:
    def __init__(self):
        # THIET LAP ZEROMQ
        self.context = zmq.asyncio.Context()
        
        bound = False
        for i in range(10):
            try:
                # Tao moi socket de bind lai tu dau neu that bai
                self.sub_socket = self.context.socket(zmq.SUB)
                self.pub_socket = self.context.socket(zmq.PUB)
                
                self.sub_socket.bind("tcp://*:5557")
                self.pub_socket.bind("tcp://*:5556")
                bound = True
                break
            except zmq.error.ZMQError:
                print(f"[Backend] Port in use, retrying in 0.5s... ({i+1}/10)")
                self.sub_socket.close()
                self.pub_socket.close()
                time.sleep(0.5)
        
        if not bound:
            print("[Backend Error] Could not bind to ports 5556/5557. Address still in use.")
            sys.exit(1)
            
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # bien trang thai dieu khien
        self.is_connected = False
        self.control_mode = "stopped"  # che do dung hoac chay
        self.cmd_pos = float('nan')
        self.cmd_vel = 0.0
        self.cmd_torque = 0.0

        self.cmd_vel_limit = math.nan
        self.cmd_accel_limit = math.nan
        
        # hang doi lenh text
        self.pending_text_commands = []
        
        self.dt = 0.02  # chu ky 20ms

        # DOC DU LIEU CALIB TU FILE
        self.base_r = 0.0
        self.base_l = 0.0
        self.base_phi = 0.0
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        calib_file = os.path.join(current_dir, "calib_data.json")
        
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r') as f:
                    data = json.load(f)
                    self.base_r = data.get("r", 0.0)
                    self.base_l = data.get("l", 0.0)
                    self.base_phi = data.get("phi_m", 0.0)
                # IN KET QUA CALIB VAO CONSOLE (DOI PHI SANG mV.s)
                print(f"[Backend] Loaded static calib from JSON: R={self.base_r:.4f}, L={self.base_l}, Phi={self.base_phi*1000.0:.4f} mV.s")
            except Exception as e:
                print(f"[Backend Error] Cannot load calib data: {e}")
        else:
            print(f"[Backend Warning] Calib file not found at: {calib_file}")

    def fault_code_to_text(self, code):
        """CHUYEN DOI MA LOI"""
        fault_map = {
            0: "No fault",
            32: "Calibration fault",
            33: "Motor driver fault",
            34: "Over voltage",
            35: "Encoder fault",
            36: "Motor not configured",
            37: "PWM cycle overrun",
            38: "Over temperature",
            39: "Outside limit",
            40: "Under voltage",
            41: "Config changed",
            42: "Theta invalid",
            43: "Position invalid",
            44: "Stop position deprecated"
        }
        # tra ve nguyen ban neu ma loi khong ro
        return fault_map.get(code, f"Unknown fault code {code}")

    async def gui_listener_task(self):
        """LANG NGHE VA XU LY LENH TU GUI"""
        print("[ZMQ] Backend is listening for commands...")
        while True:
            try:
                msg = await self.sub_socket.recv_string()
                data = json.loads(msg)
                req_type = data.get("type")
                
                if req_type == "connect_fdcan":
                    self.is_connected = True
                    print("[System] FDCAN Connected.")
                    # XOA TOAN BO LOG OK TU DONG TREN PC

                elif req_type == "disconnect_fdcan":
                    self.is_connected = False
                    self.control_mode = "stopped"
                    print("[System] FDCAN Disconnected.")
                # ================= XU LY CHUYEN MODE TU GUI =================
                elif req_type == "set_mode":
                    mode = data.get("mode", "normal")
                    val = "1" if mode == "sensorless" else "0"
                    self.is_sensorless_mode = (mode == "sensorless") 
                    self.pending_text_commands.append(f"conf set servo.enable_sensorless {val}")
                    await self.pub_socket.send_string(json.dumps({"type": "log", "msg": f"[System] Control mode switched to {mode.upper()}"}))
                # =====================================================================
                elif req_type == "d_stop":
                    if not self.is_connected:
                        await self.pub_socket.send_string(json.dumps({"type": "log", "msg": "Error: FDCAN is not connected!"}))
                    else:
                        self.control_mode = "stopped"
                        print("[System] Emergency Stop (Button)!")

                elif req_type == "raw":
                    if not self.is_connected:
                        await self.pub_socket.send_string(json.dumps({"type": "log", "msg": "Error: FDCAN is not connected!"}))
                        continue

                    cmd_str = data.get("cmd", "").strip()
                    parts = cmd_str.split()
                    
                    if not parts:
                        continue
                        
                    # DUNG KHAN CAP
                    if cmd_str == "d stop":
                        self.control_mode = "stopped"
                        await self.pub_socket.send_string(json.dumps({"type": "log", "msg": "OK"}))
                        continue
                        
                    # cu phap chay motor
                    if len(parts) >= 5 and parts[0] == "d" and parts[1] == "pos":
                        try:
                            self.cmd_pos = float(parts[2])
                            self.cmd_vel = float(parts[3])
                            self.cmd_torque = float(parts[4])
                            self.control_mode = "position"

                            self.cmd_vel_limit = math.nan
                            self.cmd_accel_limit = math.nan
                            for opt in parts[5:]:
                                if opt.startswith('v'):
                                    self.cmd_vel_limit = float(opt[1:])
                                elif opt.startswith('a'):
                                    self.cmd_accel_limit = float(opt[1:])
                            
                            # Phuc hoi chu OK nhu tview
                            if not data.get("silent", False):
                                await self.pub_socket.send_string(json.dumps({"type": "log", "msg": "OK"}))
                                
                        except ValueError:
                            if not data.get("silent", False):
                                await self.pub_socket.send_string(json.dumps({"type": "log", "msg": "Error: Invalid numbers!"}))
                    
                    # cu phap cau hinh
                    elif parts[0] in ["conf", "tel", "d"]: 
                        self.pending_text_commands.append(cmd_str)
                        await self.pub_socket.send_string(json.dumps({"type": "log", "msg": "OK"}))
                        
                    else:
                        await self.pub_socket.send_string(json.dumps({"type": "log", "msg": "Error: Unknown or incomplete command!"}))
                
                elif req_type == "log":
                    await self.pub_socket.send_string(json.dumps(data))

            except Exception as e:
                print(f"[ZMQ Error] {e}")

    async def motor_control_task(self):
        qr = moteus.QueryResolution()
        qr.q_current = moteus.F32
        qr.d_current = moteus.F32
        qr.temperature = moteus.F32
        qr.control_torque = moteus.F32
        qr._extra = {
            0x160: 3,
            0x161: 3,
            0x162: 3,
            0x165: 3,
            0x166: 3,
        }

        controller = None
        stream = None
        was_connected = False

        async def release_hardware_session():
            nonlocal controller, stream
            if controller is not None:
                try:
                    await controller.set_stop(query=False)
                except Exception:
                    pass

            # GIAI PHONG CONG COM
            try:
                if transport_factory.GLOBAL_TRANSPORT is not None:
                    transport_factory.GLOBAL_TRANSPORT.close()
                    transport_factory.GLOBAL_TRANSPORT = None
            except Exception as release_err:
                print(f"[Release Warning] Failed to close transport cleanly: {release_err}")

            stream = None
            controller = None

        while True:
            if not self.is_connected:
                if was_connected:
                    await release_hardware_session()
                    self.pending_text_commands.clear()
                    self.control_mode = "stopped"
                    was_connected = False
                    print("[System] Hardware session released.")
                await asyncio.sleep(0.05)
                continue

            if controller is None:
                try:
                    await asyncio.sleep(0.2)
                    
                    controller = moteus.Controller(id=1, query_resolution=qr)
                    stream = moteus.Stream(controller)
                    
                    # --- XA BO DEM CHONG RAC TRUOC KHI HOAT DONG ---
                    try:
                        # Bo cac ky tu '\n' thua ra khoi lenh
                        await stream.command(b"tel stop", allow_any_response=True)
                        await asyncio.sleep(0.05)
                        
                        # --- TU DONG XOA GIOI HAN POSITION DE CHONG LOI 39 ---
                        await stream.command(b"conf set servopos.position_min nan", allow_any_response=True)
                        await asyncio.sleep(0.05)
                        await stream.command(b"conf set servopos.position_max nan", allow_any_response=True)
                        await asyncio.sleep(0.05)

                        # --- KICH HOAT CHE DO MPC ---
                        await stream.command(b"conf set servo.enable_mpc 1", allow_any_response=True)
                        await asyncio.sleep(0.05)
                        
                    except Exception:
                        pass
                        
                    await controller.set_stop(query=True)
                    self.control_mode = "stopped"
                    self.pending_text_commands.clear()
                    print("Hardware initialized.")
                except Exception as init_err:
                    print(f"Failed to init hardware: {init_err}")
                    await self.pub_socket.send_string(json.dumps({"type": "log", "msg": f"Error: Failed to init hardware: {init_err}"}))
                    self.is_connected = False
                    await asyncio.sleep(0.2)
                    continue

            was_connected = True
            await self.pub_socket.send_string(json.dumps({"type": "heartbeat"}))

            try:
                while self.pending_text_commands:
                    txt_cmd = self.pending_text_commands.pop(0)
                    try:
                        response = await stream.command(txt_cmd.encode('utf-8'), allow_any_response=True)
                        if response:
                            resp_str = response.decode('utf-8', errors='ignore')
                            
                            # --- LOAI BO HOAN TOAN KY TU RAC DE FIX LOI FONT ---
                            clean_str = ''.join(c for c in resp_str if 32 <= ord(c) <= 126 or c in '\n\r').strip()
                            
                            if clean_str:
                                await self.pub_socket.send_string(json.dumps({"type": "log", "msg": f"Reply: {clean_str}"}))
                                
                            if txt_cmd in ("conf get servo.pid_dq.kp", "conf get servo.pid_dq.ki"):
                                pid_payload = {}
                                try:
                                    # --- DUNG REGEX DE BOC TACH CHINH XAC SO RA KHOI CHUOI CO RAC ---
                                    numbers = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", clean_str)
                                    value = float(numbers[-1]) if numbers else None
                                except Exception:
                                    value = None
                                    
                                if value is not None:
                                    if txt_cmd.endswith(".kp"):
                                        pid_payload["kp"] = value
                                    elif txt_cmd.endswith(".ki"):
                                        pid_payload["ki"] = value
                                    await self.pub_socket.send_string(json.dumps({"type": "pid_dq", **pid_payload}))
                    except Exception as stream_err:
                        print(f"[Stream Error] {stream_err}")

                if self.control_mode == "stopped":
                    state = await controller.set_stop(query=True)
                else:
                    state = await controller.set_position(
                        position=self.cmd_pos,
                        velocity=self.cmd_vel,
                        maximum_torque=self.cmd_torque,
                        velocity_limit=self.cmd_vel_limit,
                        accel_limit=self.cmd_accel_limit,
                        query=True,
                        query_override=qr
                    )

                if state:
                    v = state.values
                    actual_pos = v.get(0x001, 0.0)
                    actual_vel = v.get(0x002, 0.0)
                    fault_code = int(v.get(0x00f, 0) or 0)

                    if fault_code > 50:
                        fault_code = 0

                    sl_vel = v.get(0x166, 0.0) # Thanh ghi 0x166 chua sensorless_velocity (Hz)
                    
                    if getattr(self, 'is_sensorless_mode', False) and abs(sl_vel) > 1.7:
                        actual_pos = 0.0
                        actual_vel = 0.0
                        
                    report_cmd_pos = actual_pos if self.control_mode == "stopped" else self.cmd_pos
                    report_cmd_vel = 0.0 if self.control_mode == "stopped" else self.cmd_vel

                    raw_r_hat = v.get(0x160, 0.0)
                    if raw_r_hat == 0.0 or abs(raw_r_hat - 0.01) < 1e-5:
                        final_r_hat = self.base_r
                    else:
                        final_r_hat = raw_r_hat

                    raw_phi_m = v.get(0x161, 0.0)
                    final_phi_m = self.base_phi if raw_phi_m == 0.0 else raw_phi_m

                    raw_l_hat = v.get(0x162, 0.0)
                    final_l_hat = self.base_l if raw_l_hat == 0.0 else raw_l_hat

                    telemetry = {
                        "time": time.time(),
                        "command_position": report_cmd_pos,
                        "actual_position": actual_pos,
                        "sensorless_position": v.get(0x165, 0.0),
                        "command_velocity": report_cmd_vel,
                        "actual_velocity": actual_vel,
                        "sensorless_velocity": v.get(0x166, 0.0),
                        "command_torque": v.get(0x03a, 0.0),
                        "actual_torque": v.get(0x003, 0.0),
                        "i_q": v.get(0x004, 0.0),
                        "i_d": v.get(0x005, 0.0),
                        "temp": v.get(0x00e, 0.0),
                        "r_hat": final_r_hat,
                        "phi_m": final_phi_m,
                        "L_hat": final_l_hat,
                        "motor_position": {
                            "error": fault_code,
                            "code": fault_code,
                            "error_text": self.fault_code_to_text(fault_code),
                        }
                    }
                    await self.pub_socket.send_string(json.dumps(telemetry))

            except Exception as e:
                print(f"[Error] Hardware loop failure: {e}")
                await self.pub_socket.send_string(json.dumps({"type": "log", "msg": f"Hardware Disconnected: {str(e)}"}))
                await release_hardware_session()
                self.is_connected = False
                self.control_mode = "stopped"
                was_connected = False
                await asyncio.sleep(0.1)

            await asyncio.sleep(self.dt)

    async def run(self):
        await asyncio.gather(self.gui_listener_task(), self.motor_control_task())

if __name__ == '__main__':
    if sys.platform == 'win32':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
        
    backend = HardwareBackend()
    try:
        asyncio.run(backend.run())
    except KeyboardInterrupt:
        print("Backend stopped.")