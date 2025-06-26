import asyncio
import serial_asyncio
import serial
import time
import argparse
import sys
import socket
from pymavlink import mavutil
import functools
import traceback
 
# --- Configuration ---
DEFAULT_GPS_PORT = "COM3"
DEFAULT_GPS_BAUD = 115200
DEFAULT_DRONE_IP = "192.168.171.109"
DEFAULT_MAVLINK_PORT = 10030
DEFAULT_MAVLINK_SYSID = 255
DEFAULT_MAVLINK_COMPID = 190
TARGET_SYSID = 10
TARGET_COMPID = 1
 
# --- Argument Parsing ---
parser = argparse.ArgumentParser(description='Read RTCM3 from GPS, inject into MAVLink via a single UDP port (Asyncio), and monitor GPS status.')
parser.add_argument('--gps-port', type=str, default=DEFAULT_GPS_PORT, help=f'Serial port for Base F9P GPS (e.g., COM3, /dev/ttyACM0). Default: {DEFAULT_GPS_PORT}')
parser.add_argument('--gps-baud', type=int, default=DEFAULT_GPS_BAUD, help='Baud rate for Base F9P GPS. Default: %(default)s')
parser.add_argument('--drone-ip', type=str, default=DEFAULT_DRONE_IP, help=f'IP address of Drone MAVLink endpoint. Default: {DEFAULT_DRONE_IP}')
parser.add_argument('--mavlink-port', type=int, default=DEFAULT_MAVLINK_PORT, help='UDP port for MAVLink. Default: %(default)s')
parser.add_argument('--mav-sysid', type=int, default=DEFAULT_MAVLINK_SYSID, help='MAVLink System ID for this script. Default: %(default)s')
parser.add_argument('--mav-compid', type=int, default=DEFAULT_MAVLINK_COMPID, help='MAVLink Component ID for this script. Default: %(default)s')
parser.add_argument('--target-sysid', type=int, default=TARGET_SYSID, help='MAVLink Target System ID (Autopilot). Default: %(default)s')
parser.add_argument('--target-compid', type=int, default=TARGET_COMPID, help='MAVLink Target Component ID (Autopilot). Default: %(default)s')
parser.add_argument('--run-injector', action='store_true', help='Run the RTCM injector task.')
parser.add_argument('--run-monitor', action='store_true', help='Run the MAVLink monitor task.')
 
args = parser.parse_args()
 
# --- Global State ---
gps_reader = None
gps_writer = None
mav_connection = None
stop_event = asyncio.Event()
last_fix_type = -1
last_fix_type_str = "Unknown"
last_sats = -1
 
# --- GPS Fix Type Mapping ---
GPS_FIX_TYPE = {0: "NO_GPS", 1: "NO_FIX", 2: "2D_FIX", 3: "3D_FIX", 4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED", 7: "STATIC", 8: "PPP"}
 
async def setup_connections_async(attempt_gps: bool, attempt_mavlink: bool):
    global gps_reader, gps_writer, mav_connection
    gps_setup_ok = not attempt_gps  # If not attempting, consider it "ok" for now
    mav_setup_ok = not attempt_mavlink # If not attempting, consider it "ok" for now
 
    if attempt_gps:
        print(f"Attempting to connect to GPS on {args.gps_port} at {args.gps_baud} baud (async)...")
        try:
            gps_reader, gps_writer = await serial_asyncio.open_serial_connection(
                url=args.gps_port, baudrate=args.gps_baud, timeout=1.0
            )
            print("Async GPS port opened successfully.")
            gps_setup_ok = True
        except serial.SerialException as e:
            print(f"ERROR: Serial error opening async GPS port {args.gps_port}: {e}")
            gps_setup_ok = False
        except Exception as e:
            print(f"ERROR: Unexpected error opening async GPS port {args.gps_port}: {e}"); traceback.print_exc()
            gps_setup_ok = False
       
        if not gps_setup_ok:
             print("GPS setup FAILED.")
 
    if attempt_mavlink:
        print(f"\nSetting up MAVLink UDP Connection:")
        listen_ip = "0.0.0.0"
        conn_str_listen = f"udpin:{listen_ip}:{args.mavlink_port}"
        print(f"  - MAVLink will LISTEN on: {listen_ip}:{args.mavlink_port}")
        print(f"  - MAVLink will SEND to: {args.drone_ip}:{args.mavlink_port}")
 
        try:
            mav_connection = mavutil.mavlink_connection(
                conn_str_listen,
                source_system=args.mav_sysid,
                source_component=args.mav_compid,
                autoreconnect=True,
                robust_parsing=True
            )
 
            if hasattr(mav_connection, 'conn') and isinstance(mav_connection.conn, socket.socket):
                try:
                    print(f"  - Configuring send destination for UDP socket to {args.drone_ip}:{args.mavlink_port}")
                    mav_connection.conn.connect((args.drone_ip, args.mavlink_port))
                except socket.error as se:
                    print(f"WARNING: socket.connect() to ({args.drone_ip}, {args.mavlink_port}) failed: {se}. Sending might rely on last received address.")
                    if hasattr(se, 'winerror') and se.winerror == 10049: # WSAEADDRNOTAVAIL
                        print(f"ERROR: The drone IP '{args.drone_ip}' appears to be invalid for socket.connect(). Please check the IP.")
                        # Don't sys.exit, let main handle based on mav_setup_ok
                        mav_setup_ok = False # Mark as failed
                        # Close what might have been opened partially
                        if mav_connection: mav_connection.close(); mav_connection = None
                        return gps_setup_ok, mav_setup_ok # Early exit from MAVLink setup
 
            mav_connection.target_system = args.target_sysid
            mav_connection.target_component = args.target_compid
            print(f"MAVLink UDP connection configured (Target SysID {args.target_sysid}, CompID {args.target_compid}).")
            print("Waiting for initial MAVLink HEARTBEAT from drone to confirm bidirectional link...")
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, functools.partial(mav_connection.wait_heartbeat, timeout=7))
 
            if mav_connection.target_system == 0 and mav_connection.target_component == 0 :
                if args.target_sysid !=0:
                    print(f"WARNING: Did not receive an initial HEARTBEAT from target drone (SysID {args.target_sysid}) within timeout.")
                    print(f"  Will proceed using configured target {args.target_sysid}/{args.target_compid} for sending.")
                    mav_connection.target_system = args.target_sysid
                    mav_connection.target_component = args.target_compid
                else:
                     print(f"WARNING: Did not receive any initial HEARTBEAT within timeout (target_sysid was 0).")
            else:
                print(f"SUCCESS: Initial HEARTBEAT likely received (or connection established).")
                print(f"  Detected/Set Target SysID: {mav_connection.target_system}, CompID: {mav_connection.target_component}")
                if mav_connection.target_system != args.target_sysid and args.target_sysid != 0:
                     print(f"  NOTE: Initial heartbeat was from SysID {mav_connection.target_system}, but configured target is {args.target_sysid}.")
                     print(f"  Overriding detected target with configured target {args.target_sysid}/{args.target_compid} for outgoing messages.")
                     mav_connection.target_system = args.target_sysid
                     mav_connection.target_component = args.target_compid
            mav_setup_ok = True
        except (socket.error, OSError) as e:
            err_no = e.winerror if hasattr(e, 'winerror') else e.errno
            print(f"ERROR: Could not set up MAVLink UDP listening on '{conn_str_listen}': {e} (Error No: {err_no})")
            if err_no == 10048: print(f"  REASON: Local port {args.mavlink_port} is already in use.")
            elif err_no == 10049 and listen_ip != "0.0.0.0": print(f"  REASON: The listen IP '{listen_ip}' is not valid.")
            mav_setup_ok = False
        except Exception as e:
            print(f"ERROR: Unexpected error setting up MAVLink UDP connection: {e}"); traceback.print_exc()
            mav_setup_ok = False
       
        if not mav_setup_ok:
            print("MAVLink setup FAILED.")
            if mav_connection: mav_connection.close(); mav_connection = None
 
 
    if (attempt_gps and gps_setup_ok) or (attempt_mavlink and mav_setup_ok):
        print("\nWaiting briefly for connections to stabilize...")
        await asyncio.sleep(0.5)
 
    return gps_setup_ok, mav_setup_ok
 
async def inject_rtcm_task():
    global gps_reader, mav_connection, stop_event
    print("[Injector Task] Starting...")
    rtcm_buffer = b''
    last_stat_time = time.monotonic()
    bytes_read_total = 0
    bytes_sent_total = 0
    packets_sent_total = 0
    loop = asyncio.get_running_loop()
 
    while not stop_event.is_set():
        if not mav_connection:
             print("[Injector Task] CRITICAL: MAVLink connection unavailable. Stopping task and script.")
             stop_event.set(); break
        if not gps_reader:
            print("[Injector Task] CRITICAL: GPS Reader unavailable. Stopping task and script.")
            stop_event.set(); break
 
        try:
            data = b''
            try:
                data = await asyncio.wait_for(gps_reader.read(1024), timeout=1.0)
                if data:
                    rtcm_buffer += data
                    bytes_read_total += len(data)
            except asyncio.TimeoutError: pass
            except serial.SerialException as e:
                 print(f"[Injector Task] Async GPS serial read error: {e}. Attempting to continue...")
                 await asyncio.sleep(2); continue
            except Exception as e:
                 print(f"[Injector Task] Unexpected async GPS read error: {e}"); traceback.print_exc()
                 await asyncio.sleep(2); continue
 
            while len(rtcm_buffer) > 0 and not stop_event.is_set():
                chunk_len = min(len(rtcm_buffer), 180)
                chunk = rtcm_buffer[:chunk_len]
                mavlink_data_field = bytearray(180)
                mavlink_data_field[:chunk_len] = chunk
 
                if mav_connection.target_system == 0 and args.target_sysid != 0:
                     mav_connection.target_system = args.target_sysid
                     mav_connection.target_component = args.target_compid
 
                send_func = functools.partial(
                    mav_connection.mav.gps_rtcm_data_send,
                    flags=0, len=chunk_len, data=mavlink_data_field
                )
                try:
                    await loop.run_in_executor(None, send_func)
                    packets_sent_total += 1
                    bytes_sent_total += chunk_len
                    rtcm_buffer = rtcm_buffer[chunk_len:]
                except (socket.error, OSError) as e:
                     print(f"[Injector Task] UDP Socket/OS error during RTCM send: {e}.")
                     await asyncio.sleep(2); break
                except Exception as e:
                     print(f"[Injector Task] Unexpected error during executor send (RTCM): {e}"); traceback.print_exc()
                     await asyncio.sleep(1); break
 
            current_time = time.monotonic()
            if current_time - last_stat_time >= 10.0:
                if bytes_read_total > 0 or bytes_sent_total > 0 or len(rtcm_buffer) > 0:
                     print(f"[Injector Task] Stats (10s): Read={bytes_read_total} B, Sent={bytes_sent_total} B in {packets_sent_total} pkts. Buffer={len(rtcm_buffer)} B")
                bytes_read_total = 0; bytes_sent_total = 0; packets_sent_total = 0
                last_stat_time = current_time
 
            if not data and len(rtcm_buffer) == 0:
                await asyncio.sleep(0.02)
 
        except asyncio.CancelledError: print("[Injector Task] Cancelled."); break
        except Exception as e:
            if not stop_event.is_set():
                print(f"[Injector Task] FATAL: Unexpected error in task loop: {e}"); traceback.print_exc()
                print("[Injector Task] Signaling stop due to fatal error."); stop_event.set()
            break
    print("[Injector Task] Finished.")
 
async def monitor_mavlink_task():
    global mav_connection, stop_event, last_fix_type, last_fix_type_str, last_sats
    print("[Monitor Task] Starting...")
    last_heartbeat_time = time.monotonic()
    received_first_heartbeat = False # This task's own flag for first heartbeat
    loop = asyncio.get_running_loop()
 
    if not mav_connection:
        print("[Monitor Task] MAVLink connection unavailable at start. Task will not run.")
        return
 
    while not stop_event.is_set():
        try:
            recv_func = lambda: mav_connection.recv_match(
                type=['GPS_RAW_INT', 'HEARTBEAT', 'STATUSTEXT'],
                blocking=True,
                timeout=1.0
            )
            msg = None
            try:
                 msg = await asyncio.wait_for(loop.run_in_executor(None, recv_func), timeout=1.5)
            except asyncio.TimeoutError:
                 pass # Normal if no message within 1.5s
 
            if msg is None:
                # Check if we should warn about missing heartbeats
                current_time_monitor = time.monotonic()
                if received_first_heartbeat and (current_time_monitor - last_heartbeat_time > 15):
                     print(f"[Monitor Task] WARNING: No HEARTBEAT received from target drone (SysID {args.target_sysid}) for >15 seconds.")
                     last_heartbeat_time = current_time_monitor + 10 # Avoid spam by resetting effective last time
                elif not received_first_heartbeat and (current_time_monitor - last_heartbeat_time > 10): # Initial wait time
                    print(f"[Monitor Task] INFO: Still waiting for first HEARTBEAT from target drone (SysID {args.target_sysid}).")
                    last_heartbeat_time = current_time_monitor # Reset for next check
                await asyncio.sleep(0.1)
                continue
 
            # Filter by source system ID
            if msg.get_srcSystem() != args.target_sysid:
                if msg.get_type() == 'HEARTBEAT':
                    print(f"[Monitor Task] Info: Received HEARTBEAT from non-target system: ID {msg.get_srcSystem()}/{msg.get_srcComponent()}")
                continue
 
            msg_type = msg.get_type()
 
            # General log for any message from the target (unless it's GPS_RAW_INT, handled specially below)
            if msg_type not in ['GPS_RAW_INT']:
                 print(f"[Monitor Task] RX from Target ({msg.get_srcSystem()}/{msg.get_srcComponent()}): {msg_type}")
 
            if msg_type == 'HEARTBEAT':
                 if not received_first_heartbeat:
                     print(f"[Monitor Task] SUCCESS: Received first HEARTBEAST from target drone (SysID {msg.get_srcSystem()}, CompID {msg.get_srcComponent()}).")
                     received_first_heartbeat = True
                     # Update mav_connection's target if it wasn't set or was set differently
                     if mav_connection.target_system != msg.get_srcSystem() or \
                        mav_connection.target_component != msg.get_srcComponent():
                         print(f"  Updating mav_connection target to confirmed {msg.get_srcSystem()}/{msg.get_srcComponent()}")
                         mav_connection.target_system = msg.get_srcSystem()
                         mav_connection.target_component = msg.get_srcComponent()
                 last_heartbeat_time = time.monotonic()
 
 
            elif msg_type == 'GPS_RAW_INT':
                 if not received_first_heartbeat: # Only process if we've seen a heartbeat
                     # This print might be too verbose if GPS_RAW_INT comes before HEARTBEAT often
                     # print("[Monitor Task] GPS_RAW_INT received, but waiting for first HEARTBEAT from target.")
                     continue
 
                 # Brief, continuous log for every GPS_RAW_INT
                 print(f"[Monitor Task] RX from Target ({msg.get_srcSystem()}/{msg.get_srcComponent()}): {msg_type} (Sats: {msg.satellites_visible if msg.satellites_visible != 255 else 'N/A'}, Fix: {msg.fix_type})")
 
                 current_fix_type = msg.fix_type
                 current_sats = msg.satellites_visible if msg.satellites_visible != 255 else -1
                 if current_fix_type != last_fix_type or current_sats != last_sats:
                     fix_type_str = GPS_FIX_TYPE.get(current_fix_type, f"UNKNOWN_FIX({current_fix_type})")
                     sats_str = str(current_sats) if current_sats != -1 else "N/A"
                     print(f"\n---=== DRONE GPS STATUS ({time.strftime('%Y-%m-%d %H:%M:%S')}) ===---")
                     print(f"  Fix Type: {fix_type_str} (Raw: {current_fix_type})")
                     print(f"  Satellites: {sats_str}")
                     print(f"----------------------------------------------------")
                     last_fix_type, last_fix_type_str, last_sats = current_fix_type, fix_type_str, current_sats
 
            elif msg_type == 'STATUSTEXT':
                print(f"[Monitor Task] STATUSTEXT from Target Drone ({msg.get_srcSystem()}/{msg.get_srcComponent()}): {msg.text}")
 
        except asyncio.CancelledError: print("[Monitor Task] Cancelled."); break
        except (socket.error, OSError) as e:
             print(f"[Monitor Task] UDP Socket/OS error during MAVLink receive: {e}")
             await asyncio.sleep(2)
        except mavutil.mavlink.MAVLinkReadError as e:
             print(f"[Monitor Task] MAVLink message read error: {e}.");
             await asyncio.sleep(0.1)
        except Exception as e:
             if not stop_event.is_set():
                 print(f"[Monitor Task] FATAL: Unexpected error: {e}"); traceback.print_exc()
                 print("[Monitor Task] Signaling stop."); stop_event.set()
             break
    print("[Monitor Task] Finished.")
 
async def cleanup_async():
    global gps_writer, mav_connection
    print("\n[Cleanup] Initiating cleanup...")
    if gps_writer:
        try:
            if not gps_writer.is_closing():
                gps_writer.close()
                await gps_writer.wait_closed()
            print("[Cleanup] Async GPS port closed.")
        except Exception as e: print(f"[Cleanup] Error closing GPS: {e}"); traceback.print_exc()
    else: print("[Cleanup] GPS writer was not initialized or already handled.")
 
    if mav_connection:
        try:
            mav_connection.close()
            print("[Cleanup] MAVLink connection closed.")
        except Exception as e: print(f"[Cleanup] Error closing MAVLink: {e}"); traceback.print_exc()
    else: print("[Cleanup] MAVLink connection was not initialized or already handled.")
    print("[Cleanup] Finished.")
 
def print_startup_info():
    print("--- RTCM3 MAVLink Injector & Monitor (Async Single UDP Port) ---")
    print("Configuration:")
    if args.run_injector:
        print(f"  GPS Port: {args.gps_port} @ {args.gps_baud} baud")
    if args.run_injector or args.run_monitor:
        print(f"  Drone IP: {args.drone_ip}")
        print(f"  MAVLink UDP Port (Bidirectional): {args.mavlink_port}")
        print(f"  MAVLink GCS SysID/CompID: {args.mav_sysid}/{args.mav_compid}")
        print(f"  MAVLink Target SysID/CompID: {args.target_sysid}/{args.target_compid}")
    print("Tasks to run:")
    print(f"  - RTCM Injector: {'YES' if args.run_injector else 'NO'}")
    print(f"  - MAVLink Monitor: {'YES' if args.run_monitor else 'NO'}")
    print("----------------------------------------------------")
    if args.run_monitor or args.run_injector:
        print("IMPORTANT: Ensure the drone is configured to SEND MAVLink telemetry")
        print(f"           to THIS computer's IP address on UDP port {args.mavlink_port}.")
    print("----------------------------------------------------")
 
async def main():
    # Validate arguments based on selected tasks
    if not args.run_injector and not args.run_monitor:
        print("ERROR: No tasks selected. Use --run-injector and/or --run-monitor.")
        parser.print_help()
        sys.exit(1)
 
    if args.run_injector and not args.gps_port:
        parser.error("--gps-port is required when --run-injector is specified.")
    if (args.run_injector or args.run_monitor) and not args.drone_ip:
        parser.error("--drone-ip is required when --run-injector or --run-monitor is specified.")
 
    print_startup_info()
    injector_task_obj, monitor_task_obj = None, None
    active_tasks = []
 
    gps_conn_ok = False
    mav_conn_ok = False
 
    try:
        should_attempt_gps = args.run_injector
        should_attempt_mavlink = args.run_injector or args.run_monitor
 
        if should_attempt_gps or should_attempt_mavlink:
            gps_conn_ok, mav_conn_ok = await setup_connections_async(
                attempt_gps=should_attempt_gps,
                attempt_mavlink=should_attempt_mavlink
            )
 
        # Check if required connections were successful
        if args.run_injector and not gps_conn_ok:
            print("[Main] CRITICAL: GPS connection failed, but --run-injector was specified. Exiting.")
            await cleanup_async()
            return
        if args.run_injector and not mav_conn_ok: # Injector needs MAVLink too
            print("[Main] CRITICAL: MAVLink connection failed, but --run-injector was specified. Exiting.")
            await cleanup_async()
            return
        if args.run_monitor and not mav_conn_ok:
            print("[Main] CRITICAL: MAVLink connection failed, but --run-monitor was specified. Exiting.")
            await cleanup_async()
            return
 
        print("\n[Main] Starting async tasks...")
        stop_event.clear()
 
        if args.run_injector:
            print("[Main] Creating RTCM injector task...")
            injector_task_obj = asyncio.create_task(inject_rtcm_task(), name="RTCM_Injector")
            active_tasks.append(injector_task_obj)
 
        if args.run_monitor:
            print("[Main] Creating MAVLink monitor task...")
            monitor_task_obj = asyncio.create_task(monitor_mavlink_task(), name="MAVLink_Monitor")
            active_tasks.append(monitor_task_obj)
 
        if not active_tasks:
            print("[Main] No tasks were started (this should not happen if checks above are correct). Exiting.")
            await cleanup_async()
            return
 
        # ... (rest of the task running loop from the original main function) ...
        while not stop_event.is_set() and active_tasks:
            runnable_tasks = [task for task in active_tasks if not task.done()]
            if not runnable_tasks:
                print("[Main] All active tasks have completed.")
                break
 
            done, pending = await asyncio.wait(
                runnable_tasks,
                return_when=asyncio.FIRST_COMPLETED, timeout=1.0
            )
            if done:
                for task in done:
                    task_name = task.get_name() if hasattr(task, 'get_name') else "Task"
                    try:
                        await task # Propagate exceptions if any
                        print(f"[Main] Task '{task_name}' completed normally.")
                    except asyncio.CancelledError:
                        print(f"[Main] Task '{task_name}' was cancelled.")
                    except Exception as e:
                        print(f"[Main] Task '{task_name}' failed with exception: {e}"); traceback.print_exc()
                        print(f"[Main] Signaling stop due to failure in task '{task_name}'."); stop_event.set()
           
            # Check if all tasks are done (e.g., if one failed and set stop_event, others might finish quickly)
            if all(task.done() for task in active_tasks):
                if not stop_event.is_set(): # If stop_event wasn't set by a task failure
                    print("[Main] All tasks completed without explicit stop signal.")
                break
 
 
    except KeyboardInterrupt: print("\n[Main] KeyboardInterrupt. Signaling stop..."); stop_event.set()
    except asyncio.CancelledError: print("[Main] Main task cancelled."); stop_event.set()
    except Exception as e:
         print(f"\n[Main] An unexpected error occurred directly in main function: {e}"); traceback.print_exc(); stop_event.set()
    finally:
        print("[Main] Initiating shutdown sequence...");
        if not stop_event.is_set(): stop_event.set()
 
        all_managed_tasks = [t for t in [injector_task_obj, monitor_task_obj] if t is not None]
        if all_managed_tasks:
             print(f"[Main] Waiting for {len(all_managed_tasks)} managed tasks to finalize...")
             for task_idx, task in enumerate(all_managed_tasks):
                 task_name = task.get_name() if hasattr(task, 'get_name') else f"TaskObj-{task_idx}"
                 if not task.done():
                     print(f"[Main] Requesting cancellation for task '{task_name}'...")
                     task.cancel()
                 else:
                     try:
                         if task.exception(): print(f"[Main] Task '{task_name}' was already done with exception: {task.exception()}")
                         # else: print(f"[Main] Task '{task_name}' was already done normally.")
                     except (asyncio.CancelledError, asyncio.InvalidStateError):
                          print(f"[Main] Task '{task_name}' was already done (cancelled/invalid state).")
 
             results = await asyncio.gather(*all_managed_tasks, return_exceptions=True)
             for i, result in enumerate(results):
                 task_name = all_managed_tasks[i].get_name() if hasattr(all_managed_tasks[i], 'get_name') else f"TaskObj-{i}"
                 if isinstance(result, asyncio.CancelledError):
                     print(f"[Main] Task '{task_name}' confirmed cancelled during gather.")
                 elif isinstance(result, Exception):
                     print(f"[Main] Task '{task_name}' terminated with error during gather: {result}")
             print("[Main] All managed tasks are now finalized.")
        else:
             print("[Main] No tasks were actively managed or they already completed/failed before shutdown gather.")
        await cleanup_async()
    print("[Main] Script finished.")
 
 
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
         print("\n[Entry Point] KeyboardInterrupt detected. Main loop should handle.")
    except Exception as e:
         print(f"\n[Entry Point] CRITICAL Top-level unhandled exception: {e}")
         traceback.print_exc()
    finally:
        print("[Entry Point] Application has concluded.")
 
