import socket
import sys
import base64
import io
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink
from pyrtcm import RTCMReader

# --- NTRIP Caster Configuration ---
NTRIP_HOST = "127.0.0.1"
NTRIP_PORT = 2101
NTRIP_MOUNTPOINT = "pygnssutils"
NTRIP_USER = "anon"
NTRIP_PASSWORD = "password"
# -----------------------------------

# --- MAVLink Configuration ---
# This script will now LISTEN on this port for MAVLink traffic
# and send injection data back to the source of that traffic.
MAVLINK_LISTEN_HOST = '127.0.0.1'
MAVLINK_LISTEN_PORT = 14551 # Make sure MAVProxy is --out'ing to this port
# -----------------------------

# --- GPS STATUS DEBUGGING GLOBALS ---
last_fix_type = -1
last_print_time = 0

def create_ntrip_request(host, port, mountpoint, user, password):
    # This function remains unchanged
    mountpoint_str = f"/{mountpoint}"
    user_agent = "Python NTRIP/MAVLink Client"
    auth_str = ""
    if user:
        credentials = f"{user}:{password}".encode('utf-8')
        auth_str = f"Authorization: Basic {base64.b64encode(credentials).decode('ascii')}\r\n"
    request = (
        f"GET {mountpoint_str} HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        f"Ntrip-Version: Ntrip/2.0\r\n"
        f"User-Agent: {user_agent}\r\n"
        f"Connection: close\r\n"
        f"{auth_str}"
        f"\r\n"
    ).encode('utf-8')
    return request

def print_gps_status(msg):
    """Helper function to print GPS status changes."""
    global last_fix_type, last_print_time
    
    fix_type = msg.fix_type
    # Only print if the status has changed or every 5 seconds
    if fix_type != last_fix_type or time.time() - last_print_time > 5:
        fix_type_map = {
            0: "NO_GPS", 1: "NO_FIX", 2: "2D_FIX", 3: "3D_FIX",
            4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"
        }
        status_string = fix_type_map.get(fix_type, f"UNKNOWN({fix_type})")
        
        print("\n--- VEHICLE GPS STATUS ---")
        print(f"Fix Type: {fix_type} ({status_string})")
        print(f"Satellites: {msg.satellites_visible}")
        print("--------------------------\n")
        
        last_fix_type = fix_type
        last_print_time = time.time()

def run_ntrip_mavlink_client():
    """
    Connects to NTRIP, injects RTCM to MAVLink, and monitors GPS status.
    """
    global last_fix_type # Allow modification of global
    
    print(f"NTRIP: Attempting to connect to {NTRIP_HOST}:{NTRIP_PORT}/{NTRIP_MOUNTPOINT}...")
    
    ntrip_sock = None
    mav_connection = None
    
    try:
        # ------------------- MAVLINK CONNECTION SETUP --------------------
        # **MODIFIED**: Changed 'udpout' to 'udp' to allow receiving messages.
        # This will LISTEN for messages on the specified port.
        mav_connection_str = f"udp:{MAVLINK_LISTEN_HOST}:{MAVLINK_LISTEN_PORT}"
        mav_connection = mavutil.mavlink_connection(mav_connection_str, source_system=255)
        
        # ## 1. VERIFY MAVLINK CONNECTION ##
        print(f"MAVLink: Waiting for heartbeat on {mav_connection_str}...")
        mav_connection.wait_heartbeat()
        print(f"MAVLink: Heartbeat received! (system {mav_connection.target_system}, component {mav_connection.target_component})")
        # -----------------------------------------------------------------

        # ------------------- NTRIP CONNECTION SETUP ----------------------
        ntrip_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ntrip_sock.settimeout(10)
        ntrip_sock.connect((NTRIP_HOST, NTRIP_PORT))
        print("NTRIP: Connection established.")
        
        request = create_ntrip_request(NTRIP_HOST, NTRIP_PORT, NTRIP_MOUNTPOINT, NTRIP_USER, NTRIP_PASSWORD)
        ntrip_sock.sendall(request)
        
        response = ntrip_sock.recv(1024)
        if not response.startswith(b'ICY 200 OK') and not response.startswith(b'HTTP/1.1 200 OK'):
            print(f"NTRIP Error: Did not receive OK from server. Response:\n{response.decode('utf-8', errors='ignore')}")
            return

        print("NTRIP: Received OK from server. Now listening for RTCM data...")
        # -----------------------------------------------------------------
        
        buffer = b''
        while True:
            # ## 2. DEBUGGING: GPS STATUS MONITORING ##
            # Check for incoming MAVLink messages in a non-blocking way
            mav_msg = mav_connection.recv_match(type='GPS_RAW_INT', blocking=False)
            if mav_msg:
                print_gps_status(mav_msg)
            
            # This part remains the same: receive NTRIP data and inject it
            data = ntrip_sock.recv(1024)
            if not data:
                print("NTRIP: Connection closed by server.")
                break
                
            buffer += data
            
            try:
                stream = io.BytesIO(buffer)
                rtcm_reader = RTCMReader(stream)
                
                consumed_bytes = 0
                for (raw_data, parsed_data) in rtcm_reader:
                    # Send the raw RTCM data via MAVLink
                    rtcm_message_bytes = raw_data
                    while len(rtcm_message_bytes) > 0:
                        chunk = rtcm_message_bytes[:180]
                        rtcm_message_bytes = rtcm_message_bytes[180:]
                        msg = mavlink.MAVLink_gps_rtcm_data_message(
                            flags=0, len=len(chunk), data=list(chunk) + [0] * (180 - len(chunk))
                        )
                        mav_connection.mav.send(msg)
                    
                    consumed_bytes += len(raw_data)
                
                if consumed_bytes > 0:
                    buffer = buffer[consumed_bytes:]

            except Exception as e:
                print(f"Error parsing data: {e}")

    except socket.timeout:
        print("NTRIP: Connection timed out.")
    except socket.error as e:
        print(f"Socket error: {e}")
    except KeyboardInterrupt:
        print("\nStream stopped by user.")
    finally:
        if ntrip_sock:
            ntrip_sock.close()
            print("NTRIP socket closed.")

if __name__ == "__main__":
    run_ntrip_mavlink_client()
