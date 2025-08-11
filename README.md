
# RTK HANDLING


	This is the guide for RTK Handling for cm level precision with RTK GPS.

### Contents

1. Introduction
2. **System Design**
3. **Setup**
4. **Execution**
5. **Troubleshooting**
6. **Conclusion**


## Introduction

This script is a lightweight software bridge that enables high-precision, real-time kinematic (RTK) positioning for MAVLink-based vehicles like drones and rovers. It fetches GPS correction data (in RTCM format) from an Local NTRIP caster using #pygpsclient and injects it directly into your vehicle's MAVLink data stream.

This allows the vehicle to achieve centimeter-level positional accuracy (an "RTK Fix") using the ground control station's base RTK GPS with Pygpsclient for NTRIP Caster RTCM Streaming and Survey-in, with a dedicated radio telemetry link for the correction data. The script also monitors and displays the vehicle's GPS status in real-time, providing immediate feedback as the fix improves from a standard 3D fix to RTK Float or RTK Fixed.

## System Design
<img width="1656" height="857" alt="Screenshot from 2025-08-11 11-25-02" src="https://github.com/user-attachments/assets/6d4d6148-6b57-448e-bf86-b1d2655b7c0a" />


## Architecture Design

<img width="3840" height="717" alt="Untitled diagram _ Mermaid Chart-2025-08-11-055850" src="https://github.com/user-attachments/assets/fe5c392b-4aa3-41fb-967e-a1f938995748" />


## Module Design

<img width="1008" height="3842" alt="Untitled diagram _ Mermaid Chart-2025-08-11-053102" src="https://github.com/user-attachments/assets/77b695fa-59fb-4733-84bd-4107d9f8de2d" />


# Setup

``` 
Requirements:

OS: Ubuntu >=22.04
Base RTK GPS with UFO Style antenna attached
Type C cable
DL or any kind of Telemetry

```


## 📋 Prerequisites

Before running this script, ensure you have the following:

1.  **Python 3**: The script is written for Python 3.x.
2.  **Required Python Libraries**: Install them using pip:
    ```bash
    pip install pymavlink pyrtcm
    pip install pygpsclient
    ```
    Reference : https://github.com/semuconsulting/PyGPSClient
3.  **NTRIP Caster Credentials**: You need access to an NTRIP service, which includes:
    * Host / IP Address
    * Port
    * Mountpoint
    * Username and Password (or anonymous access)
4.  **MAVLink Connection**: A way to get MAVLink traffic from your vehicle to the computer running this script. The most common method is using **MAVProxy** as a message router.

## ⚙️ Configuration

All configuration is done by editing the global variables at the top of the Python script.

```python
# --- NTRIP Caster Configuration ---
NTRIP_HOST = "127.0.0.1"
NTRIP_PORT = 2101
NTRIP_MOUNTPOINT = "pygnssutils"
NTRIP_USER = "anon"
NTRIP_PASSWORD = "password"
# -----------------------------------

# --- MAVLink Configuration ---
MAVLINK_LISTEN_HOST = '127.0.0.1'
MAVLINK_LISTEN_PORT = 14551 
# -----------------------------
```

* `NTRIP_*`: Fill these in with the details from your NTRIP service provider.
* `MAVLINK_LISTEN_HOST`: The IP address this script should listen on. `'127.0.0.1'` (localhost) is usually correct if MAVProxy is running on the same computer.
* `MAVLINK_LISTEN_PORT`: The UDP port this script will listen on for MAVLink data from the vehicle and send RTCM data to.

## 🛠️ How to Use

**Step 1: Set up your MAVLink Stream**

Your vehicle's MAVLink data needs to be forwarded to the port this script is listening on. If you are using MAVProxy to connect to your vehicle, you can achieve this by adding an `--out` flag when you start it.

For example, if MAVProxy connects to your drone via a serial port (`/dev/ttyUSB0`) and you want to forward data to this script running on the same machine at port `14551`, you would run:

```bash
mavproxy.py --master udp:192.168.171.202:14550  --out udp:127.0.0.1:14551
```

**Step 2: Configure the Script**

Edit the Python script to set your correct `NTRIP_*` and `MAVLINK_*` parameters as described in the Configuration section.

**Step 3: Run the Script**

With MAVProxy (or another MAVLink router) running, execute the Python script in a new terminal:

```bash
python injecter.py
```

## 📊 Expected Output

If successful, you will see a series of messages indicating the connection status:

```
MAVLink: Waiting for heartbeat on udp:127.0.0.1:14551...
MAVLink: Heartbeat received! (system 1, component 1)
NTRIP: Attempting to connect to [caster.example.com:2101/MY_MOUNTPOINT](https://caster.example.com:2101/MY_MOUNTPOINT)...
NTRIP: Connection established.
NTRIP: Received OK from server. Now listening for RTCM data...
```

After a few moments, as the vehicle gets a better GPS lock thanks to the RTCM data, you will see status updates:

```
--- VEHICLE GPS STATUS ---
Fix Type: 5 (RTK_FLOAT)
Satellites: 18
--------------------------

--- VEHICLE GPS STATUS ---
Fix Type: 6 (RTK_FIXED)
Satellites: 19
--------------------------
```

## 🔧 Troubleshooting

* **`MAVLink: Waiting for heartbeat...` forever**: This script is not receiving any MAVLink data.
    * Check that MAVProxy (or your GCS) is running and properly connected to the vehicle.
    * Verify your MAVProxy `--out` address and port match the `MAVLINK_LISTEN_HOST` and `MAVLINK_LISTEN_PORT` in the script.
    * Check your firewall settings.
* **`NTRIP Error: Did not receive OK from server...`**: The NTRIP caster rejected your connection.
    * Double-check your `NTRIP_HOST`, `NTRIP_PORT`, `NTRIP_MOUNTPOINT`, `NTRIP_USER`, and `NTRIP_PASSWORD`. They are case-sensitive.
* **GPS status never reaches `RTK_FLOAT` or `RTK_FIXED`**:
    * Ensure your vehicle's GPS hardware is RTK-capable.
    * Check that you have a clear view of the sky.
    * Verify the NTRIP caster is active and providing data for your geographic location.
