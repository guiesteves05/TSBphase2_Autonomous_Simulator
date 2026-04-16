"""
ROS2 UDP Bridge for Unreal Engine Simulator
-------------------------------------------
This script acts as a lightweight standalone ROS2 node mock. It listens 
for UDP packets containing simulated sensor telemetry (JSON) sent from 
the Unreal Engine environment, parses them, and routes them to their 
respective pseudo-ROS2 topics.
"""

import socket
import json

# Network Configuration
# Setup UDP Server to listen to Unreal Engine on local machine
UDP_IP = "127.0.0.1" # Localhost (must match the UE target IP)
UDP_PORT = 9090      # The dedicated port for simulator telemetry

# Initialize the network socket.
# AF_INET specifies IPv4 addressing. 
# SOCK_DGRAM specifies UDP (connectionless, fast streaming, ideal for real-time sensors).
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # 
sock.bind((UDP_IP, UDP_PORT))

print(f" ROS2 Bridge Node Started!")
print(f" Listening for Unreal Engine Simulator on port {UDP_PORT}...\n")

try:
    # Main event loop: Continuously listen for incoming UDP packets
    while True:

        # Block and wait until a packet is received. 
        # Buffer size is 1024 bytes (sufficient for our lightweight JSON payload).
        data, addr = sock.recvfrom(1024) # Buffer size is 1024 bytes

        # Decode the raw network bytes back into a standard Python string
        payload = data.decode('utf-8')
        
        try:
            # Parse the JSON string from Unreal Engine into a Python dictionary
            sensor_data = json.loads(payload)
            
            #Sensor Hardware Degradation Check
            # Check the health status flag sent by the UE simulator
            status = sensor_data.get("status")
            if status == 0:
                # Simulate a hardware packet drop (do not publish to ROS2)
                print("[WARN] SENSOR DROPOUT DETECTED! No data published.")
                continue

            # ROS2 Topic Routing
            # In a full rclpy implementation, these would be official node.publish() calls.
            # Here, we route to the console to verify the bridge is active and accurate.
            print("-" * 40)
            print(f"[Topic: /gps/fix]    -> Lat: {sensor_data['lat']:.6f}, Lon: {sensor_data['lon']:.6f}")
            print(f"[Topic: /imu/data]   -> Accel X: {sensor_data['accel_x']:.2f}, Yaw Rate: {sensor_data['yaw_z']:.2f}")
            print(f"[Topic: /scan]       -> LiDAR Hits: {sensor_data['lidar']}")
            
        except json.JSONDecodeError:
            # Catch instances where the UDP packet was corrupted or incomplete over the network
            print(f"Received malformed data: {payload}")

# Handle intentional shutdown when the user presses Ctrl+C in the terminal
except KeyboardInterrupt:
    print("\n Shutting down ROS2 Bridge.")
    sock.close()