import socket
import json

# Setup UDP Server to listen to Unreal Engine
UDP_IP = "127.0.0.1" # Localhost
UDP_PORT = 9090

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f" ROS2 Bridge Node Started!")
print(f" Listening for Unreal Engine Simulator on port {UDP_PORT}...\n")

try:
    while True:
        data, addr = sock.recvfrom(1024) # Buffer size is 1024 bytes
        payload = data.decode('utf-8')
        
        try:
            # Parse the JSON from Unreal Engine
            sensor_data = json.loads(payload)
            
            status = sensor_data.get("status")
            if status == 0:
                print("[WARN] SENSOR DROPOUT DETECTED! No data published.")
                continue

            # Route to ROS2 Topics (Simulated via Print for testing)
            print("-" * 40)
            print(f"[Topic: /gps/fix]    -> Lat: {sensor_data['lat']:.6f}, Lon: {sensor_data['lon']:.6f}")
            print(f"[Topic: /imu/data]   -> Accel X: {sensor_data['accel_x']:.2f}, Yaw Rate: {sensor_data['yaw_z']:.2f}")
            print(f"[Topic: /scan]       -> LiDAR Hits: {sensor_data['lidar']}")
            
        except json.JSONDecodeError:
            print(f"Received malformed data: {payload}")

except KeyboardInterrupt:
    print("\n Shutting down ROS2 Bridge.")
    sock.close()