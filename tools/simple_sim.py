#!/usr/bin/env python3
"""
Simple Quadcopter Physics Simulator
Communicates with minzaniX flight controller via MAVLink-like messages
"""

import socket
import struct
import time
import math
import sys

class QuadcopterSim:
    def __init__(self):
        # State variables
        self.pos = [0.0, 0.0, 0.5]  # x, y, z (meters)
        self.vel = [0.0, 0.0, 0.0]  # vx, vy, vz (m/s)
        self.attitude = [0.0, 0.0, 0.0]  # roll, pitch, yaw (radians)
        self.angular_vel = [0.0, 0.0, 0.0]  # p, q, r (rad/s)
        
        # Physical constants
        self.mass = 1.5  # kg
        self.gravity = 9.81  # m/s^2
        self.dt = 0.004  # 250Hz update rate
        
        # Motor thrusts (0-1)
        self.motor_cmds = [0.0, 0.0, 0.0, 0.0]
        
        # UDP sockets
        self.fc_addr = ('127.0.0.1', 14550)  # Where FC listens
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 14551))  # Simulator listens here
        self.sock.settimeout(0.001)
        
        print("✓ Simple Quadcopter Simulator started")
        print(f"  Mass: {self.mass} kg")
        print(f"  Update rate: {1/self.dt:.0f} Hz")
        print(f"  Starting altitude: {self.pos[2]:.2f} m")
        print()
    
    def update_physics(self):
        """Simple quadcopter physics - vertical motion only"""
        
        # Calculate total thrust (normalized 0-1)
        total_thrust_norm = sum(self.motor_cmds) / 4.0
        
        # Convert to force (Newtons)
        # Hover thrust = mass * g = 1.5 * 9.81 = 14.715 N
        # Max thrust = 2x hover = 29.43 N
        max_thrust = 2.0 * self.mass * self.gravity
        total_thrust = total_thrust_norm * max_thrust
        
        # Vertical acceleration (simplified - only z-axis)
        accel_z = (total_thrust / self.mass) - self.gravity
        
        # Update velocity
        self.vel[2] += accel_z * self.dt
        
        # Update position
        self.pos[2] += self.vel[2] * self.dt
        
        # Ground collision
        if self.pos[2] < 0.05:
            self.pos[2] = 0.05
            self.vel[2] = 0.0
    
    def send_imu(self):
        """Send IMU data to FC"""
        # Simple IMU message: accel (3 floats) + gyro (3 floats)
        ax = 0.0
        ay = 0.0
        az = self.gravity  # Accelerometer reads gravity when stationary
        
        gx = self.angular_vel[0]
        gy = self.angular_vel[1]
        gz = self.angular_vel[2]
        
        msg = struct.pack('<6f', ax, ay, az, gx, gy, gz)
        self.sock.sendto(msg, self.fc_addr)
    
    def send_baro(self):
        """Send barometer data to FC"""
        # Pressure decreases with altitude
        # P = P0 * exp(-h / H) where H ≈ 8400m
        pressure_sea_level = 101325.0  # Pa
        altitude = self.pos[2]
        pressure = pressure_sea_level * math.exp(-altitude / 8400.0)
        
        # Baro message: pressure (float)
        msg = struct.pack('<f', pressure)
        self.sock.sendto(msg, self.fc_addr)
    
    def receive_motors(self):
        """Receive motor commands from FC"""
        try:
            data, addr = self.sock.recvfrom(1024)
            if len(data) == 16:  # 4 floats
                self.motor_cmds = struct.unpack('<4f', data)
        except socket.timeout:
            pass
    
    def run(self):
        """Main simulation loop"""
        iteration = 0
        start_time = time.time()
        
        print("Simulation running... (Ctrl+C to stop)")
        print()
        
        try:
            while True:
                # Physics update
                self.update_physics()
                
                # Receive motor commands
                self.receive_motors()
                
                # Send sensor data
                self.send_imu()
                if iteration % 5 == 0:  # Baro at 50Hz
                    self.send_baro()
                
                # Display status every second
                if iteration % 250 == 0:
                    elapsed = time.time() - start_time
                    motors_avg = sum(self.motor_cmds) / 4.0
                    print(f"[{elapsed:6.1f}s] Alt: {self.pos[2]:6.2f}m  "
                          f"Vel: {self.vel[2]:+6.2f}m/s  "
                          f"Motors: {motors_avg:.2f}")
                
                # Sleep to maintain 250Hz
                time.sleep(self.dt)
                iteration += 1
                
        except KeyboardInterrupt:
            print("\n\nSimulation stopped")
            print(f"Final altitude: {self.pos[2]:.2f} m")

if __name__ == "__main__":
    sim = QuadcopterSim()
    sim.run()
