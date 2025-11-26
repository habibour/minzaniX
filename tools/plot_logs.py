#!/usr/bin/env python3
"""
plot_logs.py - Plot flight controller logs

Usage:
    python plot_logs.py logs.txt
"""

import sys
import matplotlib.pyplot as plt
import numpy as np

def parse_log_file(filename):
    """Parse log file with format: time,roll,pitch,yaw,m0,m1,m2,m3"""
    data = {
        'time': [],
        'roll': [],
        'pitch': [],
        'yaw': [],
        'motor0': [],
        'motor1': [],
        'motor2': [],
        'motor3': []
    }
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split(',')
            if len(parts) >= 8:
                data['time'].append(float(parts[0]))
                data['roll'].append(float(parts[1]))
                data['pitch'].append(float(parts[2]))
                data['yaw'].append(float(parts[3]))
                data['motor0'].append(float(parts[4]))
                data['motor1'].append(float(parts[5]))
                data['motor2'].append(float(parts[6]))
                data['motor3'].append(float(parts[7]))
    
    return data

def plot_attitude(data):
    """Plot roll, pitch, yaw over time"""
    plt.figure(figsize=(12, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(data['time'], np.rad2deg(data['roll']), 'r-', label='Roll')
    plt.ylabel('Roll (deg)')
    plt.grid(True)
    plt.legend()
    
    plt.subplot(3, 1, 2)
    plt.plot(data['time'], np.rad2deg(data['pitch']), 'g-', label='Pitch')
    plt.ylabel('Pitch (deg)')
    plt.grid(True)
    plt.legend()
    
    plt.subplot(3, 1, 3)
    plt.plot(data['time'], np.rad2deg(data['yaw']), 'b-', label='Yaw')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw (deg)')
    plt.grid(True)
    plt.legend()
    
    plt.suptitle('Attitude Estimation')
    plt.tight_layout()

def plot_motors(data):
    """Plot motor outputs over time"""
    plt.figure(figsize=(12, 6))
    
    plt.plot(data['time'], data['motor0'], 'r-', label='Motor 0 (FL)')
    plt.plot(data['time'], data['motor1'], 'g-', label='Motor 1 (FR)')
    plt.plot(data['time'], data['motor2'], 'b-', label='Motor 2 (RR)')
    plt.plot(data['time'], data['motor3'], 'm-', label='Motor 3 (RL)')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Motor Command (0-1)')
    plt.title('Motor Outputs')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_logs.py <logfile>")
        print("\nLog format (CSV):")
        print("time,roll,pitch,yaw,motor0,motor1,motor2,motor3")
        sys.exit(1)
    
    filename = sys.argv[1]
    
    try:
        data = parse_log_file(filename)
        
        if not data['time']:
            print(f"ERROR: No data found in {filename}")
            sys.exit(1)
        
        print(f"Loaded {len(data['time'])} data points")
        
        plot_attitude(data)
        plot_motors(data)
        
        plt.show()
        
    except FileNotFoundError:
        print(f"ERROR: File not found: {filename}")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
