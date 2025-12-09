#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import time
import math
import threading
import sys
import os
import matplotlib.pyplot as plt
import numpy as np
import shutil

# --- HARDWARE CONSTANTS ---
TILT_MIN_DXL = 170
TILT_MAX_DXL = 1536
STEPS_PER_RAD = 4096.0 / (2.0 * math.pi)
TILT_CENTER = 1024
SAFE_TILT_RAD_MIN = (TILT_MIN_DXL - TILT_CENTER) / STEPS_PER_RAD + 0.2
SAFE_TILT_RAD_MAX = (TILT_MAX_DXL - TILT_CENTER) / STEPS_PER_RAD - 0.2

# --- MATH HELPERS ---
def pitch_from_acceleration(x, y, z):
    """Calculates Pitch from gravity vector."""
    return math.atan2(x, math.sqrt(y*y + z*z))

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('pan_tilt_calibrator')
        
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.imu_sub = self.create_subscription(
            Imu, 
            '/livox/imu', 
            self.imu_callback, 
            qos_profile=qos_policy
        )
        
        self.pan_pub = self.create_publisher(Float64, '/pan_goal', 10)
        self.tilt_pub = self.create_publisher(Float64, '/tilt_goal', 10)

        # STATE VARIABLES
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.last_imu_time = None
        self.received_first_imu = False
        self.lock = threading.Lock()
        
        # Test Grid
        pan_degs = [-30, 0, 30]
        tilt_degs = [-20, 0, 20] 
        self.test_positions = []
        for p in pan_degs:
            for t in tilt_degs:
                self.test_positions.append( (math.radians(p), math.radians(t)) )

    def imu_callback(self, msg):
        # 1. ACCELEROMETER -> PITCH (Absolute)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        p = pitch_from_acceleration(ax, ay, az)
        
        # 2. GYROSCOPE -> YAW (Relative Integration)
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        with self.lock:
            if self.last_imu_time is not None:
                dt = current_time - self.last_imu_time
                if 0.0 < dt < 1.0:
                    self.current_yaw += msg.angular_velocity.z * dt
            
            self.last_imu_time = current_time
            self.current_pitch = p
            self.received_first_imu = True

    def reset_yaw(self):
        with self.lock:
            self.current_yaw = 0.0
            self.get_logger().info("Yaw Integration RESET to 0.0")

    def get_stable_reading_with_log(self, duration):
        """Records data for user-specified duration."""
        start_time = time.time()
        history = []
        
        # Loop until duration is met
        while time.time() - start_time < duration:
            if self.received_first_imu:
                with self.lock:
                    t_rel = time.time() - start_time
                    history.append((t_rel, self.current_pitch, self.current_yaw))
            time.sleep(0.02) # 50Hz sampling
            
        if not history: return 0.0, 0.0, []
        pitches = [h[1] for h in history]
        yaws = [h[2] for h in history]
        return sum(pitches)/len(pitches), sum(yaws)/len(yaws), history

    def move_to(self, pan_rad, tilt_rad):
        if tilt_rad < SAFE_TILT_RAD_MIN or tilt_rad > SAFE_TILT_RAD_MAX:
            return
        msg_p = Float64()
        msg_p.data = float(pan_rad)
        msg_t = Float64()
        msg_t.data = float(tilt_rad)
        for _ in range(3):
            self.pan_pub.publish(msg_p)
            self.tilt_pub.publish(msg_t)
            time.sleep(0.05)

def save_step_plot(folder, step_idx, pan_tgt_deg, tilt_tgt_deg, history, base_pitch_bias, base_yaw_ref):
    if not history: return
    times = [h[0] for h in history]
    pitches = [math.degrees(h[1] - base_pitch_bias) for h in history]
    yaws = [math.degrees(h[2] - base_yaw_ref) for h in history]
    
    plt.figure(figsize=(10, 5))
    
    plt.subplot(1, 2, 1)
    plt.plot(times, pitches, 'b', label='Pitch (Accel)')
    plt.axhline(y=tilt_tgt_deg, color='g', linestyle='--', label=f'Tgt: {tilt_tgt_deg}')
    plt.title(f"Step {step_idx}: Tilt Analysis")
    plt.grid(True)
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(times, yaws, 'r', label='Yaw (Gyro)')
    plt.axhline(y=pan_tgt_deg, color='g', linestyle='--', label=f'Tgt: {pan_tgt_deg}')
    plt.title(f"Step {step_idx}: Pan Analysis")
    plt.grid(True)
    plt.legend()
    
    filename = os.path.join(folder, f"step_{step_idx}_P{pan_tgt_deg}_T{tilt_tgt_deg}.png")
    plt.savefig(filename)
    plt.close()

def generate_summary_plot(folder, results_list, mean_tilt, mean_pan):
    t_targets = [math.degrees(r[1]) for r in results_list]
    t_offsets = [math.degrees(r[4]) for r in results_list]
    p_targets = [math.degrees(r[0]) for r in results_list]
    p_offsets = [math.degrees(r[5]) for r in results_list]

    plt.figure(figsize=(12, 6))
    
    plt.subplot(1, 2, 1)
    unique_pans = sorted(list(set(p_targets)))
    colors = plt.cm.viridis(np.linspace(0, 1, len(unique_pans)))
    for i, pan_val in enumerate(unique_pans):
        x = [x for x, p in zip(t_targets, p_targets) if p == pan_val]
        y = [y for y, p in zip(t_offsets, p_targets) if p == pan_val]
        plt.plot(x, y, marker='o', label=f'Pan {pan_val}°', color=colors[i])
    plt.axhline(y=math.degrees(mean_tilt), color='r', linestyle='--', label='Mean Tilt Offset')
    plt.title("Tilt Offset (Accel)")
    plt.xlabel("Target Tilt")
    plt.ylabel("Offset (deg)")
    plt.legend()
    plt.grid(True)

    plt.subplot(1, 2, 2)
    for i, pan_val in enumerate(unique_pans):
        x = [x for x, p in zip(p_targets, p_targets) if p == pan_val]
        y = [y for y, p in zip(p_offsets, p_targets) if p == pan_val]
        plt.plot(x, y, marker='x', linestyle='None', label=f'Pan {pan_val}°', color=colors[i])
    plt.axhline(y=math.degrees(mean_pan), color='r', linestyle='--', label='Mean Pan Offset')
    plt.title("Pan Offset (Gyro)")
    plt.xlabel("Target Pan")
    plt.ylabel("Offset (deg)")
    plt.grid(True)

    plt.savefig(os.path.join(folder, "summary_calibration.png"))

def main():
    rclpy.init()
    node = CalibrationNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    plot_folder = "calibration_plots"
    if os.path.exists(plot_folder): shutil.rmtree(plot_folder)
    os.makedirs(plot_folder)

    print("\n==================================================")
    print("   LIVOX AVIA CALIBRATION (ACCEL + GYRO TRACKING) ")
    print("==================================================")
    
    # --- NEW: ASK USER FOR TIME ---
    try:
        user_time = input("Enter sampling time per step (seconds) [Default: 3.0]: ")
        calib_time = float(user_time) if user_time.strip() else 3.0
    except ValueError:
        print("Invalid input. Using default 3.0 seconds.")
        calib_time = 3.0
    
    print(f"--> Using {calib_time} seconds per measurement.")

    print("WAITING FOR RAW DATA (/livox/imu)...")
    while not node.received_first_imu:
        time.sleep(1.0)
    print("--> DATA RECEIVED!")

    # --- PHASE 1: WORLD ZERO ---
    print("\n--------------------------------------------------")
    print("STEP 1: ALIGNMENT (Define World Zero)")
    print("   1. Place LiDAR on Base.")
    print("   2. Point it EXACTLY where you want 'Pan = 0' to be.")
    print("   3. DO NOT MOVE IT.")
    input("Press [ENTER] to LOCK ZERO reference...")
    
    print("Resetting Gyro Yaw to 0.0...")
    node.reset_yaw()
    print("Measuring Base Bias...")
    # We use the user-defined time for base bias too, or slightly longer
    base_pitch, base_yaw, _ = node.get_stable_reading_with_log(calib_time + 1.0)
    
    print(f"--> Base Pitch (Gravity): {math.degrees(base_pitch):.2f} deg")
    print(f"--> Base Yaw (Reference): {math.degrees(base_yaw):.2f} deg")

    # --- PHASE 2: MOUNTING ---
    print("\n--------------------------------------------------")
    print("STEP 2: MOUNTING")
    print("   1. Attach LiDAR to Pan-Tilt system.")
    print("   2. WARNING: The Gyro is tracking rotation!")
    input("Press [ENTER] when mounted and Motors are ON...")
    
    results = [] 
    
    for i, (tgt_pan, tgt_tilt) in enumerate(node.test_positions):
        deg_p = math.degrees(tgt_pan)
        deg_t = math.degrees(tgt_tilt)
        print(f"[{i+1}/{len(node.test_positions)}] Moving to Pan: {deg_p:.0f}, Tilt: {deg_t:.0f}...")
        
        node.move_to(tgt_pan, tgt_tilt)
        time.sleep(2.5) # Wait for mechanical settling
        
        # MEASURE using User Duration
        meas_pitch, meas_yaw, history = node.get_stable_reading_with_log(calib_time)
        save_step_plot(plot_folder, i+1, int(deg_p), int(deg_t), history, base_pitch, base_yaw)
        
        actual_pitch = meas_pitch - base_pitch
        point_tilt_offset = tgt_tilt - actual_pitch
        
        actual_yaw = meas_yaw - base_yaw
        point_pan_offset = tgt_pan - actual_yaw
        
        results.append((tgt_pan, tgt_tilt, actual_pitch, actual_yaw, point_tilt_offset, point_pan_offset))
        print(f"   -> Tilt Off: {math.degrees(point_tilt_offset):.2f}° | Pan Off: {math.degrees(point_pan_offset):.2f}°")

    node.move_to(0.0, 0.0)
    
    mean_tilt = sum([r[4] for r in results]) / len(results)
    mean_pan  = sum([r[5] for r in results]) / len(results)

    print("\n==================================================")
    print(f"Mean Tilt Offset: {mean_tilt:.5f} rad ({math.degrees(mean_tilt):.2f} deg)")
    print(f"Mean Pan Offset:  {mean_pan:.5f} rad ({math.degrees(mean_pan):.2f} deg)")
    
    generate_summary_plot(plot_folder, results, mean_tilt, mean_pan)
    
    save_path = "calibration_params.yaml"
    yaml_content = f"/**:\n  ros__parameters:\n    pan_offset_rad: {mean_pan:.5f}\n    tilt_offset_rad: {mean_tilt:.5f}\n"
    
    if input(f"\nSave yaml to '{save_path}'? (y/n): ").lower() == 'y':
        with open(save_path, 'w') as f:
            f.write(yaml_content)
        print("Saved.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()