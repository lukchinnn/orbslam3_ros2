#!/usr/bin/env python3
import yaml
import os

def extract_calibration():
    # Read calibration files
    left_file = os.path.expanduser('/home/lukchin/Downloads/left.yaml')
    right_file = os.path.expanduser('/home/lukchin/Downloads/right.yaml')
    
    with open(left_file, 'r') as f:
        left_cal = yaml.safe_load(f)
    with open(right_file, 'r') as f:
        right_cal = yaml.safe_load(f)
    
    # Extract parameters
    left_K = left_cal['camera_matrix']['data']
    right_K = right_cal['camera_matrix']['data']
    left_D = left_cal['distortion_coefficients']['data']
    right_D = right_cal['distortion_coefficients']['data']
    
    print("=== Copy these values to your ORB-SLAM3 config ===")
    print("\n# Left Camera")
    print(f"Camera1.fx: {left_K[0]:.1f}")
    print(f"Camera1.fy: {left_K[4]:.1f}")
    print(f"Camera1.cx: {left_K[2]:.1f}")
    print(f"Camera1.cy: {left_K[5]:.1f}")
    print(f"Camera1.k1: {left_D[0]:.6f}")
    print(f"Camera1.k2: {left_D[1]:.6f}")
    print(f"Camera1.p1: {left_D[2]:.6f}")
    print(f"Camera1.p2: {left_D[3]:.6f}")
    
    print("\n# Right Camera")
    print(f"Camera2.fx: {right_K[0]:.1f}")
    print(f"Camera2.fy: {right_K[4]:.1f}")
    print(f"Camera2.cx: {right_K[2]:.1f}")
    print(f"Camera2.cy: {right_K[5]:.1f}")
    print(f"Camera2.k1: {right_D[0]:.6f}")
    print(f"Camera2.k2: {right_D[1]:.6f}")
    print(f"Camera2.p1: {right_D[2]:.6f}")
    print(f"Camera2.p2: {right_D[3]:.6f}")
    
    # Calculate baseline*fx for different baselines
    fx = left_K[0]
    print(f"\n# Baseline calculations (fx = {fx:.1f}):")
    for baseline_mm in [30, 40, 50, 60]:
        bf = baseline_mm * fx
        print(f"For {baseline_mm}mm baseline: Camera.bf: {bf:.0f}")

if __name__ == '__main__':
    extract_calibration()