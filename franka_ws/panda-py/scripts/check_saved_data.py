#!/usr/bin/env python3
import numpy as np
import cv2
import os
import json

# base_dir = "/home/rmx/franka_ws/panda-py/data"
base_dir = "/media/rmx/6C923B76923B43BC/data"
episode_id = 99
episode_folder = os.path.join(base_dir, f"episode_{episode_id}")

rgb_path = os.path.join(episode_folder, "rgb.npy")
pose_path = os.path.join(episode_folder, "eef_pose.npy")
instruction_path = os.path.join(episode_folder, "instruction.json")

rgb_array = np.load(rgb_path)
eef_pose_array = np.load(pose_path)

print(f"✅ Loaded episode_{episode_id}")
print(f" - RGB frames: {rgb_array.shape}")
print(f" - EEF poses:  {eef_pose_array.shape}")

if os.path.exists(instruction_path):
    with open(instruction_path, "r") as f:
        data = json.load(f)
        print(f"🗣️ Instruction: {data.get('instruction', '')}")

for i, (img, pose) in enumerate(zip(rgb_array, eef_pose_array)):
    x, y, z, qx, qy, qz, qw, width = pose

    img_display = img.copy()
    text = f"Frame {i}: Pos=({x:.3f}, {y:.3f}, {z:.3f}), Width={width*1000:.1f}mm"
    cv2.putText(img_display, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    cv2.imshow("RGB", img_display)
    key = cv2.waitKey(50)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
