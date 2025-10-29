# #!/usr/bin/env python3
# import numpy as np
# import cv2
# import os
# import json

# # base_dir = "/home/rmx/franka_ws/panda-py/data"
# base_dir = "/media/rmx/6C923B76923B43BC/data"
# episode_id = 3
# episode_folder = os.path.join(base_dir, f"episode_{episode_id}")

# rgb_path = os.path.join(episode_folder, "rgb.npy")
# pose_path = os.path.join(episode_folder, "eef_pose.npy")
# gelsight1_path = os.path.join(episode_folder, "gelsight1.npy")
# gelsight2_path = os.path.join(episode_folder, "gelsight2.npy")
# instruction_path = os.path.join(episode_folder, "instruction.json")

# rgb_array = np.load(rgb_path)
# eef_pose_array = np.load(pose_path)
# gelsight1_array = np.load(gelsight1_path)
# gelsight2_array = np.load(gelsight2_path)

# print(f"✅ Loaded episode_{episode_id}")
# print(f" - RGB frames: {rgb_array.shape}")
# print(f" - EEF poses:  {eef_pose_array.shape}")
# print(f" - GEL1 frames: {gelsight1_array.shape}")
# print(f" - GEL2 frames:  {gelsight2_array.shape}")

# if os.path.exists(instruction_path):
#     with open(instruction_path, "r") as f:
#         data = json.load(f)
#         print(f"🗣️ Instruction: {data.get('instruction', '')}")

# for i, (img, pose) in enumerate(zip(rgb_array, eef_pose_array)):
#     x, y, z, qx, qy, qz, qw, width = pose

#     img_display = img.copy()
#     text = f"Frame {i}: Pos=({x:.3f}, {y:.3f}, {z:.3f}), Width={width*1000:.1f}mm"
#     cv2.putText(img_display, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

#     cv2.imshow("RGB", img_display)
#     key = cv2.waitKey(50)
#     if key == ord('q'):
#         break

# cv2.destroyAllWindows()

#!/usr/bin/env python3
import numpy as np
import cv2
import os
import json

# 数据路径（可根据需要修改）
base_dir = "/media/rmx/6C923B76923B43BC/data"
episode_id = 3
episode_folder = os.path.join(base_dir, f"episode_{episode_id}")

# 加载数据文件
rgb_path = os.path.join(episode_folder, "rgb.npy")
pose_path = os.path.join(episode_folder, "eef_pose.npy")
gelsight1_path = os.path.join(episode_folder, "gelsight1.npy")
gelsight2_path = os.path.join(episode_folder, "gelsight2.npy")
instruction_path = os.path.join(episode_folder, "instruction.json")

# 加载数据
rgb_array = np.load(rgb_path)
eef_pose_array = np.load(pose_path)
gelsight1_array = np.load(gelsight1_path)
gelsight2_array = np.load(gelsight2_path)

# 打印数据信息
print(f"✅ Loaded episode_{episode_id}")
print(f" - RGB frames:    {rgb_array.shape}")
print(f" - EEF poses:     {eef_pose_array.shape}")
print(f" - GEL1 frames:   {gelsight1_array.shape}")
print(f" - GEL2 frames:   {gelsight2_array.shape}")
if os.path.exists(instruction_path):
    with open(instruction_path, "r") as f:
        data = json.load(f)
        print(f"🗣️ Instruction: {data.get('instruction', '')}")

# 确保三张图尺寸一致（若Gelsight图尺寸不同，统一缩放为RGB图尺寸）
# 获取RGB图尺寸（高度、宽度）
rgb_h, rgb_w = rgb_array[0].shape[:2]
# 定义缩放函数：将Gelsight图缩放到与RGB图相同宽度（保持比例）
def resize_to_match_width(img, target_w):
    h, w = img.shape[:2]
    scale = target_w / w  # 按宽度比例缩放
    target_h = int(h * scale)
    return cv2.resize(img, (target_w, target_h))

# 逐帧显示三张图片
for i, (rgb_img, gel1_img, gel2_img, pose) in enumerate(zip(
    rgb_array, gelsight1_array, gelsight2_array, eef_pose_array
)):
    # 1. 统一图片尺寸（将Gelsight图缩放到与RGB图宽度一致）
    gel1_resized = resize_to_match_width(gel1_img, rgb_w)
    gel2_resized = resize_to_match_width(gel2_img, rgb_w)

    # 2. 横向拼接三张图（RGB + Gelsight1 + Gelsight2）
    # 若高度不一致，用黑色边框补全（确保拼接后高度统一）
    max_h = max(rgb_h, gel1_resized.shape[0], gel2_resized.shape[0])
    
    # 给每张图补黑边（上下补全到最大高度）
    def pad_to_max_height(img, max_h):
        h, w = img.shape[:2]
        if h < max_h:
            top_pad = (max_h - h) // 2  # 上补边高度
            bottom_pad = max_h - h - top_pad  # 下补边高度
            # 补黑边（BGR格式，黑色为(0,0,0)）
            img_padded = cv2.copyMakeBorder(
                img, top_pad, bottom_pad, 0, 0,
                cv2.BORDER_CONSTANT, value=(0, 0, 0)
            )
            return img_padded
        return img

    rgb_padded = pad_to_max_height(rgb_img, max_h)
    gel1_padded = pad_to_max_height(gel1_resized, max_h)
    gel2_padded = pad_to_max_height(gel2_resized, max_h)

    # 横向拼接（水平排列）
    combined_img = cv2.hconcat([rgb_padded, gel1_padded, gel2_padded])

    # 3. 添加文字信息（姿态、宽度、帧号）
    x, y, z, qx, qy, qz, qw, width = pose
    # 文字内容（显示在拼接图顶部）
    text1 = f"Frame {i:03d} | Pos: ({x:.3f}, {y:.3f}, {z:.3f})"
    text2 = f"Gripper Width: {width*1000:.1f}mm | Press 'q' to quit"
    # 绘制文字（绿色字体，加粗）
    cv2.putText(combined_img, text1, (20, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    cv2.putText(combined_img, text2, (20, 70), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

    # 4. 显示拼接后的图片
    cv2.imshow("Combined View: RGB | Gelsight1 | Gelsight2", combined_img)
    
    # 控制播放速度（50ms/帧，约20fps；可修改数值调整速度，0为手动按帧播放）
    key = cv2.waitKey(50)
    if key == ord('q'):  # 按'q'退出播放
        break

# 关闭所有窗口
cv2.destroyAllWindows()
