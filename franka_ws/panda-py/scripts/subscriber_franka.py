#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import threading
import time
from pynput import keyboard
import re
import shutil

color_array, eef_pose_array = [], []
recording = False
current_time = None
data_folder = "/home/rmx/franka_ws/panda-py/data"
bridge = CvBridge()
instruction = None
episode_num = 0 
last_delete_time = 0

# ======================== Episode Num ========================
def get_max_episode_num():
    if not os.path.exists(data_folder):
        return 0
    pattern = re.compile(r'episode_(\d+)')
    max_num = 0
    for item in os.listdir(data_folder):
        if os.path.isdir(os.path.join(data_folder, item)):
            match = pattern.match(item)
            if match:
                num = int(match.group(1))
                if num > max_num:
                    max_num = num
    return max_num


def get_next_episode_num():
    if not os.path.exists(data_folder):
        os.makedirs(data_folder)
        return 0

    subfolders = [
        name for name in os.listdir(data_folder)
        if os.path.isdir(os.path.join(data_folder, name))
    ]

    next_num = len(subfolders)
    rospy.loginfo(f"🧩 Detected {next_num} existing episodes. Next: episode_{next_num}")
    return next_num



# ======================== Save Data ========================
def save_data(episode_folder):
    if not os.path.exists(episode_folder):
        os.makedirs(episode_folder)

    rgb_file = os.path.join(episode_folder, "rgb.npy")
    eef_pose_file = os.path.join(episode_folder, "eef_pose.npy")

    np.save(rgb_file, np.array(color_array))
    np.save(eef_pose_file, np.array(eef_pose_array))

    if instruction is not None:
        import json
        with open(os.path.join(episode_folder, "instruction.json"), "w") as f:
            json.dump({"instruction": instruction}, f, indent=4)

    color_array.clear()
    eef_pose_array.clear()
    rospy.loginfo(f"✅ Data saved to {episode_folder}")


# ======================== Call Back ========================
def callback(color_msg, pose_msg):
    global recording
    if not recording:
        return

    # Transfer Image
    color_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
    # color_img = cv2.resize(color_img, (256, 256))
    color_img = cv2.resize(color_img, (640, 384))
    # cv2.imshow("Color", color_img)
    # cv2.waitKey(1)
    # print(f"{color_img.shape=}")
    # exit()

    # End Pose
    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation
    eef_pose = np.array([pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])

    if "|" in pose_msg.header.frame_id:
        frame, width_str = pose_msg.header.frame_id.split("|")
        gripper_width = float(width_str)
        # print(f"Gripper width: {gripper_width:.4f} m")
    else:
        print("No width info found")

    eef_pose = np.concatenate([eef_pose, [gripper_width]])
    # print(f"{eef_pose=}")

    # Save to buffer
    color_array.append(color_img)
    eef_pose_array.append(eef_pose)

    rospy.loginfo(f"Recording... collected {len(color_array)} frames")


# ======================== Keyboard controll ========================
def on_press(key):
    global recording, episode_num, last_delete_time
    try:
        if key.char == 'c':
            if not recording:
                recording = True
                rospy.loginfo(f"▶ Started recording episode_{episode_num}")

        elif key.char == 's':
            if recording:
                recording = False
                # timestamp = time.strftime("%Y%m%d_%H%M%S")
                # episode_folder = os.path.join(data_folder, f"episode_{episode_num}_{timestamp}")
                episode_folder = os.path.join(data_folder, f"episode_{episode_num}")
                rospy.loginfo(f"⏹ Stopped recording, saving episode_{episode_num}...")
                save_data(episode_folder)
                episode_num = get_next_episode_num()  # 重新统计
                rospy.loginfo(f"Next episode index: {episode_num}")

        elif key.char == 'd':
            now = time.time()
            if now - last_delete_time < 1.0:
                if episode_num > 0:
                    last_episode_num = episode_num - 1
                    candidates = [f for f in os.listdir(data_folder) if f.startswith(f"episode_{last_episode_num}")]
                    if candidates:
                        folder_to_delete = os.path.join(data_folder, sorted(candidates)[-1])
                        shutil.rmtree(folder_to_delete)
                        rospy.loginfo(f"🗑 Deleted {folder_to_delete}")
                        episode_num = get_next_episode_num()
                    else:
                        rospy.logwarn(f"❗ No folder found for episode_{last_episode_num}")
                else:
                    rospy.logwarn("❗ No episode available to delete.")
            else:
                rospy.loginfo("⚠️ Press 'd' again within 1 second to confirm deletion.")
            last_delete_time = now

        elif key.char == 'q':
            rospy.loginfo("❌ Quitting session...")
            return False

    except AttributeError:
        pass


# ======================== Main ========================
def main():
    global instruction, episode_num
    rospy.init_node("rgb_pose_recorder", anonymous=True)

    instruction = input("Input language instruction: ")
    
    # episode_num = get_max_episode_num()
    # rospy.loginfo(f"episode num: {episode_num}")

    episode_num = get_next_episode_num()
    rospy.loginfo(f"Starting new recording at episode_{episode_num}")


    color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    pose_sub = message_filters.Subscriber("/franka_pose", PoseStamped)

    ts = message_filters.ApproximateTimeSynchronizer(
        [color_sub, pose_sub],
        queue_size=10,
        slop=0.05
    )
    ts.registerCallback(callback)

    rospy.loginfo("🌈 RGB + Franka Pose Recorder started.")
    rospy.loginfo("Press 'c' start, 's' stop and save, 'q' quit, 'd' delete file.")

    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    listener.join()

    rospy.loginfo("Program terminated.")


if __name__ == "__main__":
    main()



# #!/usr/bin/env python3
# import rospy
# import message_filters
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import PoseStamped
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import os
# import threading
# import time
# from pynput import keyboard

# # ======================== 全局变量 ========================
# color_array, eef_pose_array = [], []
# recording = False
# current_time = None
# data_folder = "/home/rmx/franka_ws/panda-py/data"   # 修改为你想保存的路径
# bridge = CvBridge()
# instruction = None
# episode_num = None

# # ======================== 数据保存函数 ========================
# def save_data(episode_folder):
#     """保存当前采集的 RGB 图像和末端位姿"""
#     if not os.path.exists(episode_folder):
#         os.makedirs(episode_folder)

#     rgb_file = os.path.join(episode_folder, "rgb.npy")
#     eef_pose_file = os.path.join(episode_folder, "eef_pose.npy")

#     np.save(rgb_file, np.array(color_array))
#     np.save(eef_pose_file, np.array(eef_pose_array))

#     # 保存语言描述（如果有）
#     if instruction is not None:
#         import json
#         with open(os.path.join(episode_folder, "instruction.json"), "w") as f:
#             json.dump({"instruction": instruction}, f, indent=4)

#     color_array.clear()
#     eef_pose_array.clear()
#     rospy.loginfo(f"✅ Data saved to {episode_folder}")


# # ======================== 回调函数 ========================
# def callback(color_msg, pose_msg):
#     global recording
#     if not recording:
#         return

#     # 转换图像
#     color_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
#     color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
#     color_img = cv2.resize(color_img, (256, 256))

#     # 提取末端位姿
#     pos = pose_msg.pose.position
#     ori = pose_msg.pose.orientation
#     eef_pose = np.array([pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])

#     # 存入缓存
#     color_array.append(color_img)
#     eef_pose_array.append(eef_pose)

#     rospy.loginfo_throttle(1.0, f"Recording... collected {len(color_array)} frames")


# # ======================== 键盘控制函数 ========================
# def on_press(key):
#     global recording, current_time, episode_num
#     try:
#         if key.char == 'c':
#             if not recording:
#                 current_time = time.strftime("%Y%m%d%H%M%S")
#                 recording = True
#                 rospy.loginfo(f"▶ Started recording episode {current_time}")

#         elif key.char == 's':
#             if recording:
#                 recording = False
#                 # episode_folder = os.path.join(data_folder, current_time)
#                 # 构造 episode_0, episode_1, ..., episode_9 的路径
#                 episode_folder = os.path.join(data_folder, f"episode_{episode_num}")
#                 print(episode_folder)
#                 rospy.loginfo("⏹ Stopped recording, saving data...")
#                 save_data(episode_folder)
#                 # episode_num = episode_num + 1

#         elif key.char == 'q':
#             rospy.loginfo("❌ Quitting session...")
#             return False  # 停止监听
#     except AttributeError:
#         pass


# # ======================== 主函数 ========================
# def main():
#     global instruction
#     rospy.init_node("rgb_pose_recorder", anonymous=True)

#     # 输入当前采集任务描述
#     instruction = input("Input language instruction: ")
#     global episode_num
#     episode_num = input("Input episode_num: ")

#     color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
#     # depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
#     pose_sub = message_filters.Subscriber("/franka_pose", PoseStamped)

#     ts = message_filters.ApproximateTimeSynchronizer(
#         # [color_sub, depth_sub, pose_sub],
#         [color_sub, pose_sub],
#         # [color_sub],
#         queue_size=10,
#         slop=0.05
#     )
#     ts.registerCallback(callback)

#     rospy.loginfo("🌈 RGB + Franka Pose Recorder started.")
#     rospy.loginfo("Press 'c' start, 's' stop and save, 'q' quit.")

#     # 键盘监听线程
#     listener = keyboard.Listener(on_press=on_press)
#     listener.start()
#     listener.join()

#     rospy.loginfo("Program terminated.")


# if __name__ == "__main__":
#     main()


###meixuan

# #!/usr/bin/env python3
# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped

# def pose_callback(msg: PoseStamped):
#     pos = msg.pose.position
#     ori = msg.pose.orientation
#     position = np.array([pos.x, pos.y, pos.z])
#     orientation = np.array([ori.x, ori.y, ori.z, ori.w])
    
#     rospy.loginfo(f"Franka pose → pos: {position.round(4)}, quat: {orientation.round(4)}")

# def listener():
#     rospy.init_node('franka_pose_listener')
#     rospy.Subscriber("/franka_pose", PoseStamped, pose_callback)
#     rospy.spin()

# if __name__ == '__main__':
#     listener()




# #!/usr/bin/env python3
# import rospy
# import message_filters
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2

# def callback(color_msg, depth_msg):
#     bridge = CvBridge()
    
#     color_img = bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
#     depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    
#     rospy.loginfo(f"Received synchronized images at time {color_msg.header.stamp.to_sec():.4f}")
#     cv2.imshow("Color", color_img)
#     cv2.imshow("Depth", depth_img / 4000.0)
#     # cv2.imshow("Depth", depth_img)
#     cv2.waitKey(1)

# def main():

#     rospy.init_node('rgbd_sync_listener', anonymous=True)

#     color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
#     depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)

#     ts = message_filters.ApproximateTimeSynchronizer(
#         [color_sub, depth_sub],
#         queue_size=10,
#         slop=0.05
#         # slop=0.2
#     )
#     ts.registerCallback(callback)

#     rospy.loginfo("RGB-D Sync Node started.")
#     rospy.spin()

# if __name__ == '__main__':
#     main()


# Depth to RGB extrisinc
# ---
# header: 
#   seq: 0
#   stamp: 
#     secs: 0
#     nsecs:         0
#   frame_id: "depth_to_color_extrinsics"
# rotation: [0.9999316930770874, -0.011597518809139729, -0.0014691469259560108, 0.011598814278841019, 0.9999323487281799, 0.0008767668041400611, 0.001458879210986197, -0.0008937472593970597, 0.9999985098838806]
# translation: [-0.024984689712524415, -0.0003068108558654785, 5.3048256784677503e-05]
# ---



# #!/usr/bin/env python3
# import rospy
# import message_filters
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import PoseStamped
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# def callback(color_msg, depth_msg, pose_msg):
#     bridge = CvBridge()

#     # --- 图像解码 ---
#     color_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
#     # depth_img = bridge.imgmsg_to_cv2(depth_msg, "passthrough")

#     # # --- 深度图归一化可视化 ---
#     # depth_vis = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
#     # depth_vis = depth_vis.astype(np.uint8)
#     # depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

#     # --- 位姿信息 ---
#     pos = pose_msg.pose.position
#     ori = pose_msg.pose.orientation
#     position = np.array([pos.x, pos.y, pos.z])
#     orientation = np.array([ori.x, ori.y, ori.z, ori.w])

#     # --- 打印信息 ---
#     rospy.loginfo(f"[{color_msg.header.stamp.to_sec():.3f}] "
#                   f"Franka pos: {position.round(4)}, quat: {orientation.round(4)}")

#     # # --- 显示图像 ---
#     # cv2.imshow("Color", color_img)
#     # cv2.imshow("Depth", depth_img / 4000.0)
#     # cv2.waitKey(1)


# def main():
#     rospy.init_node("rgbd_pose_viewer", anonymous=True)

#     color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
#     depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
#     pose_sub  = message_filters.Subscriber("/franka_pose", PoseStamped)

#     # --- 时间同步（允许 50ms 时间差）---
#     ts = message_filters.ApproximateTimeSynchronizer(
#         [color_sub, depth_sub, pose_sub],
#         queue_size=10,
#         slop=0.05
#     )
#     ts.registerCallback(callback)

#     rospy.loginfo("RGB-D + Franka Pose Viewer started.")
#     rospy.spin()


# if __name__ == "__main__":
#     main()



# #!/usr/bin/env python3
# import rospy
# import message_filters
# from sensor_msgs.msg import Image, PointCloud2
# from geometry_msgs.msg import PoseStamped
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import sensor_msgs.point_cloud2 as pc2


# def callback(color_msg, depth_msg, pose_msg, cloud_msg):
#     bridge = CvBridge()

#     # --- 图像解码 ---
#     color_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
#     depth_img = bridge.imgmsg_to_cv2(depth_msg, "passthrough")

#     # --- 深度图可视化 ---
#     depth_vis = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
#     depth_vis = depth_vis.astype(np.uint8)
#     depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

#     # --- 位姿信息 ---
#     pos = pose_msg.pose.position
#     ori = pose_msg.pose.orientation
#     position = np.array([pos.x, pos.y, pos.z])
#     orientation = np.array([ori.x, ori.y, ori.z, ori.w])

#     # --- 读取点云 ---
#     cloud_points = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))
#     num_points = len(cloud_points)

#     if num_points > 0:
#         # 取部分点进行统计
#         cloud_np = np.array(cloud_points)
#         mean_point = np.mean(cloud_np, axis=0)
#     else:
#         mean_point = np.array([np.nan, np.nan, np.nan])

#     # --- 打印信息 ---
#     rospy.loginfo(f"[{color_msg.header.stamp.to_sec():.3f}] "
#                   f"Franka pos: {position.round(4)}, quat: {orientation.round(4)}, "
#                   f"PointCloud: {num_points} pts, mean={mean_point.round(4)}")

#     # # --- 显示图像 ---
#     # cv2.imshow("Color", color_img)
#     # cv2.imshow("Depth", depth_img / 4000.0)
#     # cv2.waitKey(1)


# def main():
#     rospy.init_node("rgbd_pose_pointcloud_viewer", anonymous=True)

#     # --- 订阅话题 ---
#     color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
#     depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
#     pose_sub  = message_filters.Subscriber("/franka_pose", PoseStamped)
#     cloud_sub = message_filters.Subscriber("/camera/depth/points", PointCloud2)

#     # --- 时间同步（允许 50ms 时间差）---
#     ts = message_filters.ApproximateTimeSynchronizer(
#         [color_sub, depth_sub, pose_sub, cloud_sub],
#         queue_size=10,
#         slop=0.05
#     )
#     ts.registerCallback(callback)

#     rospy.loginfo("RGB-D + Franka Pose + PointCloud Viewer started.")
#     rospy.spin()


# if __name__ == "__main__":
#     main()
