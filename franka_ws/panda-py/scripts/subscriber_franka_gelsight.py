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
# import re
# import shutil

# color_array, eef_pose_array, gelsight1_array, gelsight2_array = [], [], [], []
# recording = False
# current_time = None
# # data_folder = "/home/rmx/franka_ws/panda-py/data"
# data_folder = "/media/rmx/6C923B76923B43BC/data"
# bridge = CvBridge()
# instruction = None
# episode_num = 0 
# last_delete_time = 0

# # ======================== Episode Num ========================
# def get_max_episode_num():
#     if not os.path.exists(data_folder):
#         return 0
#     pattern = re.compile(r'episode_(\d+)')
#     max_num = 0
#     for item in os.listdir(data_folder):
#         if os.path.isdir(os.path.join(data_folder, item)):
#             match = pattern.match(item)
#             if match:
#                 num = int(match.group(1))
#                 if num > max_num:
#                     max_num = num
#     return max_num


# def get_next_episode_num():
#     if not os.path.exists(data_folder):
#         os.makedirs(data_folder)
#         return 0

#     subfolders = [
#         name for name in os.listdir(data_folder)
#         if os.path.isdir(os.path.join(data_folder, name))
#     ]

#     next_num = len(subfolders)
#     rospy.loginfo(f"🧩 Detected {next_num} existing episodes. Next: episode_{next_num}")
#     return next_num



# # ======================== Save Data ========================
# def save_data(episode_folder):
#     if not os.path.exists(episode_folder):
#         os.makedirs(episode_folder)

#     rgb_file = os.path.join(episode_folder, "rgb.npy")
#     eef_pose_file = os.path.join(episode_folder, "eef_pose.npy")
#     gelsight1_file = os.path.join(episode_folder, "gelsight1.npy")
#     gelsight2_file = os.path.join(episode_folder, "gelsight2.npy")

#     np.save(rgb_file, np.array(color_array))
#     np.save(eef_pose_file, np.array(eef_pose_array))
#     np.save(gelsight1_file, np.array(gelsight1_array))
#     np.save(gelsight2_file, np.array(gelsight2_array))

#     if instruction is not None:
#         import json
#         with open(os.path.join(episode_folder, "instruction.json"), "w") as f:
#             json.dump({"instruction": instruction}, f, indent=4)

#     color_array.clear()
#     eef_pose_array.clear()
#     gelsight1_array.clear()
#     gelsight2_array.clear()
#     rospy.loginfo(f"✅ Data saved to {episode_folder}")


# # ======================== Call Back ========================
# def callback(color_msg, pose_msg, gelsight1_msg, gelsight2_msg):
#     global recording
#     if not recording:
#         return

#     # Transfer Image
#     color_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
#     # color_img = cv2.resize(color_img, (256, 256))
#     color_img = cv2.resize(color_img, (640, 384))
#     # cv2.imshow("Color", color_img)
#     # cv2.waitKey(1)
#     # print(f"{color_img.shape=}")
#     # exit()

#     # End Pose
#     pos = pose_msg.pose.position
#     ori = pose_msg.pose.orientation
#     eef_pose = np.array([pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])

#     if "|" in pose_msg.header.frame_id:
#         frame, width_str = pose_msg.header.frame_id.split("|")
#         gripper_width = float(width_str)
#         # print(f"Gripper width: {gripper_width:.4f} m")
#     else:
#         print("No width info found")

#     eef_pose = np.concatenate([eef_pose, [gripper_width]])
#     # print(f"{eef_pose=}")

#     gel1_img = bridge.imgmsg_to_cv2(gelsight1_msg, "bgr8")
#     gel2_img = bridge.imgmsg_to_cv2(gelsight2_msg, "bgr8")

#     # Save to buffer
#     color_array.append(color_img)
#     eef_pose_array.append(eef_pose)
#     gelsight1_array.append(gel1_img)
#     gelsight2_array.append(gel2_img)

#     rospy.loginfo(f"Recording... collected {len(color_array)} frames")


# # ======================== Keyboard controll ========================
# def on_press(key):
#     global recording, episode_num, last_delete_time
#     try:
#         if key.char == 'c':
#             if not recording:
#                 recording = True
#                 rospy.loginfo(f"▶ Started recording episode_{episode_num}")

#         elif key.char == 's':
#             if recording:
#                 recording = False
#                 # timestamp = time.strftime("%Y%m%d_%H%M%S")
#                 # episode_folder = os.path.join(data_folder, f"episode_{episode_num}_{timestamp}")
#                 episode_folder = os.path.join(data_folder, f"episode_{episode_num}")
#                 rospy.loginfo(f"⏹ Stopped recording, saving episode_{episode_num}...")
#                 save_data(episode_folder)
#                 episode_num = get_next_episode_num()  # 重新统计
#                 rospy.loginfo(f"Next episode index: {episode_num}")

#         elif key.char == 'd':
#             now = time.time()
#             if now - last_delete_time < 1.0:
#                 if episode_num > 0:
#                     last_episode_num = episode_num - 1
#                     candidates = [f for f in os.listdir(data_folder) if f.startswith(f"episode_{last_episode_num}")]
#                     if candidates:
#                         folder_to_delete = os.path.join(data_folder, sorted(candidates)[-1])
#                         shutil.rmtree(folder_to_delete)
#                         rospy.loginfo(f"🗑 Deleted {folder_to_delete}")
#                         episode_num = get_next_episode_num()
#                     else:
#                         rospy.logwarn(f"❗ No folder found for episode_{last_episode_num}")
#                 else:
#                     rospy.logwarn("❗ No episode available to delete.")
#             else:
#                 rospy.loginfo("⚠️ Press 'd' again within 1 second to confirm deletion.")
#             last_delete_time = now

#         elif key.char == 'q':
#             rospy.loginfo("❌ Quitting session...")
#             return False

#     except AttributeError:
#         pass


# # ======================== Main ========================
# def main():
#     global instruction, episode_num
#     rospy.init_node("rgb_pose_recorder", anonymous=True)

#     instruction = input("Input language instruction: ")
    
#     # episode_num = get_max_episode_num()
#     # rospy.loginfo(f"episode num: {episode_num}")

#     episode_num = get_next_episode_num()
#     rospy.loginfo(f"Starting new recording at episode_{episode_num}")


#     color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
#     pose_sub = message_filters.Subscriber("/franka_pose", PoseStamped)
#     gelsight1_sub = message_filters.Subscriber("/gsmini_rawimg_0", Image)
#     gelsight2_sub = message_filters.Subscriber("/gsmini_rawimg_1", Image)


#     ts = message_filters.ApproximateTimeSynchronizer(
#         [color_sub, pose_sub, gelsight1_sub, gelsight2_sub],
#         queue_size=10,
#         slop=0.05
#     )
#     ts.registerCallback(callback)

#     rospy.loginfo("🌈 RGB + Franka Pose Recorder started.")
#     rospy.loginfo("Press 'c' start, 's' stop and save, 'q' quit, 'd' delete file.")

#     listener = keyboard.Listener(on_press=on_press)
#     listener.start()
#     listener.join()

#     rospy.loginfo("Program terminated.")


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64  # 新增：用于接收夹爪宽度
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import threading
import time
from pynput import keyboard
import re
import shutil

color_array, eef_pose_array, gelsight1_array, gelsight2_array = [], [], [], []
recording = False
current_time = None
data_folder = "/media/rmx/6C923B76923B43BC/data"  # 数据保存路径
bridge = CvBridge()
instruction = None
episode_num = 0 
last_delete_time = 0

# ========================  episode编号管理（不变） ========================
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


# ======================== 数据保存（不变） ========================
def save_data(episode_folder):
    if not os.path.exists(episode_folder):
        os.makedirs(episode_folder)

    rgb_file = os.path.join(episode_folder, "rgb.npy")
    eef_pose_file = os.path.join(episode_folder, "eef_pose.npy")
    gelsight1_file = os.path.join(episode_folder, "gelsight1.npy")
    gelsight2_file = os.path.join(episode_folder, "gelsight2.npy")

    np.save(rgb_file, np.array(color_array))
    np.save(eef_pose_file, np.array(eef_pose_array))
    np.save(gelsight1_file, np.array(gelsight1_array))
    np.save(gelsight2_file, np.array(gelsight2_array))

    if instruction is not None:
        import json
        with open(os.path.join(episode_folder, "instruction.json"), "w") as f:
            json.dump({"instruction": instruction}, f, indent=4)

    color_array.clear()
    eef_pose_array.clear()
    gelsight1_array.clear()
    gelsight2_array.clear()
    rospy.loginfo(f"✅ Data saved to {episode_folder}")


# ======================== 回调函数（修改：接收宽度消息） ========================
def callback(color_msg, pose_msg, gelsight1_msg, gelsight2_msg, width_msg):  # 新增width_msg参数
    global recording
    if not recording:
        return

    # 处理彩色图像（不变）
    color_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
    color_img = cv2.resize(color_img, (640, 384))

    # 处理机械臂姿态（不变）
    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation
    eef_pose = np.array([pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])

    # 新增：从width_msg中获取夹爪宽度（替代原frame_id解析）
    gripper_width = width_msg.data  # 直接读取Float64消息的data字段
    # print(f"Gripper width: {gripper_width:.4f} m")  # 调试用

    # 拼接姿态+宽度（不变）
    eef_pose = np.concatenate([eef_pose, [gripper_width]])

    # 处理Gelsight图像（不变）
    gel1_img = bridge.imgmsg_to_cv2(gelsight1_msg, "bgr8")
    gel2_img = bridge.imgmsg_to_cv2(gelsight2_msg, "bgr8")

    # 保存到缓冲区（不变）
    color_array.append(color_img)
    eef_pose_array.append(eef_pose)
    gelsight1_array.append(gel1_img)
    gelsight2_array.append(gel2_img)

    rospy.loginfo(f"Recording... collected {len(color_array)} frames")


# ======================== 键盘控制（不变） ========================
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


# ======================== 主函数（修改：添加宽度订阅） ========================
def main():
    global instruction, episode_num
    rospy.init_node("rgb_pose_recorder", anonymous=True)

    instruction = input("Input language instruction: ")
    episode_num = get_next_episode_num()
    rospy.loginfo(f"Starting new recording at episode_{episode_num}")

    # 订阅器（新增：width_sub订阅/gripper_width）
    color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    pose_sub = message_filters.Subscriber("/franka_pose", PoseStamped)
    gelsight1_sub = message_filters.Subscriber("/gsmini_rawimg_0", Image)
    gelsight2_sub = message_filters.Subscriber("/gsmini_rawimg_1", Image)
    width_sub = message_filters.Subscriber("/gripper_width", Float64)  # 新增宽度订阅

    # 时间同步器（新增：加入width_sub，同步5个话题）
    ts = message_filters.ApproximateTimeSynchronizer(
        [color_sub, pose_sub, gelsight1_sub, gelsight2_sub, width_sub],  # 加入width_sub
        queue_size=10,
        slop=0.05,  # 允许0.05秒的时间偏差
        allow_headerless=True
    )
    ts.registerCallback(callback)  # 回调函数会接收5个消息

    rospy.loginfo("🌈 RGB + Franka Pose Recorder started.")
    rospy.loginfo("Press 'c' start, 's' stop and save, 'q' quit, 'd' delete file.")

    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    listener.join()

    rospy.loginfo("Program terminated.")


if __name__ == "__main__":
    main()
