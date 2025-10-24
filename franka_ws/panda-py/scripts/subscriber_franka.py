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



#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

def callback(color_msg, depth_msg, pose_msg):
    bridge = CvBridge()

    # --- 图像解码 ---
    color_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
    depth_img = bridge.imgmsg_to_cv2(depth_msg, "passthrough")

    # --- 深度图归一化可视化 ---
    depth_vis = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
    depth_vis = depth_vis.astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

    # --- 位姿信息 ---
    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation
    position = np.array([pos.x, pos.y, pos.z])
    orientation = np.array([ori.x, ori.y, ori.z, ori.w])

    # --- 打印信息 ---
    rospy.loginfo(f"[{color_msg.header.stamp.to_sec():.3f}] "
                  f"Franka pos: {position.round(4)}, quat: {orientation.round(4)}")

    # # --- 显示图像 ---
    # cv2.imshow("Color", color_img)
    # cv2.imshow("Depth", depth_color)
    # cv2.waitKey(1)


def main():
    rospy.init_node("rgbd_pose_viewer", anonymous=True)

    color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
    pose_sub  = message_filters.Subscriber("/franka_pose", PoseStamped)

    # --- 时间同步（允许 50ms 时间差）---
    ts = message_filters.ApproximateTimeSynchronizer(
        [color_sub, depth_sub, pose_sub],
        queue_size=10,
        slop=0.05
    )
    ts.registerCallback(callback)

    rospy.loginfo("RGB-D + Franka Pose Viewer started.")
    rospy.spin()


if __name__ == "__main__":
    main()
