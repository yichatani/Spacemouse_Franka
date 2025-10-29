#!/usr/bin/env python3
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from panda_py import Panda
from panda_py.controllers import CartesianImpedance
from panda_py import constants as C
from panda_py.libfranka import Gripper
from space_mouse import Spacemouse

import multiprocessing as mp
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64  # Float64无header，仅含data字段

# ================== 全局参数（不变） ==================
HOST = "172.16.0.2"
CTRL_FREQ = 200.0    
PUB_FREQ = 30.0      
LIN_VEL_MAX = 0.1
ANG_VEL_MAX = 0.8
DEADMAN_BTN = 0
ESTOP_BTN = 1

NEUTRAL_ENTER = 0.08
NEUTRAL_EXIT  = 0.12

XYZ_MIN = np.array([-2.00, -2.00, 0.000])
XYZ_MAX = np.array([2.00,  2.00, 0.60])

ANG_VEL_DEADZONE = 0.05
PERIOD = 1.0 / CTRL_FREQ
PUB_PERIOD = 1.0 / PUB_FREQ

MAX_STEP_LIN = 0.004      
MAX_STEP_ANG = 0.010      
NEUTRAL_MICRO = 0.03      
MICRO_RELEASE_FRAMES = 6  
OVERRUN_FACTOR = 2.0      
# ======================================================

# ================== 工具函数（不变） ==================
def clamp(x, lo, hi):
    return np.minimum(np.maximum(x, lo), hi)

def quat_multiply_scalar_last(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*x2 - z1*z2
    ])

def axisangle_to_quat_scalar_last(axis, angle):
    if angle < 1e-12:
        return np.array([0,0,0,1.0])
    half = 0.5*angle
    s = np.sin(half)
    return np.array([axis[0]*s, axis[1]*s, axis[2]*s, np.cos(half)])

def freeze_to_measured_pose(robot):
    pos_m = robot.get_position().astype(float).reshape(3)
    quat_m = robot.get_orientation(scalar_first=False).astype(float).reshape(4)
    return pos_m, quat_m

# ----------------- 异步发布进程（修复：删除宽度消息的header） -----------------
def pose_width_publisher_proc(q: mp.Queue, stop_evt: mp.Event):
    rospy.init_node("franka_state_pub", anonymous=True, disable_signals=True)
    # 1. 发布姿态话题（含header）
    pose_pub = rospy.Publisher("/franka_pose", PoseStamped, queue_size=10)
    # 2. 发布宽度话题（Float64，仅含data）
    width_pub = rospy.Publisher("/gripper_width", Float64, queue_size=10)
    
    rate = rospy.Rate(PUB_FREQ)  # 同频30Hz发布
    pose_msg = PoseStamped()     # 姿态消息含header
    width_msg = Float64()        # 宽度消息仅含data字段
    pose_msg.header.frame_id = "panda_link0"

    last_sample = None

    while not rospy.is_shutdown() and not stop_evt.is_set():
        # 读取队列最新数据
        drained = False
        sample = None
        while True:
            try:
                sample = q.get_nowait()  # 格式：(时间戳, pos, quat, 实时宽度)
                drained = True
            except Exception:
                break
        if drained:
            last_sample = sample

        if last_sample is not None:
            t_sec, pos, quat, realtime_width = last_sample
            ros_time = rospy.Time.from_sec(t_sec)

            # 发布姿态（含时间戳，不变）
            pose_msg.header.stamp = ros_time
            pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = pos
            pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = quat
            pose_pub.publish(pose_msg)

            width_msg.data = realtime_width  # 直接给宽度数值
            width_pub.publish(width_msg)     # 发布纯数值宽度

        rate.sleep()

# ----------------- 夹爪动作后台线程（不变） -----------------
def gripper_action_thread(gripper, action_type):
    try:
        if action_type == "close":
            gripper.grasp(
                width=0.00, speed=0.05, force=1.0,
                epsilon_inner=0.03, epsilon_outer=0.03
            )
        elif action_type == "open":
            gripper.move(0.04, 0.05)
    except Exception as e:
        print(f"[WARN] 夹爪{action_type}动作失败: {e}")

# ----------------- 主控程序（不变） -----------------
def main():
    data_queue = mp.Queue(maxsize=1)
    stop_event = mp.Event()
    pub_proc = mp.Process(
        target=pose_width_publisher_proc,
        args=(data_queue, stop_event),
        daemon=True
    )
    pub_proc.start()

    robot = Panda(HOST)
    print("<<< Wait until move to initial position >>>")
    robot.move_to_joint_position(
        [0.16492545, 0.41526617, -0.25812275, -1.99572812, 0.09467096, 2.39399699, 0.63381358],
        speed_factor=0.05
    )
    ctrl = CartesianImpedance()
    robot.start_controller(ctrl)

    try:
        print("<<< Start control! Press Ctrl+C to exit safely >>>")
        pos0, quat0 = freeze_to_measured_pose(robot)
        q_null = robot.q
        ctrl.set_control(pos0, quat0, q_nullspace=q_null)

        # 夹爪初始化
        gripper = Gripper(HOST)
        gripper.move(0.04, 0.05)
        gripper_width = gripper.read_once().width
        gripper_thread = None
        is_gripper_running = False

        # 主循环变量
        v_state = np.zeros(3)
        w_state = np.zeros(3)
        neutral_mode = True
        pos_hold = None
        quat_hold = None
        micro_cnt = 0
        last_pub_time = time.time()

        with robot.create_context(CTRL_FREQ) as ctx, Spacemouse(max_value=500, deadzone=0.2, scale_factor=1.0) as sm:
            pos = robot.get_position().astype(float).reshape(3)
            quat = robot.get_orientation(scalar_first=False).astype(float).reshape(4)
            q_null = C.JOINT_POSITION_START
            sm.lock_rotation_axes(roll=False, pitch=False, yaw=True)
            last_time = ctx.time

            while ctx.ok():
                frame_start = ctx.time
                dt = max(1e-3, frame_start - last_time)
                last_time = frame_start

                # 1. 实时读取夹爪宽度（每帧200Hz）
                try:
                    gripper_state = gripper.read_once()
                    gripper_width = gripper_state.width
                except Exception as e:
                    print(f"[WARN] 读取夹爪宽度失败: {e}")

                # 2. 夹爪动作线程控制
                if gripper_thread is not None and not gripper_thread.is_alive():
                    is_gripper_running = False
                    gripper_thread = None

                close_pressed = sm.is_button_pressed(0)
                open_pressed = sm.is_button_pressed(1)

                if close_pressed and not is_gripper_running:
                    gripper_thread = threading.Thread(
                        target=gripper_action_thread,
                        args=(gripper, "close"),
                        daemon=True
                    )
                    gripper_thread.start()
                    is_gripper_running = True
                elif open_pressed and not is_gripper_running:
                    gripper_thread = threading.Thread(
                        target=gripper_action_thread,
                        args=(gripper, "open"),
                        daemon=True
                    )
                    gripper_thread.start()
                    is_gripper_running = True

                # 3. 中性模式与姿态控制
                u = sm.get_motion_state_transformed()
                u_level = np.max(np.abs(u))

                if neutral_mode:
                    if u_level > NEUTRAL_EXIT:
                        neutral_mode = False
                        v_state[:] = 0.0; w_state[:] = 0.0
                        if pos_hold is not None:
                            pos = pos_hold.copy()
                            quat = quat_hold.copy()
                        else:
                            pos, quat = freeze_to_measured_pose(robot)
                        pos_hold = None; quat_hold = None
                    else:
                        if pos_hold is None:
                            pos_hold = robot.get_position().astype(float).reshape(3)
                            quat_hold = robot.get_orientation(scalar_first=False).astype(float).reshape(4)
                        q_null = robot.q
                        ctrl.set_control(pos_hold, quat_hold, q_nullspace=q_null)

                        # 发布数据
                        now_wall = time.time()
                        if now_wall - last_pub_time >= PUB_PERIOD:
                            sample = (now_wall, pos_hold.copy(), quat_hold.copy(), gripper_width)
                            if data_queue.full():
                                data_queue.get_nowait()
                            data_queue.put_nowait(sample)
                            last_pub_time = now_wall

                        remain = PERIOD - (ctx.time - frame_start)
                        if remain > 0:
                            time.sleep(remain)
                        continue
                else:
                    if u_level < NEUTRAL_ENTER:
                        neutral_mode = True
                        pos_hold = robot.get_position().astype(float).reshape(3)
                        quat_hold = robot.get_orientation(scalar_first=False).astype(float).reshape(4)
                        v_state[:] = 0.0; w_state[:] = 0.0
                        q_null = robot.q
                        ctrl.set_control(pos_hold, quat_hold, q_nullspace=q_null)

                        # 发布数据
                        now_wall = time.time()
                        if now_wall - last_pub_time >= PUB_PERIOD:
                            sample = (now_wall, pos_hold.copy(), quat_hold.copy(), gripper_width)
                            if data_queue.full():
                                data_queue.get_nowait()
                            data_queue.put_nowait(sample)
                            last_pub_time = now_wall

                        remain = PERIOD - (ctx.time - frame_start)
                        if remain > 0:
                            time.sleep(remain)
                        continue
                    else:
                        pos_hold = None; quat_hold = None

                # 4. 速度与姿态更新
                if u_level < NEUTRAL_MICRO:
                    micro_cnt += 1
                    if micro_cnt >= MICRO_RELEASE_FRAMES:
                        v_state[:] = 0.0
                        w_state[:] = 0.0
                else:
                    micro_cnt = 0

                v_cmd = np.clip(u[:3] * LIN_VEL_MAX, -LIN_VEL_MAX, LIN_VEL_MAX)
                w_cmd = np.clip(u[3:] * ANG_VEL_MAX, -ANG_VEL_MAX, ANG_VEL_MAX)

                if np.linalg.norm(w_cmd) < ANG_VEL_DEADZONE:
                    w_cmd[:] = 0.0

                tau_on_lin, tau_off_lin = 0.05, 0.03
                tau_on_rot, tau_off_rot = 0.05, 0.02
                alpha_v = np.clip(dt / (tau_on_lin if np.any(v_cmd) else tau_off_lin), 0.0, 1.0)
                alpha_w = np.clip(dt / (tau_on_rot if np.any(w_cmd) else tau_off_rot), 0.0, 1.0)

                v_state = (1 - alpha_v) * v_state + alpha_v * v_cmd
                w_state = (1 - alpha_w) * w_state + alpha_w * w_cmd

                step = v_state * dt
                at_min = pos <= XYZ_MIN + 1e-6
                at_max = pos >= XYZ_MAX - 1e-6
                push_out = ((step < 0) & at_min) | ((step > 0) & at_max)
                step[push_out] = 0.0
                step = np.clip(step, -MAX_STEP_LIN, MAX_STEP_LIN)
                pos = clamp(pos + step, XYZ_MIN, XYZ_MAX)

                ang = np.linalg.norm(w_state) * dt
                if ang > 1e-12:
                    ang = min(ang, MAX_STEP_ANG)
                    axis = w_state / (np.linalg.norm(w_state) + 1e-12)
                    dq = axisangle_to_quat_scalar_last(axis, ang)
                    quat = quat_multiply_scalar_last(quat, dq)
                    quat = quat / np.linalg.norm(quat)

                q_null = robot.q
                ctrl.set_control(pos, quat, q_nullspace=q_null)

                # 5. 发布数据
                now_wall = time.time()
                if now_wall - last_pub_time >= PUB_PERIOD:
                    sample = (now_wall, pos.copy(), quat.copy(), gripper_width)
                    if data_queue.full():
                        data_queue.get_nowait()
                    data_queue.put_nowait(sample)
                    last_pub_time = now_wall

                remain = PERIOD - (ctx.time - frame_start)
                if remain > 0:
                    time.sleep(remain)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected, stopping safely ...")
    except Exception as e:
        print(f"[ERROR] Unexpected exception: {e}")
    finally:
        print("[INFO] Stopping publisher process and controller...")
        stop_event.set()
        try:
            if pub_proc.is_alive():
                pub_proc.join(timeout=1.0)
        except Exception:
            pass
        try:
            robot.stop_controller()
        except Exception:
            pass
        time.sleep(0.3)
        print("[INFO] Teleop terminated cleanly.")

if __name__ == "__main__":
    main()
