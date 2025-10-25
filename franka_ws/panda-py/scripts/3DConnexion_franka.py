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
import rospy
from geometry_msgs.msg import PoseStamped

HOST = "172.16.0.2"
CTRL_FREQ = 200.0    # Python侧发送频率（Hz)
LIN_VEL_MAX = 0.1
ANG_VEL_MAX = 0.8
DEADMAN_BTN = 0
ESTOP_BTN = 1

NEUTRAL_ENTER = 0.08
NEUTRAL_EXIT  = 0.12

XYZ_MIN = np.array([-2.00, -2.00, 0.001])
XYZ_MAX = np.array([2.00,  2.00, 0.60])

ANG_VEL_DEADZONE = 0.05
PERIOD = 1.0 / CTRL_FREQ

# ================== 安全护栏参数 ==================
MAX_STEP_LIN = 0.004      # 每 tick 最大线位移（m）
MAX_STEP_ANG = 0.010      # 每 tick 最大角位移（rad）
NEUTRAL_MICRO = 0.03      # 微输入阈值
MICRO_RELEASE_FRAMES = 6  # 微输入连续帧数（约30ms@200Hz）
OVERRUN_FACTOR = 2.0      # dt 超过 2×周期视为超时
# =================================================

def clamp(x, lo, hi):
    return np.minimum(np.maximum(x, lo), hi)

def quat_multiply_scalar_last(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
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

# ----------------- 异步发布进程 -----------------
def pose_publisher_proc(q: mp.Queue, stop_evt: mp.Event, pub_hz: float = 30.0):
    """
    独立进程：从队列取 (t, pos[3], quat[4])，以 pub_hz 发布到 /franka_pose
    只保留最新帧，永不阻塞控制环
    """
    rospy.init_node("franka_state_pub", anonymous=True, disable_signals=True)
    pub = rospy.Publisher("/franka_pose", PoseStamped, queue_size=10)
    rate = rospy.Rate(pub_hz)
    msg = PoseStamped()
    msg.header.frame_id = "panda_link0"

    # 上一次取到的数据（防止队列暂时为空时断流）
    last_sample = None

    while not rospy.is_shutdown() and not stop_evt.is_set():
        # 尝试“掏空队列”，只保留最新
        drained = False
        sample = None
        while True:
            try:
                sample = q.get_nowait()  # (t, pos, quat)
                drained = True
            except Exception:
                break
        if drained:
            last_sample = sample

        if last_sample is not None:
            # t_sec, pos, quat = last_sample
            t_sec, pos, quat, width = last_sample
            msg.header.stamp = rospy.Time.from_sec(t_sec)

            msg.header.frame_id = f"panda_link0|{width:.4f}"

            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quat
            pub.publish(msg)

        rate.sleep()

# ----------------- 主控程序 -----------------
def main():
    # multiprocessing：Linux 下默认 fork，直接可用；若在其他平台需设置 mp.set_start_method('spawn')
    q = mp.Queue(maxsize=1)     # 仅保留最新一帧
    stop_evt = mp.Event()
    pub_hz = 30.0
    proc = mp.Process(target=pose_publisher_proc, args=(q, stop_evt, pub_hz), daemon=True)
    proc.start()

    robot = Panda(HOST)
    print("<<< Wait until move to initial position >>>")
    robot.move_to_joint_position([ 0.16492545,  0.41526617, -0.25812275, -1.99572812,  0.09467096, 2.39399699,  0.63381358],
                                 speed_factor=0.05)
    ctrl = CartesianImpedance()
    robot.start_controller(ctrl)
    # exit()
    try:
        print("<<< Start control! Press Ctrl+C to exit safely >>>")
        pos0, quat0 = freeze_to_measured_pose(robot)
        # print(f"{pos0=},{quat0=}")
        # exit()
        q_null = robot.q
        # q_null=array([ 0.1657624 ,  0.4105261 , -0.26013412, -1.99558975,  0.0918271 ,2.3915283 ,  0.63938177])
        # quat0 = np.array([1, 0, 0, 0])
        ctrl.set_control(pos0, quat0, q_nullspace=q_null)
        # print(f"{pos0=},{quat0=}")
        # print(f"{robot.q=}")
        # exit()
        v_state = np.zeros(3)
        w_state = np.zeros(3)
        neutral_mode = True
        pos_hold = None; quat_hold = None

        gripper = Gripper(HOST)
        gripper.move(0.04, 0.05)
        gripper_state = gripper.read_once()
        gripper_width = gripper_state.width
        micro_cnt = 0

        with robot.create_context(CTRL_FREQ) as ctx, Spacemouse(max_value=500, deadzone=0.2, scale_factor=1.0) as sm:
            pos = robot.get_position().astype(float).reshape(3)
            quat = robot.get_orientation(scalar_first=False).astype(float).reshape(4)
            q_null = C.JOINT_POSITION_START
            sm.lock_rotation_axes(roll=False, pitch=False, yaw=True)

            last_time = ctx.time
            last_pub_time = time.time()  # 仅用于限频（队列发送），不影响控制
            pub_period = 1.0 / pub_hz

            while ctx.ok():
                frame_start = ctx.time
                now = frame_start
                dt = max(1e-3, now - last_time)
                last_time = now

                if dt > OVERRUN_FACTOR * PERIOD:
                    v_state[:] = 0.0
                    w_state[:] = 0.0
                    pos, quat = freeze_to_measured_pose(robot)

                u = sm.get_motion_state_transformed()
                u_level = np.max(np.abs(u))

                close_pressed = sm.is_button_pressed(0)
                open_pressed  = sm.is_button_pressed(1)
                if close_pressed:
                    try:
                        # 避免打印卡顿，可在需要时打开
                        # print("Closing and grasping...")
                        _ = gripper.grasp(width=0.00, speed=0.05, force=1.0, epsilon_inner=0.03, epsilon_outer=0.03)
                        # print(f"{gripper_state.width=}")
                        # exit()
                        # gripper.move(gripper_state.width-0.01, 2.0)
                        # print(f"{gripper_state.is_grasped=}")
                        # gripper.move(0.0, 0.05)
                        gripper_state = gripper.read_once()
                        gripper_width = gripper_state.width
                    except Exception:
                        print("[WARN] Gripper move blocked (maybe object inside). Ignoring.")
                elif open_pressed:
                    # print("Opening gripper...")
                    try:
                        gripper.move(0.04, 0.05)
                        gripper_state = gripper.read_once()
                        gripper_width = gripper_state.width
                    except Exception:
                        print("[WARN] Gripper move failed to open fully.")

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

                        # 限频发送最新姿态到队列（异步发布）
                        now_wall = time.time()
                        if now_wall - last_pub_time >= pub_period:
                            sample = (now_wall, pos_hold.copy(), quat_hold.copy(), gripper_width)
                            try:
                                # 丢掉旧帧，只保留最新
                                if q.full():
                                    _ = q.get_nowait()
                                q.put_nowait(sample)
                            except Exception:
                                pass
                            last_pub_time = now_wall

                        remain = PERIOD - (ctx.time - frame_start)
                        if remain > 0: time.sleep(remain)
                        continue
                else:
                    if u_level < NEUTRAL_ENTER:
                        neutral_mode = True
                        pos_hold = robot.get_position().astype(float).reshape(3)
                        quat_hold = robot.get_orientation(scalar_first=False).astype(float).reshape(4)
                        v_state[:] = 0.0; w_state[:] = 0.0
                        q_null = robot.q
                        ctrl.set_control(pos_hold, quat_hold, q_nullspace=q_null)

                        # 限频发送最新姿态到队列
                        now_wall = time.time()
                        if now_wall - last_pub_time >= pub_period:
                            sample = (now_wall, pos_hold.copy(), quat_hold.copy(), gripper_width)
                            try:
                                if q.full():
                                    _ = q.get_nowait()
                                q.put_nowait(sample)
                            except Exception:
                                pass
                            last_pub_time = now_wall

                        remain = PERIOD - (ctx.time - frame_start)
                        if remain > 0: time.sleep(remain)
                        continue
                    else:
                        pos_hold = None; quat_hold = None

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

                # 限频发送本帧姿态到队列（异步发布进程拿最新）
                now_wall = time.time()
                if now_wall - last_pub_time >= pub_period:
                    sample = (now_wall, pos.copy(), quat.copy(), gripper_width)
                    try:
                        if q.full():
                            _ = q.get_nowait()
                        q.put_nowait(sample)
                    except Exception:
                        pass
                    last_pub_time = now_wall

                remain = PERIOD - (ctx.time - frame_start)
                if remain > 0: time.sleep(remain)
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected, stopping safely ...")
    except Exception as e:
        print(f"[ERROR] Unexpected exception: {e}")
    finally:
        print("[INFO] Stopping publisher process and controller...")
        stop_evt.set()
        try:
            if proc.is_alive():
                proc.join(timeout=1.0)
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
