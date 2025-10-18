import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from panda_py import Panda
from panda_py.controllers import CartesianImpedance
from panda_py import constants as C
from panda_py.libfranka import Gripper
from space_mouse import Spacemouse

HOST = "172.16.0.2"
CTRL_FREQ = 200.0    # Python侧发送频率（Hz)
LIN_VEL_MAX = 0.1
ANG_VEL_MAX = 0.8
DEADMAN_BTN = 0
ESTOP_BTN = 1

NEUTRAL_ENTER = 0.08
NEUTRAL_EXIT  = 0.12

XYZ_MIN = np.array([-0.80, -0.80, 0.05])
XYZ_MAX = np.array([0.80,  0.80, 0.60])

ANG_VEL_DEADZONE = 0.05
PERIOD = 1.0 / CTRL_FREQ

# ================== 安全护栏参数 ==================
MAX_STEP_LIN = 0.002      # 每 tick 最大线位移（m）
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

def main():
    robot = Panda(HOST)
    ctrl = CartesianImpedance()
    robot.start_controller(ctrl)

    # pos0, quat0 = freeze_to_measured_pose(robot)
    # q_null = C.JOINT_POSITION_START
    # ctrl.set_control(pos0, quat0, q_nullspace=q_null)

    v_state = np.zeros(3)
    w_state = np.zeros(3)
    neutral_mode = True
    pos_hold = None; quat_hold = None

    # 初始化 Gripper 与参数
    gripper = Gripper(HOST)
    gripper.move(0.08, 0.05)
    # try:
    #     gs = gripper.read_once()
    #     grip_width = float(gs.width)
    #     grip_max   = float(gs.max_width)
    # except Exception:
    #     grip_width = 0.04
    #     grip_max   = 0.08
    # GRIP_SPEED = 2.0      # m/s，手爪执行速度
    # GRIP_STEP  = 0.001      # m，每次按键触发的一小步（2mm，可改小到1mm）

    # 微输入计数器
    micro_cnt = 0

    with robot.create_context(CTRL_FREQ) as ctx, Spacemouse(max_value=500, deadzone=0.2, scale_factor=1.0) as sm:
        pos = robot.get_position().astype(float).reshape(3)
        quat = robot.get_orientation(scalar_first=False).astype(float).reshape(4)
        q_null = C.JOINT_POSITION_START

        sm.lock_rotation_axes(roll=False, pitch=False, yaw=True)

        # print("Start teleop. Hold button {} to move, button {} for e-stop.".format(DEADMAN_BTN, ESTOP_BTN))
        last_time = ctx.time
        while ctx.ok():
            frame_start = ctx.time        # <<< 本帧起点（用控制器时钟）
            now = frame_start
            dt = max(1e-3, now - last_time)
            last_time = now

            # <<< 新增：超时看门狗，防止一次性大步
            if dt > OVERRUN_FACTOR * PERIOD:
                v_state[:] = 0.0
                w_state[:] = 0.0
                pos, quat = freeze_to_measured_pose(robot)

            u = sm.get_motion_state_transformed()
            u_level = np.max(np.abs(u))

            # if sm.is_button_pressed(ESTOP_BTN):
            #     print("E-STOP pressed! Stopping...")
            #     break

            # 夹爪按键优先，单步执行
            close_pressed = sm.is_button_pressed(0)
            open_pressed  = sm.is_button_pressed(1)
            if close_pressed:
                print("Closing and grasping...")
                grasp_state = gripper.grasp(width=0.00, speed=0.05, force=1.0, epsilon_inner=0.03, epsilon_outer=0.03)
                print(f"grasp state:{grasp_state}")
            elif open_pressed:
                print("Opening gripper...")
                # gripper.move(0.04, 0.05)
                gripper.move(0.08, 0.05)

                # remain = PERIOD - (ctx.time - frame_start)   # 周期补偿
                # if remain > 0: time.sleep(remain)
                # print(f"Start3:{time.time()}")
                # continue
            # print(f"Start1:{time.time()}")

            # 摇杆回中刹车
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
                    ctrl.set_control(pos_hold, quat_hold, q_nullspace=q_null)
                    # print("1")
                    # 周期补偿
                    remain = PERIOD - (ctx.time - frame_start)   # <<< 替换原 sleep
                    if remain > 0: time.sleep(remain)
                    continue
            else:
                if u_level < NEUTRAL_ENTER:
                    neutral_mode = True
                    pos_hold = robot.get_position().astype(float).reshape(3)
                    quat_hold = robot.get_orientation(scalar_first=False).astype(float).reshape(4)
                    v_state[:] = 0.0; w_state[:] = 0.0
                    ctrl.set_control(pos_hold, quat_hold, q_nullspace=q_null)
                    # 周期补偿
                    remain = PERIOD - (ctx.time - frame_start)   # <<< 替换原 sleep
                    if remain > 0: time.sleep(remain)
                    continue
                else:
                    pos_hold = None; quat_hold = None

            # print("2")

            # 微输入去累积（防止在轻微输入下慢慢攒起来）
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

            # 啓動和停下的时间常数
            tau_on_lin, tau_off_lin = 0.05, 0.03
            tau_on_rot, tau_off_rot = 0.05, 0.02
            alpha_v = np.clip(dt / (tau_on_lin if np.any(v_cmd) else tau_off_lin), 0.0, 1.0)
            alpha_w = np.clip(dt / (tau_on_rot if np.any(w_cmd) else tau_off_rot), 0.0, 1.0)

            v_state = (1 - alpha_v) * v_state + alpha_v * v_cmd
            w_state = (1 - alpha_w) * w_state + alpha_w * w_cmd

            # ================== 单步限幅 + 边界防积分 ==================
            step = v_state * dt

            # 到边界且仍往外推的分量清零
            at_min = pos <= XYZ_MIN + 1e-6
            at_max = pos >= XYZ_MAX - 1e-6
            push_out = ((step < 0) & at_min) | ((step > 0) & at_max)
            step[push_out] = 0.0

            # 每轴单步线位移限幅
            step = np.clip(step, -MAX_STEP_LIN, MAX_STEP_LIN)
            pos = clamp(pos + step, XYZ_MIN, XYZ_MAX)

            # 单步角位移限幅
            ang = np.linalg.norm(w_state) * dt
            if ang > 1e-12:
                ang = min(ang, MAX_STEP_ANG)
                axis = w_state / (np.linalg.norm(w_state) + 1e-12)
                dq = axisangle_to_quat_scalar_last(axis, ang)
                quat = quat_multiply_scalar_last(quat, dq)
                quat = quat / np.linalg.norm(quat)
            # =====================================================================

            ctrl.set_control(pos, quat, q_nullspace=q_null)

            # 统一用周期补偿，保证稳定 200 Hz
            remain = PERIOD - (ctx.time - frame_start)
            if remain > 0: time.sleep(remain)

    robot.stop_controller()
    print("Teleop finished.")

if __name__ == "__main__":
    main()
