# import time
# from spnav import spnav_open, spnav_close, spnav_poll_event, SpnavMotionEvent, SpnavButtonEvent

# spnav_open()
# try:
#     while True:
#         ev = spnav_poll_event()
#         if isinstance(ev, SpnavMotionEvent):
#             print("trans:", ev.translation, "rot:", ev.rotation)
#         elif isinstance(ev, SpnavButtonEvent):
#             print("button:", ev.bnum, "pressed" if ev.press else "released")
#         else:
#             time.sleep(0.005)
# finally:
#     spnav_close()



import time
from threading import Thread, Event
from collections import defaultdict
import numpy as np

from spnav import (
    spnav_open, spnav_close, spnav_poll_event,
    SpnavMotionEvent, SpnavButtonEvent
)

class Spacemouse(Thread):
    def __init__(
        self,
        max_value=500,                 # 500: 无线常用，300: 有线常用
        deadzone=(0,0,0,0,0,0),        # 可设标量或 6 元组
        dtype=np.float32,
        scale_factor=1.0,              # 你原代码里的 SCALE_FACTOR
        poll_sleep=1/200,              # 无事件时休眠，降低 CPU 占用
    ):
        """
        持续监听 3Dconnexion SpaceMouse 事件，并维护最新状态。
        max_value: {300(有线), 500(无线)} 用于归一化
        deadzone: [0,1]，死区，小于该值的轴清零
        """
        super().__init__()
        self.daemon = True

        if np.issubdtype(type(deadzone), np.number):
            deadzone = np.full(6, fill_value=deadzone, dtype=dtype)
        else:
            deadzone = np.array(deadzone, dtype=dtype)
        assert (deadzone >= 0).all()

        self.stop_event = Event()
        self.max_value = float(max_value)
        self.dtype = dtype
        self.deadzone = deadzone.astype(dtype)
        self.scale_factor = float(scale_factor)
        self.poll_sleep = float(poll_sleep)
        self._lock_roll = False
        self._lock_pitch = False
        self._lock_yaw = False

        # 默认“空事件”（不要直接构造 SpnavMotionEvent，以防版本不兼容）
        self.motion_event = type("Motion", (), {
            "translation": [0,0,0],
            "rotation":    [0,0,0],
            "period":      0
        })()

        self.button_state = defaultdict(lambda: False)

        # 你原来的坐标变换矩阵（Z-up -> 右手系）
        self.tx_zup_spnav = np.array([
            [0, 0, -1],
            [1, 0, 0],
            [0, 1, 0]
        ], dtype=dtype)

    # -------- 便捷获取原始量 --------
    def get_raw_translation(self):
        return np.array(self.motion_event.translation, dtype=self.dtype)

    def get_raw_rotation(self):
        return np.array(self.motion_event.rotation, dtype=self.dtype)

    # -------- 归一化 + 死区处理 --------
    def get_motion_state(self):
        """
        返回 6 维向量: [tx, ty, tz, rx, ry, rz]，已按 max_value 归一化并应用死区
        """
        me = self.motion_event
        state = np.array(me.translation + me.rotation, dtype=self.dtype) / self.max_value
        is_dead = (-self.deadzone < state) & (state < self.deadzone)
        state[is_dead] = 0
        return state

    # -------- 坐标变换 + 缩放 --------
    def get_motion_state_transformed(self):
        """
        返回右手系下的 6 维量，并做小阈值抑制与 scale_factor 缩放
        """
        state = self.get_motion_state()
        tf_state = np.zeros_like(state)
        tf_state[:3] = self.tx_zup_spnav @ state[:3]
        tf_state[3:] = self.tx_zup_spnav @ state[3:]

        # 小阈值抑制：可视需要调小/关闭
        mask = np.abs(tf_state) < 0.3
        tf_state[mask] = 0

        tf_state = tf_state * self.scale_factor
        tf_state[4] = -tf_state[4]
        tf_state[5] = -tf_state[5]

        if self._lock_roll is False:
            tf_state[3] = tf_state[3] * 0.5
        if self._lock_pitch is False:
            tf_state[4] = tf_state[4] * 0.5
        if self._lock_yaw is False:
            tf_state[5] = tf_state[5] * 0.5

        return tf_state
    
    def lock_rotation_axes(self, roll=False, pitch=False, yaw=False):
        self._lock_roll = roll
        self._lock_pitch = pitch
        self._lock_yaw = yaw

    def is_button_pressed(self, button_id):
        return bool(self.button_state[button_id])

    def stop(self):
        self.stop_event.set()
        # 等待线程结束
        if self.is_alive():
            self.join(timeout=1.0)

    # 上下文管理
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # 线程主循环
    def run(self):
        spnav_open()
        try:
            while not self.stop_event.is_set():
                event = spnav_poll_event()
                if event is None:
                    time.sleep(self.poll_sleep)
                    continue

                if isinstance(event, SpnavMotionEvent):
                    # 直接保存最新事件
                    self.motion_event = event
                elif isinstance(event, SpnavButtonEvent):
                    self.button_state[event.bnum] = bool(event.press)
                else:
                    # 未知事件/空事件，稍作休眠防止空转
                    time.sleep(self.poll_sleep)
        finally:
            spnav_close()


if __name__ == "__main__":
    with Spacemouse(max_value=500, deadzone=0.2, scale_factor=1.0) as sm:
        sm.lock_rotation_axes(roll=False, pitch=False, yaw=True)
        while True:
            tf6 = sm.get_motion_state_transformed()
            if np.any(tf6):
                print("tf6:", np.round(tf6, 3))
            if sm.is_button_pressed(0):
                print("Btn0 pressed")
            if sm.is_button_pressed(1):
                print("Btn1 pressed")
            time.sleep(0.01)

