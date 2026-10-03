"""
局部路径规划 - 环境定义
"""

import numpy as np
from scipy.interpolate import CubicSpline


class Env:  # 2D 场地：范围、起终点、障碍与参考路径
    def __init__(self):  # 初始化场地参数并生成参考路径
        self.x_range = (0.0, 50.0)          # 场地 x 范围
        self.y_range = (0.0, 30.0)          # 场地 y 范围
        self.start = (2.0, 8.0, 0.0)        # 起点位姿 (x, y, yaw)
        self.goal = (47.0, 22.0)            # 终点坐标 (x, y)

        self.waypoints = [                  # 全局参考路径的途经点
            (2.0, 8.0),                     # 第 1 个途经点：起点
            (10.0, 9.0),                    # 第 2 个途经点
            (20.0, 12.0),                   # 第 3 个途经点
            (30.0, 16.0),                   # 第 4 个途经点
            (40.0, 19.0),                   # 第 5 个途经点
            (47.0, 22.0),                   # 第 6 个途经点：终点
        ]
        self.obs_rect = [                   # 矩形障碍 [x, y, w, h]
            [23.0, 12.5, 4.5, 3.0],         # 压住参考路径，必须绕行
            [12.0, 18.0, 6.0, 2.5],         # 路径北侧的障碍
        ]
        self.obs_circle = [                 # 圆形障碍 [x, y, r]
            [36.0, 12.0, 2.5],              # 路径南侧的障碍
            [42.0, 25.5, 2.0],              # 终点附近的障碍
        ]
        self.path = self.reference_path()   # 稠密参考路径 (M, 2)

    def reference_path(self):  # 由途经点插值出平滑参考路径
        """把途经点用三次样条插值成平滑的参考路径
        :return: (M, 2) 的 numpy 数组
        """
        wp = np.array(self.waypoints, dtype=float)                  # 途经点转成数组
        seg = np.hypot(np.diff(wp[:, 0]), np.diff(wp[:, 1]))        # 相邻途经点的间距
        s = np.r_[0.0, np.cumsum(seg)]                              # 累计弧长作为参数
        cs_x = CubicSpline(s, wp[:, 0])                             # x 关于弧长的样条
        cs_y = CubicSpline(s, wp[:, 1])                             # y 关于弧长的样条
        t = np.linspace(0.0, s[-1], int(s[-1] / 0.1) + 1)           # 按 0.1 间隔重采样
        return np.column_stack([cs_x(t), cs_y(t)])                  # 拼成 (M, 2) 返回


if __name__ == '__main__':              # 直接运行本文件时
    env = Env()                                                     # 构造环境
    print("Reference path points =", len(env.path))                 # 打印路径点数
    print("Start =", env.start, " Goal =", env.goal)                # 打印起终点
