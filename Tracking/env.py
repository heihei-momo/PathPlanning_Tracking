"""
路径跟踪 - 环境与参考轨迹
"""

import numpy as np


class Env:
    def __init__(self):
        self.x_range = (0.0, 60.0)          # 场地 x 范围
        self.y_range = (-12.0, 12.0)        # 场地 y 范围
        self.L = 2.5                        # 车辆轴距 [m]
        self.v_ref = 5.0                    # 目标车速 [m/s]
        self.dt = 0.05                      # 仿真步长 [s]
        self.delta_max = np.deg2rad(30.0)   # 前轮最大转角 [rad]
        self.amp = 4.0                      # 参考轨迹正弦振幅 [m]
        self.wave = 45.0                    # 参考轨迹正弦波长 [m]
        self.x_end = 60.0                   # 参考轨迹终点 x 坐标
        self.t_end = 20.0                   # 误差曲线的横轴上限 [s]
        self.path = self.reference_path()   # 参考轨迹 (M, 4): x, y, theta, kappa
        self.start = self.path[0, :3].copy()    # 起点位姿取自轨迹起点

    def ref_y(self, x):  # 参考轨迹的 y 坐标
        """正弦参考轨迹的 y 坐标，x 可以是数组"""
        return self.amp * np.sin(2.0 * np.pi * x / self.wave)     # 正弦函数

    def ref_theta(self, x):  # 参考轨迹的切线方向角
        """正弦参考轨迹的切向角，x 可以是数组"""
        k = 2.0 * np.pi / self.wave                               # 角频率
        dydx = self.amp * k * np.cos(k * x)                       # 一阶导
        return np.arctan(dydx)                                    # 切线角

    def ref_kappa(self, x):  # 参考轨迹的曲率
        """正弦参考轨迹的曲率，x 可以是数组"""
        k = 2.0 * np.pi / self.wave                               # 角频率
        dydx = self.amp * k * np.cos(k * x)                       # 一阶导
        d2ydx2 = -self.amp * k * k * np.sin(k * x)                # 二阶导
        return d2ydx2 / (1.0 + dydx ** 2) ** 1.5                  # 曲率公式

    def reference_path(self):  # 采样出参考轨迹
        """按 0.1 m 间隔采样，返回 (M, 4) 的 [x, y, theta, kappa]"""
        x = np.arange(0.0, self.x_end + 1e-9, 0.1)                # 沿 x 均匀采样
        return np.column_stack([x, self.ref_y(x),                  # x、y 坐标
                                self.ref_theta(x), self.ref_kappa(x)])   # 切向角与曲率


if __name__ == '__main__':              # 直接运行本文件时
    env = Env()                                                     # 构造环境
    print("Reference path points =", len(env.path))                 # 打印轨迹点数
    print("Start =", np.round(env.start, 3))                        # 打印起点位姿
    print("Max curvature =", round(float(np.abs(env.path[:, 3]).max()), 4), "1/m")   # 最大曲率
