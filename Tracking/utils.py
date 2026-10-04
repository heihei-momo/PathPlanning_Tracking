"""
路径跟踪 - 通用工具
包含：运动学自行车模型、路径投影、跟踪误差计算
"""

import math
import numpy as np


class Utils:  # 跟踪相关的几何与运动学工具
    def __init__(self, env):  # 绑定环境
        self.env = env                          # 环境对象
        self.path = env.path                    # 参考轨迹 (M, 4)
        self.L = env.L                          # 轴距
        self.path_s = self.path_lengths()       # 参考轨迹累计弧长

    @staticmethod
    def normalize_angle(theta):  # 把角度归一化到 (-pi, pi]
        """把角度归一化到 (-pi, pi]"""
        while theta > math.pi:                  # 超过 pi 就减一圈
            theta -= 2.0 * math.pi              # 减一个整圈
        while theta <= -math.pi:                # 小于等于 -pi 就加一圈
            theta += 2.0 * math.pi              # 加一个整圈
        return theta                            # 返回归一化角度

    @staticmethod
    def normalize_array(theta):  # 数组版的角度归一化
        """把角度数组归一化到 (-pi, pi]"""
        return (theta + np.pi) % (2.0 * np.pi) - np.pi   # 用取模一次算完

    @staticmethod
    def dist(p, q):  # 两点欧氏距离
        """两点欧氏距离"""
        return math.hypot(p[0] - q[0], p[1] - q[1])     # 勾股定理

    def path_lengths(self):  # 参考轨迹的累计弧长
        """参考轨迹的累计弧长"""
        seg = np.hypot(np.diff(self.path[:, 0]), np.diff(self.path[:, 1]))   # 相邻点间距
        return np.r_[0.0, np.cumsum(seg)]                                    # 累计弧长

    def nearest_on_path(self, x, y):  # 找参考轨迹上最近的点
        """找参考轨迹上离 (x, y) 最近的点
        :return: (最近点下标, 最近距离)
        """
        d = np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)          # 到各点的距离
        i = int(np.argmin(d))                                           # 最近点下标
        return i, float(d[i])                                           # 返回下标与距离

    def lateral_error(self, state):  # 计算跟踪误差
        """计算相对参考轨迹的带符号横向误差与航向误差
        :return: (e_y 横向误差[左正], e_theta 航向误差, 最近点下标)
        """
        x, y, theta = state                                             # 拆出位姿
        i, _ = self.nearest_on_path(x, y)                               # 最近点下标
        xr, yr, thr = self.path[i, 0], self.path[i, 1], self.path[i, 2]  # 参考点与切向角
        dx, dy = x - xr, y - yr                                         # 位置偏差
        e_y = -math.sin(thr) * dx + math.cos(thr) * dy                  # 投影到法向（左正）
        e_theta = Utils.normalize_angle(theta - thr)                    # 航向误差
        return e_y, e_theta, i                                          # 返回误差与下标

    def lookahead_point(self, x, y, ld):  # 沿轨迹向前取前视点
        """沿参考轨迹向前取 ld 距离处的点，作为 Pure Pursuit 的前视点
        :return: (px, py) 前视点坐标
        """
        i, _ = self.nearest_on_path(x, y)                               # 先找最近点
        j = int(np.searchsorted(self.path_s, self.path_s[i] + ld))      # 按弧长定位
        j = min(max(j, 1), len(self.path) - 1)                          # 防止下标越界
        return self.path[j, 0], self.path[j, 1]                         # 返回前视点

    def path_remaining(self, x, y):  # 到轨迹终点的剩余弧长
        """从当前位置沿参考轨迹到终点的剩余弧长"""
        i, _ = self.nearest_on_path(x, y)                               # 最近点下标
        return float(self.path_s[-1] - self.path_s[i])                  # 剩余弧长

    @staticmethod
    def bicycle_step(state, v, delta, L, dt):  # 运动学自行车模型积分一步
        """运动学自行车模型积分一步，state = (x, y, theta)，控制为 (v, delta)"""
        x, y, theta = state                                             # 拆出位姿
        x = x + v * math.cos(theta) * dt                                # 更新 x
        y = y + v * math.sin(theta) * dt                                # 更新 y
        theta = Utils.normalize_angle(theta + v * math.tan(delta) / L * dt)   # 更新航向角
        return np.array([x, y, theta], dtype=float)                     # 返回新位姿

    def simulate(self, state, v, delta, steps):  # 用固定控制推演多步
        """用固定控制 (v, delta) 推演 steps 步
        :return: (steps + 1, 3) 的轨迹数组
        """
        dt = self.env.dt                                                # 仿真步长
        traj = [np.array(state, dtype=float)]                           # 轨迹首元素
        for _ in range(steps):                                          # 逐步积分
            traj.append(Utils.bicycle_step(traj[-1], v, delta, self.L, dt))   # 追加新位姿
        return np.array(traj)                                           # 转成数组返回
