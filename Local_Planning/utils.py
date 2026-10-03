"""
局部路径规划 - 通用工具
包含：碰撞检测、路径投影、运动学积分
"""

import math
import numpy as np


class Utils:  # 几何与运动学工具集
    def __init__(self, env):  # 绑定环境并设置机器人参数
        self.env = env                          # 环境对象
        self.path = env.path                    # 参考路径点集
        self.robot_radius = 0.6                 # 机器人外接圆半径
        self.reso = 0.2                         # 碰撞检测采样间隔

    @staticmethod
    def normalize_angle(theta):  # 把角度归一化到 (-pi, pi]
        """把角度归一化到 (-pi, pi]"""
        while theta > math.pi:                  # 超过 pi 就减一圈
            theta -= 2.0 * math.pi              # 减一个整圈
        while theta <= -math.pi:                # 小于等于 -pi 就加一圈
            theta += 2.0 * math.pi              # 加一个整圈
        return theta                            # 返回归一化后的角度

    @staticmethod
    def dist(p, q):  # 计算两点欧氏距离
        """两点欧氏距离"""
        return math.hypot(p[0] - q[0], p[1] - q[1])     # 勾股定理求距离

    @staticmethod
    def angle_diff(a, b):  # 计算两个角度之差
        """两个角度之差，结果归一化到 (-pi, pi]"""
        return Utils.normalize_angle(a - b)     # 差值再做归一化

    def obstacle_distance(self, x, y):  # 点到障碍表面的带符号距离
        """点到所有障碍表面的带符号距离，越大越安全，负值表示已进入障碍内部"""
        d = min(x - self.env.x_range[0], self.env.x_range[1] - x,       # 到左右边界
                y - self.env.y_range[0], self.env.y_range[1] - y)       # 到上下边界
        for (ox, oy, w, h) in self.env.obs_rect:                        # 逐个矩形障碍
            dx = max(ox - x, 0.0, x - (ox + w))                         # 外部时 x 方向超出量
            dy = max(oy - y, 0.0, y - (oy + h))                         # 外部时 y 方向超出量
            if dx == 0.0 and dy == 0.0:                                 # 点落在矩形内部
                d = min(d, -min(x - ox, ox + w - x,                     # 取到最近边的负距离
                                y - oy, oy + h - y))                    # 内部记负值
            else:                                                       # 点在矩形外部
                d = min(d, math.hypot(dx, dy))                          # 取到顶点的正距离
        for (ox, oy, r) in self.env.obs_circle:                         # 逐个圆形障碍
            d = min(d, math.hypot(x - ox, y - oy) - r)                  # 减去圆半径
        return d                                                        # 返回最小带符号距离

    def is_point_collision(self, x, y, margin=0.0):  # 判断点是否撞上障碍
        """点是否撞上障碍，margin 为安全裕度"""
        return self.obstacle_distance(x, y) < margin                    # 距离小于裕度即碰撞

    def is_segment_collision(self, p, q, margin=0.0):  # 判断线段是否撞上障碍
        """线段是否撞上障碍，沿线等间隔采样判断"""
        n = max(2, int(self.dist(p, q) / self.reso) + 1)                # 需要采样的点数
        for t in np.linspace(0.0, 1.0, n):                              # 沿线均匀采样
            x = p[0] + t * (q[0] - p[0])                                # 采样点 x 坐标
            y = p[1] + t * (q[1] - p[1])                                # 采样点 y 坐标
            if self.is_point_collision(x, y, margin):                   # 撞上就立刻返回
                return True                                             # 线段有碰撞
        return False                                                    # 整段都安全

    def is_trajectory_collision(self, traj, margin=0.0):  # 判断整条轨迹是否撞上障碍
        """轨迹是否撞上障碍，traj 为 (N, 2) 或 (N, 3) 数组"""
        traj = np.asarray(traj, dtype=float)                            # 统一转成数组
        for i in range(len(traj) - 1):                                  # 逐段检查
            if self.is_segment_collision(traj[i], traj[i + 1], margin):  # 有一段撞上就算
                return True                                             # 轨迹有碰撞
        last = traj[-1]                                                 # 末点单独判断
        return self.is_point_collision(last[0], last[1], margin)        # 返回末点判断结果

    def nearest_on_path(self, x, y):  # 找参考路径上离给定点最近的点
        """找参考路径上离 (x, y) 最近的点
        :return: (最近点下标, 最近距离)
        """
        d = np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)          # 到各路径点的距离
        i = int(np.argmin(d))                                           # 最近点下标
        return i, float(d[i])                                           # 返回下标与距离

    def path_lengths(self):  # 计算参考路径的累计弧长
        """参考路径的累计弧长，用于按弧长取点"""
        seg = np.hypot(np.diff(self.path[:, 0]), np.diff(self.path[:, 1]))   # 相邻点间距
        return np.r_[0.0, np.cumsum(seg)]                               # 返回累计弧长

    def path_lookahead(self, x, y, dist_ahead):  # 沿路径向前取前瞻点
        """沿参考路径向前取 dist_ahead 处的点，用作局部目标"""
        s = self.path_lengths()                                         # 参考路径累计弧长
        i, _ = self.nearest_on_path(x, y)                               # 先找最近点下标
        j = int(np.searchsorted(s, s[i] + dist_ahead))                  # 按弧长定位目标点
        j = min(max(j, 1), len(self.path) - 1)                          # 防止下标越界
        return self.path[j]                                             # 返回前瞻点坐标

    def path_remaining(self, x, y):  # 计算到终点的剩余弧长
        """从当前位置沿参考路径到终点的剩余弧长"""
        s = self.path_lengths()                                         # 参考路径累计弧长
        i, _ = self.nearest_on_path(x, y)                               # 最近点下标
        return float(s[-1] - s[i])                                      # 返回剩余弧长

    @staticmethod
    def motion(state, v, w, dt):  # 单车模型单步积分
        """按单车模型积分一步，state = (x, y, yaw)，控制为 (v, w)"""
        x, y, yaw = state                                               # 拆出状态分量
        x = x + v * math.cos(yaw) * dt                                  # 更新 x 坐标
        y = y + v * math.sin(yaw) * dt                                  # 更新 y 坐标
        yaw = Utils.normalize_angle(yaw + w * dt)                       # 更新航向角
        return np.array([x, y, yaw], dtype=float)                       # 返回新状态

    def simulate(self, state, v, w, dt, steps):  # 用固定控制推演多步
        """从 state 出发用固定控制 (v, w) 推演 steps 步
        :return: (steps + 1, 3) 的轨迹数组
        """
        traj = [np.array(state, dtype=float)]                           # 轨迹的首个状态
        for _ in range(steps):                                          # 逐步向前积分
            traj.append(self.motion(traj[-1], v, w, dt))                # 追加新状态
        return np.array(traj)                                           # 转成数组返回
