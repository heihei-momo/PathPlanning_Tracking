"""
RRT_2D
@author: huiming zhou
"""

import os
import sys
import math
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env, plotting, utils


class Node:  # 搜索树节点
    def __init__(self, n):  # 由坐标构造节点
        self.x = n[0]  # 节点 x 坐标
        self.y = n[1]  # 节点 y 坐标
        self.parent = None  # 父节点，用于回溯路径


class Rrt:  # RRT 快速扩展随机树
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):  # 初始化规划参数
        self.s_start = Node(s_start)  # 起点节点
        self.s_goal = Node(s_goal)  # 目标节点
        self.step_len = step_len  # 扩展步长
        self.goal_sample_rate = goal_sample_rate  # 目标偏置采样概率
        self.iter_max = iter_max  # 最大迭代次数
        self.vertex = [self.s_start]  # 树节点集合，初始只有起点

        self.env = env.Env()  # 环境实例
        self.plotting = plotting.Plotting(s_start, s_goal)  # 绘图工具
        self.utils = utils.Utils()  # 碰撞检测工具

        self.x_range = self.env.x_range  # 采样 x 范围
        self.y_range = self.env.y_range  # 采样 y 范围
        self.obs_circle = self.env.obs_circle  # 圆形障碍
        self.obs_rectangle = self.env.obs_rectangle  # 矩形障碍
        self.obs_boundary = self.env.obs_boundary  # 边界墙

    def planning(self):  # 主循环：迭代扩展随机树
        for i in range(self.iter_max):  # 每次迭代扩展一个节点
            node_rand = self.generate_random_node(self.goal_sample_rate)  # 采样点（含目标偏置）
            node_near = self.nearest_neighbor(self.vertex, node_rand)  # 树中，距采样点最近节点
            node_new = self.new_state(node_near, node_rand)  # 从最近邻朝采样点扩展

            if node_new and not self.utils.is_collision(node_near, node_new):  # 新节点有效且无碰撞
                self.vertex.append(node_new)  # 新节点加入树
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)  # 新节点到目标的距离

                if dist <= self.step_len and not self.utils.is_collision(node_new, self.s_goal):  # 接近目标
                    self.new_state(node_new, self.s_goal)  # 扩展一步直达目标
                    return self.extract_path(node_new)  # 回溯得到路径

        return None

    def generate_random_node(self, goal_sample_rate):  # 采样：按概率偏向目标
        delta = self.utils.delta  # 安全裕度

        if np.random.random() > goal_sample_rate:  # 以一定概率随机采样
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))  # y 方向采样

        return self.s_goal  # 目标偏置：直接采样目标

    @staticmethod
    def nearest_neighbor(node_list, n):  # 返回距采样点最近节点
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]  # 取距离最小者

    def new_state(self, node_start, node_end):  # 朝目标方向前进一个步长
        dist, theta = self.get_distance_and_angle(node_start, node_end)  # 两点距离与方位角

        dist = min(self.step_len, dist)  # 步长截断
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))  # 沿角度方向扩展
        node_new.parent = node_start  # 记录父节点

        return node_new

    def extract_path(self, node_end):  # 沿 parent 链回溯路径
        path = [(self.s_goal.x, self.s_goal.y)]  # 路径从目标点开始
        node_now = node_end  # 当前回溯节点

        while node_now.parent is not None:  # 一直回溯到起点
            node_now = node_now.parent  # 上溯到父节点
            path.append((node_now.x, node_now.y))  # 记录节点坐标

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):  # 计算距离与方位角
        dx = node_end.x - node_start.x  # x 方向差
        dy = node_end.y - node_start.y  # y 方向差
        return math.hypot(dx, dy), math.atan2(dy, dx)  # 距离, 角度


def main():
    x_start = (2, 2)  # Starting node
    x_goal = (49, 24)  # Goal node

    rrt = Rrt(x_start, x_goal, 0.5, 0.05, 10000)  # 步长0.5，目标偏置0.05
    path = rrt.planning()  # 执行规划

    if path:  # 找到路径
        rrt.plotting.animation(rrt.vertex, path, "RRT", True)  # 动画展示
    else:
        print("No Path Found!")  # 未找到路径


if __name__ == '__main__':  # 脚本入口
    main()  # 运行示例
