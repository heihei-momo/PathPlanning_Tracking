"""
ARA_star 2D (Anytime Repairing A*)
@author: huiming zhou

@description: local inconsistency: g-value decreased.
g(s) decreased introduces a local inconsistency between s and its successors.

"""

import os
import sys
import math

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")  # 将上级目录加入模块搜索路径

from Search_2D import plotting, env


class AraStar:  # ARA* 搜索类
    def __init__(self, s_start, s_goal, e, heuristic_type):  # 初始化起点、终点与初始权重
        self.s_start, self.s_goal = s_start, s_goal  # 记录起点与终点
        self.heuristic_type = heuristic_type  # 启发式类型

        self.Env = env.Env()                                                # class Env

        self.u_set = self.Env.motions                                       # feasible input set
        self.obs = self.Env.obs                                             # position of obstacles
        self.e = e                                                          # weight

        self.g = dict()                                                     # Cost to come
        self.OPEN = dict()                                                  # priority queue / OPEN set
        self.CLOSED = set()                                                 # CLOSED set
        self.INCONS = {}                                                    # INCONSISTENT set
        self.PARENT = dict()                                                # relations
        self.path = []                                                      # planning path
        self.visited = []                                                   # order of visited nodes

    def init(self):  # 初始化各数据结构
        """
        initialize each set.
        """

        self.g[self.s_start] = 0.0  # 起点 g 值为 0
        self.g[self.s_goal] = math.inf  # 终点 g 值先置为无穷
        self.OPEN[self.s_start] = self.f_value(self.s_start)  # 起点按 f 值放入 OPEN
        self.PARENT[self.s_start] = self.s_start  # 起点父节点为自身

    def searching(self):  # ARA* 主流程：权重递减反复搜索
        self.init()  # 初始化各数据结构
        self.ImprovePath()  # 先以较大权重快速求路径
        self.path.append(self.extract_path())  # 保存当前次优路径

        while self.update_e() > 1:                                          # continue condition
            self.e -= 0.4                                                   # increase weight
            self.OPEN.update(self.INCONS)  # 不一致节点重新并入 OPEN
            self.OPEN = {s: self.f_value(s) for s in self.OPEN}             # update f_value of OPEN set

            self.INCONS = dict()  # 清空不一致表
            self.CLOSED = set()  # 清空 CLOSED 表
            self.ImprovePath()                                              # improve path
            self.path.append(self.extract_path())  # 记录本轮更优的路径

        return self.path, self.visited  # 返回路径序列与访问记录

    def ImprovePath(self):  # 在当前权重下改进路径
        """
        :return: a e'-suboptimal path
        """

        visited_each = []  # 本轮新扩展的节点

        while True:  # 反复扩展直到本轮收敛
            s, f_small = self.calc_smallest_f()  # 取 OPEN 中 f 最小节点

            if self.f_value(self.s_goal) <= f_small:  # 目标 f 已不大于当前最小值
                break  # 本轮搜索结束

            self.OPEN.pop(s)  # 从 OPEN 中移除
            self.CLOSED.add(s)  # 标记为已扩展

            for s_n in self.get_neighbor(s):  # 遍历相邻节点
                if s_n in self.obs:  # 跳过障碍节点
                    continue  # 处理下一邻居

                new_cost = self.g[s] + self.cost(s, s_n)  # 经 s 到达邻居的代价

                if s_n not in self.g or new_cost < self.g[s_n]:  # 发现更优的 g 值
                    self.g[s_n] = new_cost  # 更新 g 值
                    self.PARENT[s_n] = s  # 更新父节点
                    visited_each.append(s_n)  # 记录本轮访问节点

                    if s_n not in self.CLOSED:  # 邻居尚未扩展
                        self.OPEN[s_n] = self.f_value(s_n)  # 直接放入 OPEN
                    else:  # 已扩展节点 g 值下降
                        self.INCONS[s_n] = 0.0  # 放入不一致表待重扩

        self.visited.append(visited_each)  # 保存本轮访问顺序

    def calc_smallest_f(self):  # 求 OPEN 中 f 最小的节点
        """
        :return: node with smallest f_value in OPEN set.
        """

        s_small = min(self.OPEN, key=self.OPEN.get)  # 按 f 值取最小键

        return s_small, self.OPEN[s_small]  # 返回节点及其 f 值

    def get_neighbor(self, s):  # 获取 s 的所有邻接节点
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return {(s[0] + u[0], s[1] + u[1]) for u in self.u_set}  # 由运动基元生成邻居集合

    def update_e(self):  # 计算新的权重下界
        v = float("inf")  # 记录 g+h 的最小值

        if self.OPEN:  # OPEN 非空时统计
            v = min(self.g[s] + self.h(s) for s in self.OPEN)  # OPEN 中最小 g+h
        if self.INCONS:  # 不一致表非空时统计
            v = min(v, min(self.g[s] + self.h(s) for s in self.INCONS))  # 并入 INCONS 的最小值

        return min(self.e, self.g[self.s_goal] / v)  # 权重下界，保证次优性

    def f_value(self, x):  # 计算带权 f = g + e*h
        """
        f = g + e * h
        f = cost-to-come + weight * cost-to-go
        :param x: current state
        :return: f_value
        """

        return self.g[x] + self.e * self.h(x)  # g 值加权重乘启发式

    def extract_path(self):  # 由父节点表回溯路径
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]  # 从终点开始回溯
        s = self.s_goal  # 当前回溯节点

        while True:  # 沿父节点向前回溯
            s = self.PARENT[s]  # 取父节点
            path.append(s)  # 加入路径

            if s == self.s_start:  # 到达起点则结束
                break  # 退出回溯

        return list(path)  # 返回起点到终点的路径

    def h(self, s):  # 计算启发式距离
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type                                # heuristic type
        goal = self.s_goal                                                  # goal node

        if heuristic_type == "manhattan":  # 曼哈顿距离启发式
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])  # 坐标差绝对值之和
        else:  # 默认欧氏距离启发式
            return math.hypot(goal[0] - s[0], goal[1] - s[1])  # 两点间欧氏距离

    def cost(self, s_start, s_goal):  # 计算两节点间运动代价
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):  # 该运动是否发生碰撞
            return math.inf  # 碰撞时代价为无穷

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])  # 无碰撞时取欧氏距离

    def is_collision(self, s_start, s_end):  # 检测线段是否穿越障碍
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:  # 端点落在障碍内即碰撞
            return True  # 判定为碰撞

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:  # 斜向运动需检查对角点
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:  # 主对角线方向
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))  # 取左下角点
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))  # 取右上角点
            else:  # 副对角线方向
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))  # 取左上角点
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))  # 取右下角点

            if s1 in self.obs or s2 in self.obs:  # 对角点被占据则碰撞
                return True  # 判定为碰撞

        return False  # 未发生碰撞


def main():  # 演示入口
    s_start = (5, 5)  # 起点坐标
    s_goal = (45, 25)  # 终点坐标

    arastar = AraStar(s_start, s_goal, 2.5, "euclidean")  # 初始权重 2.5 构造 ARA*
    plot = plotting.Plotting(s_start, s_goal)  # 构造绘图对象

    path, visited = arastar.searching()  # 执行 ARA* 搜索
    plot.animation_ara_star(path, visited, "Anytime Repairing A* (ARA*)")  # 动画演示各权重下的路径


if __name__ == '__main__':  # 脚本直接运行时
    main()  # 调用演示入口
