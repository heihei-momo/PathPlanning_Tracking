"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")  # 将上级目录加入模块搜索路径

from Search_2D import plotting, env


class AStar:  # A* 搜索类：按 f=g+h 扩展
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, s_start, s_goal, heuristic_type):  # 初始化起点、终点与启发式类型
        self.s_start = s_start  # 起点坐标
        self.s_goal = s_goal  # 终点坐标
        self.heuristic_type = heuristic_type  # 启发式类型：manhattan 或 euclidean

        self.Env = env.Env()  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come

    def searching(self):  # 标准 A* 主搜索流程
        """
        A_star Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start  # 起点父节点指向自身
        self.g[self.s_start] = 0  # 起点 g 值为 0
        self.g[self.s_goal] = math.inf  # 终点 g 值先置为无穷
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))  # 起点按 f 值入优先队列

        while self.OPEN:  # OPEN 非空则继续扩展
            _, s = heapq.heappop(self.OPEN)  # 弹出 f 值最小的节点
            self.CLOSED.append(s)  # 记入已扩展节点顺序

            if s == self.s_goal:  # stop condition
                break  # 到达终点，退出搜索

            for s_n in self.get_neighbor(s):  # 遍历 s 的可行邻居
                new_cost = self.g[s] + self.cost(s, s_n)  # 经 s 到邻居的候选 g 值

                if s_n not in self.g:  # 首次遇到该邻居
                    self.g[s_n] = math.inf  # g 值先置为无穷

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost  # 更新更小的 g 值
                    self.PARENT[s_n] = s  # 记录父节点用于回溯
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))  # 邻居按 f 值入堆

        return self.extract_path(self.PARENT), self.CLOSED  # 返回路径与访问顺序

    def searching_repeated_astar(self, e):  # 加权 A* 重复搜索（供 ARA* 用）
        """
        repeated A*.
        :param e: weight of A*
        :return: path and visited order
        """

        path, visited = [], []  # 保存各权重的路径与访问序

        while e >= 1:  # 权重从 e 递减到 1
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)  # 以当前权重搜索一次
            path.append(p_k)  # 收集本次路径
            visited.append(v_k)  # 收集本次访问顺序
            e -= 0.5  # 权重递减，逐步逼近最优

        return path, visited  # 返回所有权重下的结果

    def repeated_searching(self, s_start, s_goal, e):  # 单次带权重 e 的 A* 搜索
        """
        run A* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}  # 局部 g 值表
        PARENT = {s_start: s_start}  # 局部父节点表
        OPEN = []  # 局部 OPEN 优先队列
        CLOSED = []  # 局部 CLOSED 表
        heapq.heappush(OPEN,
                       (g[s_start] + e * self.heuristic(s_start), s_start))  # 起点按加权 f 值入堆

        while OPEN:  # OPEN 非空则继续扩展
            _, s = heapq.heappop(OPEN)  # 弹出 f 值最小节点
            CLOSED.append(s)  # 记入已扩展集合

            if s == s_goal:  # 到达终点则结束
                break  # 到达终点，退出搜索

            for s_n in self.get_neighbor(s):  # 遍历 s 的可行邻居
                new_cost = g[s] + self.cost(s, s_n)  # 经 s 到邻居的候选代价

                if s_n not in g:  # 邻居尚未访问过
                    g[s_n] = math.inf  # g 值先置为无穷

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost  # 更新更小的 g 值
                    PARENT[s_n] = s  # 记录父节点
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))  # 邻居按加权 f 值入堆

        return self.extract_path(PARENT), CLOSED  # 返回路径与访问顺序

    def get_neighbor(self, s):  # 获取 s 的所有邻接节点
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]  # 由运动基元生成邻居坐标

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

    def f_value(self, s):  # 计算节点 f = g + h
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g[s] + self.heuristic(s)  # g 值加启发式值

    def extract_path(self, PARENT):  # 由父节点表回溯路径
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]  # 从终点开始回溯
        s = self.s_goal  # 当前回溯节点

        while True:  # 沿父节点向前回溯
            s = PARENT[s]  # 取父节点
            path.append(s)  # 加入路径

            if s == self.s_start:  # 到达起点则结束
                break  # 退出回溯循环

        return list(path)  # 返回起点到终点的路径

    def heuristic(self, s):  # 计算启发式距离
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":  # 曼哈顿距离启发式
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])  # 坐标差绝对值之和
        else:  # 默认欧氏距离启发式
            return math.hypot(goal[0] - s[0], goal[1] - s[1])  # 两点间欧氏距离


def main():  # 演示入口
    s_start = (5, 5)  # 起点坐标
    s_goal = (45, 25)  # 终点坐标

    astar = AStar(s_start, s_goal, "euclidean")  # 构造欧氏启发式的 A*
    plot = plotting.Plotting(s_start, s_goal)  # 构造绘图对象

    path, visited = astar.searching()  # 执行 A* 搜索
    plot.animation(path, visited, "A*")  # animation

    # path, visited = astar.searching_repeated_astar(2.5)               # initial weight e = 2.5
    # plot.animation_ara_star(path, visited, "Repeated A*")


if __name__ == '__main__':  # 脚本直接运行时
    main()  # 调用演示入口
