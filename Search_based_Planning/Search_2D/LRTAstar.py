"""
LRTA_star 2D (Learning Real-time A*)
@author: huiming zhou
"""

import os
import sys
import copy
import math

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import queue, plotting, env


class LrtAStarN:
    def __init__(self, s_start, s_goal, N, heuristic_type):
        self.s_start, self.s_goal = s_start, s_goal  # 起点与终点
        self.heuristic_type = heuristic_type  # 启发式类型

        self.Env = env.Env()  # 环境实例

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.N = N  # number of expand nodes each iteration
        self.visited = []  # order of visited nodes in planning
        self.path = []  # path of each iteration
        self.h_table = {}  # h_value table

    def init(self):
        """
        initialize the h_value of all nodes in the environment.
        it is a global table.
        """

        for i in range(self.Env.x_range):  # 遍历 x 方向全部栅格
            for j in range(self.Env.y_range):  # 遍历 y 方向全部栅格
                self.h_table[(i, j)] = self.h((i, j))  # 用初始启发式填满 h 表

    def searching(self):
        self.init()  # 建立全局 h 表
        s_start = self.s_start  # initialize start node

        while True:  # 每轮扩展 N 步，直至到达终点
            OPEN, CLOSED = self.AStar(s_start, self.N)  # OPEN, CLOSED sets in each iteration

            if OPEN == "FOUND":  # reach the goal node
                self.path.append(CLOSED)  # 记录本轮路径
                break  # 已到终点，结束

            h_value = self.iteration(CLOSED)  # h_value table of CLOSED nodes

            for x in h_value:  # 把学到的 h 值写回全局表
                self.h_table[x] = h_value[x]  # 更新该节点启发式

            s_start, path_k = self.extract_path_in_CLOSE(s_start, h_value)  # x_init -> expected node in OPEN set
            self.path.append(path_k)  # 记录本轮实际执行路径

    def extract_path_in_CLOSE(self, s_start, h_value):
        path = [s_start]  # 从当前起点出发
        s = s_start  # 当前节点

        while True:  # 向 OPEN 中的期望节点逐步移动
            h_list = {}  # 邻居的 h 值表

            for s_n in self.get_neighbor(s):  # 遍历可行邻居
                if s_n in h_value:  # 邻居在本轮 CLOSED 中
                    h_list[s_n] = h_value[s_n]  # 采用更新后的 h 值
                else:  # 邻居未在本轮扩展
                    h_list[s_n] = self.h_table[s_n]  # 采用全局 h 表的值

            s_key = min(h_list, key=h_list.get)  # move to the smallest node with min h_value
            path.append(s_key)  # generate path
            s = s_key  # use end of this iteration as the start of next

            if s_key not in h_value:  # reach the expected node in OPEN set
                return s_key, path  # 返回新起点与本段路径

    def iteration(self, CLOSED):
        h_value = {}  # 本轮 CLOSED 节点的 h 值

        for s in CLOSED:  # 遍历本轮全部关闭节点
            h_value[s] = float("inf")  # initialize h_value of CLOSED nodes，先把 CLOSED 中所有节点的 h 设成无穷

        while True:  # 迭代直到 h 值收敛
            h_value_rec = copy.deepcopy(h_value)  # 备份上一轮 h 值
            for s in CLOSED:  # 逐个节点更新 h 值
                h_list = []  # 候选后继代价列表
                for s_n in self.get_neighbor(s):  # 遍历邻居
                    if s_n not in CLOSED:  # 邻居未在本轮扩展
                        h_list.append(self.cost(s, s_n) + self.h_table[s_n])  # 用全局 h 估计后继代价
                    else:  # 邻居也在本轮 CLOSED 中
                        h_list.append(self.cost(s, s_n) + h_value[s_n])  # 用本轮新 h 估计后继代价
                h_value[s] = min(h_list)  # update h_value of current node

            if h_value == h_value_rec:  # h_value table converged
                return h_value  # 返回收敛后的 h 值

    def AStar(self, x_start, N):
        OPEN = queue.QueuePrior()  # OPEN set
        OPEN.put(x_start, self.h(x_start))  # 起点按初始 h 入队
        CLOSED = []  # CLOSED set
        g_table = {x_start: 0, self.s_goal: float("inf")}  # Cost to come
        PARENT = {x_start: x_start}  # relations
        count = 0  # counter

        while not OPEN.empty():  # OPEN 非空则继续扩展
            count += 1  # 已扩展节点计数
            s = OPEN.get()  # 取出 f 最小的节点
            CLOSED.append(s)  # 加入关闭表

            if s == self.s_goal:  # reach the goal node
                self.visited.append(CLOSED)  # 记录本轮访问顺序
                return "FOUND", self.extract_path(x_start, PARENT)  # 找到终点并回溯路径

            for s_n in self.get_neighbor(s):  # 遍历邻居
                if s_n not in CLOSED:  # 跳过已关闭节点
                    new_cost = g_table[s] + self.cost(s, s_n)  # 经 s 到邻居的新代价
                    if s_n not in g_table:  # 邻居首次访问
                        g_table[s_n] = float("inf")  # g 值先置无穷
                    if new_cost < g_table[s_n]:  # conditions for updating Cost
                        g_table[s_n] = new_cost  # 更新 g 值
                        PARENT[s_n] = s  # 记录父节点
                        OPEN.put(s_n, g_table[s_n] + self.h_table[s_n])  # 按 g + 学习到的 h 入队

            if count == N:  # expand needed CLOSED nodes
                break  # 已扩展 N 个节点，暂停本轮

        self.visited.append(CLOSED)  # visited nodes in each iteration

        return OPEN, CLOSED  # 返回剩余 OPEN 与本轮 CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        s_list = []  # 可行邻居列表

        for u in self.u_set:  # 遍历运动基元
            s_next = tuple([s[i] + u[i] for i in range(2)])  # 计算候选邻居
            if s_next not in self.obs:  # 排除障碍格
                s_list.append(s_next)  # 加入可行邻居

        return s_list  # 返回邻居列表

    def extract_path(self, x_start, parent):
        """
        Extract the path based on the relationship of nodes.

        :return: The planning path
        """

        path_back = [self.s_goal]  # 从终点反向回溯
        x_current = self.s_goal  # 当前回溯节点

        while True:  # 沿父指针回到起点
            x_current = parent[x_current]  # 取父节点
            path_back.append(x_current)  # 加入路径

            if x_current == x_start:  # 回到起点
                break  # 回溯结束

        return list(reversed(path_back))  # 反转为起点到终点顺序

    def h(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":  # 曼哈顿距离
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])  # 坐标差绝对值之和
        else:  # 其余用欧氏距离
            return math.hypot(goal[0] - s[0], goal[1] - s[1])  # 欧氏直线距离

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):  # 该运动是否发生碰撞
            return float("inf")  # 碰撞代价记为无穷

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])  # 运动欧氏长度

    def is_collision(self, s_start, s_end):
        if s_start in self.obs or s_end in self.obs:  # 端点落在障碍上
            return True  # 判定为碰撞

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:  # 斜向运动才需检查中间格
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:  # 判断斜线的方向
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))  # 斜线一侧的中间格
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))  # 斜线另一侧的中间格
            else:  # 另一条斜线方向
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))  # 斜线一侧的中间格
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))  # 斜线另一侧的中间格

            if s1 in self.obs or s2 in self.obs:  # 中间格被障碍占据
                return True  # 判定为碰撞

        return False  # 无碰撞


def main():
    s_start = (10, 5)  # 起点坐标
    s_goal = (45, 25)  # 终点坐标

    lrta = LrtAStarN(s_start, s_goal, 250, "euclidean")  # 构造 LRTA* 实例
    plot = plotting.Plotting(s_start, s_goal)  # 绘图工具

    lrta.searching()  # 执行实时搜索
    plot.animation_lrta(lrta.path, lrta.visited,  # 动画展示搜索过程
                        "Learning Real-time A* (LRTA*)")


if __name__ == '__main__':  # 脚本入口
    main()  # 运行示例
