"""
RTAAstar 2D (Real-time Adaptive A*)
@author: huiming zhou
"""

import os
import sys
import copy
import math

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import queue, plotting, env


class RTAAStar:
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

        while True:  # 每轮扩展 N 个节点，直到终点
            OPEN, CLOSED, g_table, PARENT = \
                self.Astar(s_start, self.N)  # 一轮有界 A* 返回中间结果

            if OPEN == "FOUND":  # reach the goal node
                self.path.append(CLOSED)
                break  # 已到终点，结束

            s_next, h_value = self.cal_h_value(OPEN, CLOSED, g_table, PARENT)  # 用搜索信息更新 h 值

            for x in h_value:  # 写回全局 h 表
                self.h_table[x] = h_value[x]  # 更新节点启发式

            s_start, path_k = self.extract_path_in_CLOSE(s_start, s_next, h_value)  # 取出本段要执行的路径
            self.path.append(path_k)

    def cal_h_value(self, OPEN, CLOSED, g_table, PARENT):
        v_open = {}  # OPEN 节点的代价估计
        h_value = {}  # 待更新的 h 值
        for (_, x) in OPEN.enumerate():  # 遍历 OPEN 中节点
            v_open[x] = g_table[PARENT[x]] + 1 + self.h_table[x]  # 经父节点一步到达的代价
        s_open = min(v_open, key=v_open.get)  # 取估计最小的 OPEN 节点
        f_min = v_open[s_open]  # 该节点的最小 f 值
        for x in CLOSED:  # 用 f_min 反向更新关闭节点
            h_value[x] = f_min - g_table[x]  # RTAA* 的 h 更新公式

        return s_open, h_value  # 返回下一目标节点与 h 值

    def iteration(self, CLOSED):
        h_value = {}  # CLOSED 节点的 h 值

        for s in CLOSED:  # 遍历本轮关闭节点
            h_value[s] = float("inf")  # initialize h_value of CLOSED nodes

        while True:  # 迭代至 h 值收敛
            h_value_rec = copy.deepcopy(h_value)  # 备份上一轮 h 值
            for s in CLOSED:  # 逐个节点更新 h 值
                h_list = []  # 候选后继代价列表
                for s_n in self.get_neighbor(s):  # 遍历邻居
                    if s_n not in CLOSED:  # 邻居不在本轮 CLOSED
                        h_list.append(self.cost(s, s_n) + self.h_table[s_n])  # 用全局 h 估计后继代价
                    else:  # 邻居也在 CLOSED 中
                        h_list.append(self.cost(s, s_n) + h_value[s_n])  # 用本轮新 h 估计后继代价
                h_value[s] = min(h_list)  # update h_value of current node

            if h_value == h_value_rec:  # h_value table converged
                return h_value  # 返回收敛后的 h 值

    def Astar(self, x_start, N):
        OPEN = queue.QueuePrior()  # OPEN set
        OPEN.put(x_start, self.h_table[x_start])  # 起点按学习到的 h 入队
        CLOSED = []  # CLOSED set
        g_table = {x_start: 0, self.s_goal: float("inf")}  # Cost to come
        PARENT = {x_start: x_start}  # relations
        count = 0  # counter

        while not OPEN.empty():  # OPEN 非空则继续扩展
            count += 1  # 扩展节点计数
            s = OPEN.get()  # 弹出 f 最小的节点
            CLOSED.append(s)  # 加入关闭表

            if s == self.s_goal:  # reach the goal node
                self.visited.append(CLOSED)  # 记录访问顺序
                return "FOUND", self.extract_path(x_start, PARENT), [], []  # 到达终点并回溯路径

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
                break  # 已扩展 N 个节点，本轮结束

        self.visited.append(CLOSED)  # visited nodes in each iteration

        return OPEN, CLOSED, g_table, PARENT  # 返回搜索中间结果

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        s_list = set()  # 可行邻居集合

        for u in self.u_set:  # 遍历运动基元
            s_next = tuple([s[i] + u[i] for i in range(2)])  # 计算候选邻居
            if s_next not in self.obs:  # 排除障碍格
                s_list.add(s_next)  # 加入邻居集合

        return s_list  # 返回邻居集合

    def extract_path_in_CLOSE(self, s_end, s_start, h_value):
        path = [s_start]  # 从下一目标节点出发
        s = s_start  # 当前节点

        while True:  # 沿 h 值增大的方向前进
            h_list = {}  # 邻居的 h 值表
            for s_n in self.get_neighbor(s):  # 遍历邻居
                if s_n in h_value:  # 只看本轮更新过 h 的邻居
                    h_list[s_n] = h_value[s_n]  # 记录其 h 值
            s_key = max(h_list, key=h_list.get)  # move to the smallest node with min h_value
            path.append(s_key)  # generate path
            s = s_key  # use end of this iteration as the start of next

            if s_key == s_end:  # reach the expected node in OPEN set
                return s_start, list(reversed(path))  # 返回新起点与前进路径

    def extract_path(self, x_start, parent):
        """
        Extract the path based on the relationship of nodes.
        :return: The planning path
        """

        path = [self.s_goal]  # 从终点反向回溯
        s = self.s_goal  # 当前回溯节点

        while True:  # 沿父指针回到起点
            s = parent[s]  # 取父节点
            path.append(s)  # 加入路径
            if s == x_start:  # 回到起点
                break  # 回溯结束

        return list(reversed(path))  # 反转为起点到终点顺序

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

    rtaa = RTAAStar(s_start, s_goal, 240, "euclidean")  # 构造 RTAA* 实例
    plot = plotting.Plotting(s_start, s_goal)  # 绘图工具

    rtaa.searching()  # 执行实时自适应搜索
    plot.animation_lrta(rtaa.path, rtaa.visited,  # 动画展示搜索过程
                        "Real-time Adaptive A* (RTAA*)")


if __name__ == '__main__':  # 脚本入口
    main()  # 运行示例
