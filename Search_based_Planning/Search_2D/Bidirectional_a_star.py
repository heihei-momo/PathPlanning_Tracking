"""
Bidirectional_a_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env


class BidirectionalAStar:
    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start = s_start  # 前向搜索起点
        self.s_goal = s_goal  # 后向搜索起点（即终点）
        self.heuristic_type = heuristic_type  # 启发式类型

        self.Env = env.Env()  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.OPEN_fore = []  # OPEN set for forward searching
        self.OPEN_back = []  # OPEN set for backward searching
        self.CLOSED_fore = []  # CLOSED set for forward
        self.CLOSED_back = []  # CLOSED set for backward
        self.PARENT_fore = dict()  # recorded parent for forward
        self.PARENT_back = dict()  # recorded parent for backward
        self.g_fore = dict()  # cost to come for forward
        self.g_back = dict()  # cost to come for backward

    def init(self):
        """
        initialize parameters
        """

        self.g_fore[self.s_start] = 0.0  # 前向起点 g 值置零
        self.g_fore[self.s_goal] = math.inf  # 前向终点 g 值先置无穷
        self.g_back[self.s_goal] = 0.0  # 后向起点（终点）g 值置零
        self.g_back[self.s_start] = math.inf  # 后向起点 g 值先置无穷
        self.PARENT_fore[self.s_start] = self.s_start  # 前向起点父节点指向自身
        self.PARENT_back[self.s_goal] = self.s_goal  # 后向起点父节点指向自身
        heapq.heappush(self.OPEN_fore,  # 前向 OPEN 压入起点
                       (self.f_value_fore(self.s_start), self.s_start))  # 元素为 (f 值, 状态)
        heapq.heappush(self.OPEN_back,  # 后向 OPEN 压入终点
                       (self.f_value_back(self.s_goal), self.s_goal))  # 元素为 (f 值, 状态)

    def searching(self):
        """
        Bidirectional A*
        :return: connected path, visited order of forward, visited order of backward
        """

        self.init()  # 初始化两张 OPEN 与 g 表
        s_meet = self.s_start  # 前向与后向的相遇点

        while self.OPEN_fore and self.OPEN_back:  # 两侧 OPEN 均非空才交替扩展
            # solve foreward-search
            _, s_fore = heapq.heappop(self.OPEN_fore)  # 前向弹出 f 最小的节点

            if s_fore in self.PARENT_back:  # 被后向搜索标记过即相遇
                s_meet = s_fore  # 记录相遇点
                break  # 双向连通，结束搜索

            self.CLOSED_fore.append(s_fore)  # 前向关闭表加入该节点

            for s_n in self.get_neighbor(s_fore):  # 遍历前向邻居
                new_cost = self.g_fore[s_fore] + self.cost(s_fore, s_n)  # 经 s_fore 到邻居的新代价

                if s_n not in self.g_fore:  # 邻居尚无前向 g 值
                    self.g_fore[s_n] = math.inf  # 前向 g 值先置无穷

                if new_cost < self.g_fore[s_n]:  # 找到更优的前向路径
                    self.g_fore[s_n] = new_cost  # 更新前向 g 值
                    self.PARENT_fore[s_n] = s_fore  # 记录前向父节点
                    heapq.heappush(self.OPEN_fore,  # 前向 OPEN 压入邻居
                                   (self.f_value_fore(s_n), s_n))  # 前向 f = g + h

            # solve backward-search
            _, s_back = heapq.heappop(self.OPEN_back)  # 后向弹出 f 最小的节点

            if s_back in self.PARENT_fore:  # 被前向搜索标记过即相遇
                s_meet = s_back  # 记录相遇点
                break  # 双向连通，结束搜索

            self.CLOSED_back.append(s_back)  # 后向关闭表加入该节点

            for s_n in self.get_neighbor(s_back):  # 遍历后向邻居
                new_cost = self.g_back[s_back] + self.cost(s_back, s_n)  # 经 s_back 到邻居的新代价

                if s_n not in self.g_back:  # 邻居尚无后向 g 值
                    self.g_back[s_n] = math.inf  # 后向 g 值先置无穷

                if new_cost < self.g_back[s_n]:  # 找到更优的后向路径
                    self.g_back[s_n] = new_cost  # 更新后向 g 值
                    self.PARENT_back[s_n] = s_back  # 记录后向父节点
                    heapq.heappush(self.OPEN_back,  # 后向 OPEN 压入邻居
                                   (self.f_value_back(s_n), s_n))  # 后向 f = g + h(到起点)

        return self.extract_path(s_meet), self.CLOSED_fore, self.CLOSED_back  # 返回路径与两侧访问序

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]  # 由运动基元生成全部邻居

    def extract_path(self, s_meet):
        """
        extract path from start and goal
        :param s_meet: meet point of bi-direction a*
        :return: path
        """

        # extract path for foreward part
        path_fore = [s_meet]  # 前向路径从相遇点回溯
        s = s_meet  # 当前回溯节点

        while True:  # 沿前向父指针回溯到起点
            s = self.PARENT_fore[s]  # 取前向父节点
            path_fore.append(s)  # 加入前向路径
            if s == self.s_start:  # 已回到起点
                break  # 前向回溯结束

        # extract path for backward part
        path_back = []  # 后向路径从相遇点出发
        s = s_meet  # 当前回溯节点

        while True:  # 沿后向父指针回溯到终点
            s = self.PARENT_back[s]  # 取后向父节点
            path_back.append(s)  # 加入后向路径
            if s == self.s_goal:  # 已到达终点
                break  # 后向回溯结束

        return list(reversed(path_fore)) + list(path_back)  # 拼接起点→相遇点→终点

    def f_value_fore(self, s):
        """
        forward searching: f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g_fore[s] + self.h(s, self.s_goal)  # 前向 f = g + h(到终点)

    def f_value_back(self, s):
        """
        backward searching: f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g_back[s] + self.h(s, self.s_start)  # 后向 f = g + h(到起点)

    def h(self, s, goal):
        """
        Calculate heuristic value.
        :param s: current node (state)
        :param goal: goal node (state)
        :return: heuristic value
        """

        heuristic_type = self.heuristic_type  # 读取启发式类型

        if heuristic_type == "manhattan":  # 曼哈顿距离
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])  # 坐标差绝对值之和
        else:  # 其余按欧氏距离
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
            return math.inf  # 碰撞代价记为无穷

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])  # 运动欧氏长度

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

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
    x_start = (5, 5)  # 起点坐标
    x_goal = (45, 25)  # 终点坐标

    bastar = BidirectionalAStar(x_start, x_goal, "euclidean")  # 构造双向 A* 实例
    plot = plotting.Plotting(x_start, x_goal)  # 绘图工具

    path, visited_fore, visited_back = bastar.searching()  # 执行双向搜索
    plot.animation_bi_astar(path, visited_fore, visited_back, "Bidirectional-A*")  # animation


if __name__ == '__main__':  # 脚本入口
    main()  # 运行示例
