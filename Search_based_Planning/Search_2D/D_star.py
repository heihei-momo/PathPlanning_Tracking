"""
D_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")  # 把上级目录加入模块搜索路径

from Search_2D import plotting, env


class DStar:  # D* 算法：动态 A*
    def __init__(self, s_start, s_goal):  # 初始化：记录起点、终点与环境
        self.s_start, self.s_goal = s_start, s_goal  # 起点与终点

        self.Env = env.Env()  # 环境对象：地图与障碍物
        self.Plot = plotting.Plotting(self.s_start, self.s_goal)  # 绘图工具对象

        self.u_set = self.Env.motions  # 可行运动集合（8 邻域）
        self.obs = self.Env.obs  # 障碍物坐标集合
        self.x = self.Env.x_range  # 地图 x 方向栅格数
        self.y = self.Env.y_range  # 地图 y 方向栅格数

        self.fig = plt.figure()  # 创建绘图窗口

        self.OPEN = set()  # OPEN 表：待扩展状态集合
        self.t = dict()  # 状态标签表：NEW/OPEN/CLOSED
        self.PARENT = dict()  # 父节点表：用于回溯路径
        self.h = dict()  # h 值表：状态到目标的代价估计
        self.k = dict()  # k 值表：OPEN 状态的优先队列键
        self.path = []  # 规划得到的路径
        self.visited = set()  # 已扩展状态集合（用于绘图）
        self.count = 0  # 环境变化次数（用于配色）

    def init(self):  # 初始化全部状态为 NEW
        for i in range(self.Env.x_range):  # 遍历地图所有 x 坐标
            for j in range(self.Env.y_range):  # 遍历地图所有 y 坐标
                self.t[(i, j)] = 'NEW'  # 初始状态均未访问
                self.k[(i, j)] = 0.0  # 键值初始化为 0
                self.h[(i, j)] = float("inf")  # h 值初始化为无穷大
                self.PARENT[(i, j)] = None  # 父节点初始为空

        self.h[self.s_goal] = 0.0  # 目标点 h=0，作为反向搜索起点

    def run(self, s_start, s_end):  # 执行 D* 主流程
        self.init()  # 初始化状态表
        self.insert(s_end, 0)  # 把目标点以 h=0 插入 OPEN

        while True:
            self.process_state()  # 处理一个状态并传播代价变化
            if self.t[s_start] == 'CLOSED':  # 起点已 CLOSED 说明最优解确定
                break

        self.path = self.extract_path(s_start, s_end)  # 沿 PARENT 回溯得到路径
        self.Plot.plot_grid("Dynamic A* (D*)")  # 绘制网格地图
        self.plot_path(self.path)  # 绘制路径
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)  # 绑定鼠标点击事件
        plt.show()  # 显示交互窗口

    def on_press(self, event):  # 鼠标点击回调：动态增删障碍
        x, y = event.xdata, event.ydata  # 取点击位置坐标
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:  # 点击超出地图范围
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)  # 转为整数栅格坐标
            if (x, y) not in self.obs:  # 原无障碍则新增障碍
                print("Add obstacle at: s =", x, ",", "y =", y)
                self.obs.add((x, y))  # 把该点加入障碍集合
                self.Plot.update_obs(self.obs)  # 刷新障碍显示

                s = self.s_start  # 从起点开始检查路径是否被阻断
                self.visited = set()  # 清空已访问集合
                self.count += 1  # 环境变化计数加一

                while s != self.s_goal:  # 沿当前路径逐点检查
                    if self.is_collision(s, self.PARENT[s]):  # 该边被新障碍阻断
                        self.modify(s)  # 修改代价并重规划
                        continue
                    s = self.PARENT[s]  # 沿父节点前移

                self.path = self.extract_path(self.s_start, self.s_goal)  # 重新提取路径

                plt.cla()  # 清空画布
                self.Plot.plot_grid("Dynamic A* (D*)")  # 重绘网格地图
                self.plot_visited(self.visited)  # 绘制重规划访问过的状态
                self.plot_path(self.path)  # 绘制新路径

            self.fig.canvas.draw_idle()  # 刷新画布

    def extract_path(self, s_start, s_end):  # 由 PARENT 回溯路径
        path = [s_start]  # 路径从起点开始
        s = s_start  # 当前回溯节点
        while True:
            s = self.PARENT[s]  # 取父节点
            path.append(s)  # 加入路径
            if s == s_end:  # 到达终点则结束
                return path

    def process_state(self):  # 处理 k 最小的状态（D* 核心）
        s = self.min_state()  # get node in OPEN set with min k value
        self.visited.add(s)  # 记录该状态已访问

        if s is None:  # OPEN 为空则搜索失败
            return -1  # OPEN set is empty

        k_old = self.get_k_min()  # record the min k value of this iteration (min path cost)
        self.delete(s)  # move state s from OPEN set to CLOSED set

        # k_min < h[s] --> s: RAISE state (increased cost)
        if k_old < self.h[s]:  # 旧键小于 h，代价上升
            for s_n in self.get_neighbor(s):  # 遍历 s 的所有可行邻居
                if self.h[s_n] <= k_old and \
                        self.h[s] > self.h[s_n] + self.cost(s_n, s):  # 邻居更优可降低 h[s]

                    # update h_value and choose parent
                    self.PARENT[s] = s_n  # 改选该邻居为父节点
                    self.h[s] = self.h[s_n] + self.cost(s_n, s)  # 更新 h 值

        # s: k_min >= h[s] -- > s: LOWER state (cost reductions)
        if k_old == self.h[s]:  # 旧键等于 h，代价下降
            for s_n in self.get_neighbor(s):  # 遍历邻居
                if self.t[s_n] == 'NEW' or \
                        (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)) or \
                        (self.PARENT[s_n] != s and self.h[s_n] > self.h[s] + self.cost(s, s_n)):

                    # Condition:
                    # 1) t[s_n] == 'NEW': not visited
                    # 2) s_n's parent: cost reduction
                    # 3) s_n find a better parent
                    self.PARENT[s_n] = s  # 把 s 设为 s_n 的父节点
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))  # 用新代价插入 OPEN
        else:
            for s_n in self.get_neighbor(s):  # 遍历邻居
                if self.t[s_n] == 'NEW' or \
                        (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)):

                    # Condition:
                    # 1) t[s_n] == 'NEW': not visited
                    # 2) s_n's parent: cost reduction
                    self.PARENT[s_n] = s  # 把 s 设为 s_n 的父节点
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))  # 用新代价插入 OPEN
                else:
                    if self.PARENT[s_n] != s and \
                            self.h[s_n] > self.h[s] + self.cost(s, s_n):  # 邻居可经 s 获得更小 h

                        # Condition: LOWER happened in OPEN set (s), s should be explored again
                        self.insert(s, self.h[s])  # OPEN 中代价下降，重新插入 s
                    else:
                        if self.PARENT[s_n] != s and \
                                self.h[s] > self.h[s_n] + self.cost(s_n, s) and \
                                self.t[s_n] == 'CLOSED' and \
                                self.h[s_n] > k_old:  # CLOSED 邻居代价下降且高于 k_old

                            # Condition: LOWER happened in CLOSED set (s_n), s_n should be explored again
                            self.insert(s_n, self.h[s_n])  # 重新插入该邻居

        return self.get_k_min()  # 返回本次迭代的最小键

    def min_state(self):
        """
        choose the node with the minimum k value in OPEN set.
        :return: state
        """

        if not self.OPEN:  # OPEN 为空则无解
            return None

        return min(self.OPEN, key=lambda x: self.k[x])  # 按键值取最小的状态

    def get_k_min(self):
        """
        calc the min k value for nodes in OPEN set.
        :return: k value
        """

        if not self.OPEN:  # OPEN 为空
            return -1

        return min([self.k[x] for x in self.OPEN])  # 返回最小键值

    def insert(self, s, h_new):
        """
        insert node into OPEN set.
        :param s: node
        :param h_new: new or better cost to come value
        """

        if self.t[s] == 'NEW':  # 首次进入 OPEN
            self.k[s] = h_new  # 键值取新代价
        elif self.t[s] == 'OPEN':  # 已在 OPEN 中
            self.k[s] = min(self.k[s], h_new)  # 键值取更小者
        elif self.t[s] == 'CLOSED':  # 曾被扩展过（CLOSED）
            self.k[s] = min(self.h[s], h_new)  # 键值取旧 h 与新代价较小者

        self.h[s] = h_new  # 更新 h 值
        self.t[s] = 'OPEN'  # 标记为 OPEN
        self.OPEN.add(s)  # 加入 OPEN 集合

    def delete(self, s):
        """
        delete: move state s from OPEN set to CLOSED set.
        :param s: state should be deleted
        """

        if self.t[s] == 'OPEN':  # 仅 OPEN 状态需改标签
            self.t[s] = 'CLOSED'  # 移入 CLOSED

        self.OPEN.remove(s)  # 从 OPEN 集合移除

    def modify(self, s):
        """
        start processing from state s.
        :param s: is a node whose status is RAISE or LOWER.
        """

        self.modify_cost(s)  # 重算 s 到父节点的代价

        while True:
            k_min = self.process_state()  # 处理状态并取最小键

            if k_min >= self.h[s]:  # 传播收敛则结束重规划
                break

    def modify_cost(self, s):  # 障碍变化后重算受影响边的代价
        # if node in CLOSED set, put it into OPEN set.
        # Since cost may be changed between s - s.parent, calc cost(s, s.p) again

        if self.t[s] == 'CLOSED':  # 只处理已扩展过的状态
            self.insert(s, self.h[self.PARENT[s]] + self.cost(s, self.PARENT[s]))  # 用父节点代价重新入队

    def get_neighbor(self, s):  # 取 s 的非障碍邻居
        nei_list = set()  # 邻居集合

        for u in self.u_set:  # 遍历所有可行运动
            s_next = tuple([s[i] + u[i] for i in range(2)])  # 生成相邻栅格坐标
            if s_next not in self.obs:  # 排除障碍栅格
                nei_list.add(s_next)  # 加入邻居集合

        return nei_list  # 返回邻居集合

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):  # 该运动被阻挡
            return float("inf")  # 代价为无穷大

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])  # 否则代价为欧氏距离

    def is_collision(self, s_start, s_end):  # 判断运动是否发生碰撞
        if s_start in self.obs or s_end in self.obs:  # 端点落在障碍上
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:  # 对角运动需检查斜穿
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:  # 判断是哪种对角线
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))  # 主对角线左下角栅格
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))  # 主对角线右上角栅格
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))  # 副对角线左上角栅格
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))  # 副对角线右下角栅格

            if s1 in self.obs or s2 in self.obs:  # 斜穿经过障碍栅格
                return True

        return False  # 无碰撞

    def plot_path(self, path):  # 绘制路径与起终点
        px = [x[0] for x in path]  # 路径 x 坐标序列
        py = [x[1] for x in path]  # 路径 y 坐标序列
        plt.plot(px, py, linewidth=2)  # 绘制路径折线
        plt.plot(self.s_start[0], self.s_start[1], "bs")  # 起点为蓝色方块
        plt.plot(self.s_goal[0], self.s_goal[1], "gs")  # 终点为绿色方块

    def plot_visited(self, visited):  # 按配色计数绘制访问状态
        color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
                 'bisque', 'navajowhite', 'moccasin', 'wheat',
                 'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']  # 备选颜色列表

        if self.count >= len(color) - 1:  # 颜色用尽则循环
            self.count = 0  # 重置配色下标

        for x in visited:  # 遍历本次访问的状态
            plt.plot(x[0], x[1], marker='s', color=color[self.count])  # 绘制小方块


def main():  # 示例入口
    s_start = (5, 5)  # 起点坐标
    s_goal = (45, 25)  # 终点坐标
    dstar = DStar(s_start, s_goal)  # 构造 D* 对象
    dstar.run(s_start, s_goal)  # 运行算法


if __name__ == '__main__':
    main()
