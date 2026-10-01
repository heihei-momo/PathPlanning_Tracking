"""
Anytime_D_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")  # 把上级目录加入模块搜索路径

from Search_2D import plotting
from Search_2D import env


class ADStar:  # Anytime D*：带 eps 权重的增量搜索
    def __init__(self, s_start, s_goal, eps, heuristic_type):  # 初始化起点、终点、权重与启发式
        self.s_start, self.s_goal = s_start, s_goal  # 起点与终点
        self.heuristic_type = heuristic_type  # 启发式类型：manhattan 或 euclidean

        self.Env = env.Env()  # class Env
        self.Plot = plotting.Plotting(s_start, s_goal)  # 绘图工具对象

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        self.x = self.Env.x_range  # 地图 x 方向栅格数
        self.y = self.Env.y_range  # 地图 y 方向栅格数

        self.g, self.rhs, self.OPEN = {}, {}, {}  # g 值表、rhs 值表与 OPEN 字典

        for i in range(1, self.Env.x_range - 1):  # 遍历内部栅格 x 坐标
            for j in range(1, self.Env.y_range - 1):  # 遍历内部栅格 y 坐标
                self.rhs[(i, j)] = float("inf")  # rhs 初始化为无穷大
                self.g[(i, j)] = float("inf")  # g 初始化为无穷大

        self.rhs[self.s_goal] = 0.0  # 目标点 rhs=0，反向搜索起点
        self.eps = eps  # 当前启发式膨胀权重 eps
        self.OPEN[self.s_goal] = self.Key(self.s_goal)  # 目标点入 OPEN 并计算键
        self.CLOSED, self.INCONS = set(), dict()  # CLOSED 集合与不一致状态表

        self.visited = set()  # 本次搜索访问过的状态
        self.count = 0  # 绘图配色计数
        self.count_env_change = 0  # 环境变化次数（大幅变化判定）
        self.obs_add = set()  # 新增障碍集合
        self.obs_remove = set()  # 移除障碍集合
        self.title = "Anytime D*: Small changes"  # Significant changes
        self.fig = plt.figure()  # 创建绘图窗口

    def run(self):  # 主流程：先求初始路径再逐步减小 eps
        self.Plot.plot_grid(self.title)  # 绘制网格地图
        self.ComputeOrImprovePath()  # 计算或改进当前路径
        self.plot_visited()  # 绘制访问过的状态
        self.plot_path(self.extract_path())  # 绘制当前路径
        self.visited = set()  # 清空访问集合

        while True:
            if self.eps <= 1.0:  # 权重已降到 1，达到最优
                break
            self.eps -= 0.5  # 减小膨胀权重以逼近最优解
            self.OPEN.update(self.INCONS)  # 把不一致状态并回 OPEN
            for s in self.OPEN:  # 重新计算所有 OPEN 状态的键
                self.OPEN[s] = self.Key(s)  # 键值随 eps 变化需更新
            self.CLOSED = set()  # 清空 CLOSED 允许重新扩展
            self.ComputeOrImprovePath()  # 用新权重继续改进路径
            self.plot_visited()  # 绘制本次访问的状态
            self.plot_path(self.extract_path())  # 绘制新路径
            self.visited = set()  # 清空访问集合
            plt.pause(0.5)  # 暂停以显示动画

        self.fig.canvas.mpl_connect('button_press_event', self.on_press)  # 绑定鼠标点击事件
        plt.show()  # 显示交互窗口

    def on_press(self, event):  # 点击增删障碍并触发增量重规划
        x, y = event.xdata, event.ydata  # 取点击位置坐标
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:  # 点击超出地图范围
            print("Please choose right area!")
        else:
            self.count_env_change += 1  # 环境变化计数加一
            x, y = int(x), int(y)  # 转为整数栅格坐标
            print("Change position: s =", x, ",", "y =", y)

            # for small changes
            if self.title == "Anytime D*: Small changes":  # 小变化模式分支
                if (x, y) not in self.obs:  # 该处原无障碍则新增
                    self.obs.add((x, y))  # 加入障碍集合
                    self.g[(x, y)] = float("inf")  # 障碍点 g 置为无穷大
                    self.rhs[(x, y)] = float("inf")  # 障碍点 rhs 置为无穷大
                else:
                    self.obs.remove((x, y))  # 移除该障碍
                    self.UpdateState((x, y))  # 更新该状态

                self.Plot.update_obs(self.obs)  # 刷新障碍显示

                for sn in self.get_neighbor((x, y)):  # 更新该点所有邻居
                    self.UpdateState(sn)  # 重算 rhs 并决定是否入 OPEN

                plt.cla()  # 清空画布
                self.Plot.plot_grid(self.title)  # 重绘网格地图

                while True:
                    if len(self.INCONS) == 0:  # 无不一致状态则已收敛
                        break
                    self.OPEN.update(self.INCONS)  # 把不一致状态并回 OPEN
                    for s in self.OPEN:  # 重新计算所有 OPEN 状态的键
                        self.OPEN[s] = self.Key(s)  # 更新键值
                    self.CLOSED = set()  # 清空 CLOSED
                    self.ComputeOrImprovePath()  # 继续改进路径
                    self.plot_visited()  # 绘制访问状态
                    self.plot_path(self.extract_path())  # 绘制新路径
                    # plt.plot(self.title)
                    self.visited = set()  # 清空访问集合

                    if self.eps <= 1.0:  # 权重已达 1 无需继续
                        break

            else:  # 大幅变化模式分支
                if (x, y) not in self.obs:  # 原无障碍则新增
                    self.obs.add((x, y))  # 加入障碍集合
                    self.obs_add.add((x, y))  # 记录为新增障碍
                    plt.plot(x, y, 'sk')  # 用黑色方块标出
                    if (x, y) in self.obs_remove:  # 若此前记录为移除
                        self.obs_remove.remove((x, y))  # 从移除集合中删除
                else:
                    self.obs.remove((x, y))  # 移除该障碍
                    self.obs_remove.add((x, y))  # 记录为移除障碍
                    plt.plot(x, y, marker='s', color='white')  # 用白色方块擦除
                    if (x, y) in self.obs_add:  # 若此前记录为新增
                        self.obs_add.remove((x, y))  # 从新增集合中删除

                self.Plot.update_obs(self.obs)  # 刷新障碍显示

                if self.count_env_change >= 15:  # 累积足够多变化才触发重规划
                    self.count_env_change = 0  # 变化计数清零
                    self.eps += 2.0  # 提高权重先快速求次优解
                    for s in self.obs_add:  # 遍历新增障碍
                        self.g[(x, y)] = float("inf")  # 障碍处 g 置为无穷大
                        self.rhs[(x, y)] = float("inf")  # 障碍处 rhs 置为无穷大

                        for sn in self.get_neighbor(s):  # 更新障碍的邻居
                            self.UpdateState(sn)  # 重算 rhs 并入队

                    for s in self.obs_remove:  # 遍历被移除的障碍
                        for sn in self.get_neighbor(s):  # 更新其邻居
                            self.UpdateState(sn)  # 重算邻居 rhs
                        self.UpdateState(s)  # 该点恢复可通行，更新自身

                    plt.cla()  # 清空画布
                    self.Plot.plot_grid(self.title)  # 重绘网格地图

                    while True:
                        if self.eps <= 1.0:  # 权重为 1 时结束
                            break
                        self.eps -= 0.5  # 逐步降低权重求更优解
                        self.OPEN.update(self.INCONS)  # 把不一致状态并回 OPEN
                        for s in self.OPEN:  # 重新计算所有 OPEN 状态的键
                            self.OPEN[s] = self.Key(s)  # 更新键值
                        self.CLOSED = set()  # 清空 CLOSED
                        self.ComputeOrImprovePath()  # 继续改进路径
                        self.plot_visited()  # 绘制访问状态
                        self.plot_path(self.extract_path())  # 绘制新路径
                        plt.title(self.title)  # 更新图标题
                        self.visited = set()  # 清空访问集合
                        plt.pause(0.5)  # 暂停显示动画

            self.fig.canvas.draw_idle()  # 刷新画布

    def ComputeOrImprovePath(self):  # 主循环：扩展直到起点一致
        while True:
            s, v = self.TopKey()  # 取 OPEN 中键最小的状态及键值
            if v >= self.Key(self.s_start) and \
                    self.rhs[self.s_start] == self.g[self.s_start]:  # 起点已一致则收敛退出
                break

            self.OPEN.pop(s)  # 从 OPEN 中取出该状态
            self.visited.add(s)  # 记录已访问

            if self.g[s] > self.rhs[s]:  # g 偏大，属 LOWER 需下降
                self.g[s] = self.rhs[s]  # 用 rhs 更新 g
                self.CLOSED.add(s)  # 加入 CLOSED
                for sn in self.get_neighbor(s):  # 向邻居传播
                    self.UpdateState(sn)  # 更新邻居 rhs
            else:
                self.g[s] = float("inf")  # g 置无穷，进入 RAISE 处理
                for sn in self.get_neighbor(s):  # 遍历邻居
                    self.UpdateState(sn)  # 邻居可能因 s 失效而变差
                self.UpdateState(s)  # 重新评估 s 自身

    def UpdateState(self, s):  # 重算 rhs 并维护 OPEN/INCONS
        if s != self.s_goal:  # 目标点 rhs 恒为 0
            self.rhs[s] = float("inf")  # 先重置 rhs
            for x in self.get_neighbor(s):  # 遍历可行邻居
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.cost(s, x))  # 取最优一步代价作为 rhs
        if s in self.OPEN:  # 若已在 OPEN 中
            self.OPEN.pop(s)  # 先移除稍后按需重插

        if self.g[s] != self.rhs[s]:  # g 与 rhs 不一致
            if s not in self.CLOSED:  # 未扩展过则直接入 OPEN
                self.OPEN[s] = self.Key(s)  # 按新键插入 OPEN
            else:
                self.INCONS[s] = 0  # 已扩展过则放入不一致集合

    def Key(self, s):  # 计算优先队列键（含 eps 膨胀）
        if self.g[s] > self.rhs[s]:  # 局部不一致，用 rhs 估计
            return [self.rhs[s] + self.eps * self.h(self.s_start, s), self.rhs[s]]  # 含 eps 加权启发式
        else:
            return [self.g[s] + self.h(self.s_start, s), self.g[s]]  # 一致时用 g 加启发式

    def TopKey(self):
        """
        :return: return the min key and its value.
        """

        s = min(self.OPEN, key=self.OPEN.get)  # 键最小的状态
        return s, self.OPEN[s]  # 返回状态及其键值

    def h(self, s_start, s_goal):  # 启发式距离函数
        heuristic_type = self.heuristic_type  # heuristic type

        if heuristic_type == "manhattan":  # 曼哈顿距离
            return abs(s_goal[0] - s_start[0]) + abs(s_goal[1] - s_start[1])  # 两坐标差绝对值之和
        else:
            return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])  # 欧氏距离

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

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])  # 否则取欧氏距离

    def is_collision(self, s_start, s_end):  # 判断运动是否发生碰撞
        if s_start in self.obs or s_end in self.obs:  # 端点落在障碍上
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:  # 对角运动需检查斜穿
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:  # 判断是哪种对角线
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))  # 主对角线一角
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))  # 主对角线另一角
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))  # 副对角线一角
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))  # 副对角线另一角

            if s1 in self.obs or s2 in self.obs:  # 斜穿经过障碍栅格
                return True

        return False  # 无碰撞

    def get_neighbor(self, s):  # 取 s 的非障碍邻居
        nei_list = set()  # 邻居集合
        for u in self.u_set:  # 遍历可行运动
            s_next = tuple([s[i] + u[i] for i in range(2)])  # 生成相邻栅格坐标
            if s_next not in self.obs:  # 排除障碍栅格
                nei_list.add(s_next)  # 加入邻居集合

        return nei_list  # 返回邻居集合

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_start]  # 从起点出发
        s = self.s_start  # 当前节点

        for k in range(100):  # 最多走 100 步防止死循环
            g_list = {}  # 邻居 g 值表
            for x in self.get_neighbor(s):  # 遍历邻居
                if not self.is_collision(s, x):  # 排除碰撞边
                    g_list[x] = self.g[x]  # 记录邻居 g 值
            s = min(g_list, key=g_list.get)  # 选 g 最小的邻居前进
            path.append(s)  # 加入路径
            if s == self.s_goal:  # 到达终点
                break

        return list(path)  # 返回路径副本

    def plot_path(self, path):  # 绘制路径与起终点
        px = [x[0] for x in path]  # 路径 x 坐标序列
        py = [x[1] for x in path]  # 路径 y 坐标序列
        plt.plot(px, py, linewidth=2)  # 绘制路径折线
        plt.plot(self.s_start[0], self.s_start[1], "bs")  # 起点为蓝色方块
        plt.plot(self.s_goal[0], self.s_goal[1], "gs")  # 终点为绿色方块

    def plot_visited(self):  # 绘制本次访问的状态
        self.count += 1  # 配色计数递增

        color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
                 'bisque', 'navajowhite', 'moccasin', 'wheat',
                 'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']  # 备选颜色列表

        if self.count >= len(color) - 1:  # 颜色用尽则归零
            self.count = 0  # 重置配色下标

        for x in self.visited:  # 遍历访问状态
            plt.plot(x[0], x[1], marker='s', color=color[self.count])  # 绘制小方块


def main():  # 示例入口
    s_start = (5, 5)  # 起点坐标
    s_goal = (45, 25)  # 终点坐标

    dstar = ADStar(s_start, s_goal, 2.5, "euclidean")  # 初始 eps=2.5，欧氏启发式
    dstar.run()  # 运行算法


if __name__ == '__main__':
    main()
