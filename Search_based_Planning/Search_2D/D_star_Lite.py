"""
D_star_Lite 2D
@author: huiming zhou
"""

import os
import sys
import math
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")  # 追加搜索算法根目录到模块路径

from Search_2D import plotting, env  # 导入绘图与环境模块


class DStar:
    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start, self.s_goal = s_start, s_goal  # 起点与目标点
        self.heuristic_type = heuristic_type  # 启发函数类型：欧氏或曼哈顿

        self.Env = env.Env()  # class Env
        self.Plot = plotting.Plotting(s_start, s_goal)  # 绘图对象

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        self.x = self.Env.x_range  # 地图 x 方向范围
        self.y = self.Env.y_range  # 地图 y 方向范围

        self.g, self.rhs, self.U = {}, {}, {}  # g 值表、rhs 值表、U 优先队列
        self.km = 0  # 起点位移累积的启发式修正量

        for i in range(1, self.Env.x_range - 1):  # 遍历 x 方向内部栅格
            for j in range(1, self.Env.y_range - 1):  # 遍历 y 方向内部栅格
                self.rhs[(i, j)] = float("inf")  # rhs 初始化为无穷大
                self.g[(i, j)] = float("inf")  # g 初始化为无穷大

        self.rhs[self.s_goal] = 0.0  # 目标点 rhs 置 0，作为搜索源头
        self.U[self.s_goal] = self.CalculateKey(self.s_goal)  # 目标点入优先队列
        self.visited = set()  # 记录已扩展节点，用于绘图
        self.count = 0  # 重规划轮次计数，用于配色
        self.fig = plt.figure()  # 创建画布

    def run(self):
        self.Plot.plot_grid("D* Lite")  # 绘制栅格地图
        self.ComputePath()  # 首次搜索最短路径
        self.plot_path(self.extract_path())  # 回溯并绘制路径
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)  # 绑定鼠标点击事件
        plt.show()  # 显示交互窗口

    def on_press(self, event):
        x, y = event.xdata, event.ydata  # 获取鼠标点击坐标
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:  # 点击越界则忽略
            print("Please choose right area!")  # 提示点击区域非法
        else:
            x, y = int(x), int(y)  # 坐标取整为栅格索引
            print("Change position: s =", x, ",", "y =", y)  # 打印变更的位置

            s_curr = self.s_start  # 当前移动到的节点
            s_last = self.s_start  # 上次计算 km 时的参考节点
            i = 0  # 标记本次点击是否已处理
            path = [self.s_start]  # 记录机器人实际行走路径

            while s_curr != self.s_goal:  # 沿 g 值下降方向走向目标
                s_list = {}  # 邻居 -> 一步后继的 g 值

                for s in self.get_neighbor(s_curr):  # 枚举当前点可行邻居
                    s_list[s] = self.g[s] + self.cost(s_curr, s)  # 记录邻居的代价值
                s_curr = min(s_list, key=s_list.get)  # 选代价最小的邻居前进
                path.append(s_curr)  # 加入行走路径

                if i < 1:  # 仅在首次经过时处理障碍变化
                    self.km += self.h(s_last, s_curr)  # 累积起点位移的启发式修正
                    s_last = s_curr  # 更新 km 的参考节点
                    if (x, y) not in self.obs:  # 点击处原为空地：新增障碍
                        self.obs.add((x, y))  # 障碍集合加入该点
                        plt.plot(x, y, 'sk')  # 绘制黑色障碍方块
                        self.g[(x, y)] = float("inf")  # 新障碍 g 值置无穷
                        self.rhs[(x, y)] = float("inf")  # 新障碍 rhs 值置无穷
                    else:  # 点击处原为障碍：删除该障碍
                        self.obs.remove((x, y))  # 从障碍集合移除
                        plt.plot(x, y, marker='s', color='white')  # 用白色恢复为空地
                        self.UpdateVertex((x, y))  # 更新该点的 rhs 与键值
                    for s in self.get_neighbor((x, y)):  # 枚举受影响的邻居
                        self.UpdateVertex(s)  # 逐个更新邻居顶点
                    i += 1  # 标记本次点击已处理

                    self.count += 1  # 重规划轮次加一
                    self.visited = set()  # 清空上一轮扩展记录
                    self.ComputePath()  # 增量重规划最短路径

            self.plot_visited(self.visited)  # 绘制本轮扩展过的节点
            self.plot_path(path)  # 绘制实际行走路径
            self.fig.canvas.draw_idle()  # 刷新画布显示

    def ComputePath(self):
        while True:  # 反复处理队首节点直到起点一致
            s, v = self.TopKey()  # 取优先队列中最小键的节点与键值
            if v >= self.CalculateKey(self.s_start) and \
                    self.rhs[self.s_start] == self.g[self.s_start]:  # 起点已局部一致则收敛
                break

            k_old = v  # 记录该节点弹出时的旧键值
            self.U.pop(s)  # 从优先队列中移除
            self.visited.add(s)  # 记录为已扩展节点

            if k_old < self.CalculateKey(s):  # 键值变大：节点需重新入队
                self.U[s] = self.CalculateKey(s)  # 以新键值重新入队
            elif self.g[s] > self.rhs[s]:  # 局部过一致：g 可直接降为 rhs
                self.g[s] = self.rhs[s]  # 用 rhs 更新 g 值
                for x in self.get_neighbor(s):  # 枚举受影响的邻居
                    self.UpdateVertex(x)  # 重算邻居的 rhs 与键值
            else:  # 局部欠一致：原 g 值已失效
                self.g[s] = float("inf")  # g 置无穷，等待重新计算
                self.UpdateVertex(s)  # 重算自身的 rhs 与键值
                for x in self.get_neighbor(s):  # 枚举受影响的邻居
                    self.UpdateVertex(x)  # 逐个更新邻居

    def UpdateVertex(self, s):
        if s != self.s_goal:  # 目标点的 rhs 恒为 0，无需更新
            self.rhs[s] = float("inf")  # 先置无穷再取最小值
            for x in self.get_neighbor(s):  # 枚举所有可行邻居
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.cost(s, x))  # 取最优后继的一步代价
        if s in self.U:  # 若已在优先队列中
            self.U.pop(s)  # 先移除旧键值

        if self.g[s] != self.rhs[s]:  # 局部不一致则需要重新入队
            self.U[s] = self.CalculateKey(s)  # 计算新键值并入队

    def CalculateKey(self, s):
        return [min(self.g[s], self.rhs[s]) + self.h(self.s_start, s) + self.km,  # 主键含 km 增量修正
                min(self.g[s], self.rhs[s])]  # 次键：取 g 与 rhs 的较小者

    def TopKey(self):
        """
        :return: return the min key and its value.
        """

        s = min(self.U, key=self.U.get)  # 按键的字典序取最小节点
        return s, self.U[s]  # 返回该节点及其键值

    def h(self, s_start, s_goal):
        heuristic_type = self.heuristic_type  # heuristic type

        if heuristic_type == "manhattan":
            return abs(s_goal[0] - s_start[0]) + abs(s_goal[1] - s_start[1])  # 横纵坐标距离之和
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

        if self.is_collision(s_start, s_goal):
            return float("inf")  # 该移动被障碍阻挡，代价无穷

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])  # 欧氏距离作为移动代价

    def is_collision(self, s_start, s_end):
        if s_start in self.obs or s_end in self.obs:
            return True  # 端点落在障碍上，判定碰撞

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:  # 仅斜向移动需检查中间角格
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:  # 区分两条对角线方向
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))  # 主对角线一端
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))  # 主对角线另一端
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))  # 副对角线一端
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))  # 副对角线另一端

            if s1 in self.obs or s2 in self.obs:
                return True  # 对角穿过的角格有障碍，判定碰撞

        return False  # 无碰撞

    def get_neighbor(self, s):
        nei_list = set()  # 可行邻居集合
        for u in self.u_set:  # 遍历八个运动方向
            s_next = tuple([s[i] + u[i] for i in range(2)])  # 计算相邻节点坐标
            if s_next not in self.obs:  # 排除落在障碍上的节点
                nei_list.add(s_next)  # 加入可行邻居

        return nei_list  # 返回可行邻居集合

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_start]  # 从起点开始回溯路径
        s = self.s_start  # 当前回溯节点

        for k in range(100):  # 最多回溯 100 步
            g_list = {}  # 邻居 -> 其 g 值
            for x in self.get_neighbor(s):  # 枚举可行邻居
                if not self.is_collision(s, x):  # 跳过发生碰撞的移动
                    g_list[x] = self.g[x]  # 记录邻居的 g 值
            s = min(g_list, key=g_list.get)  # 选择 g 值最小的邻居
            path.append(s)  # 加入路径
            if s == self.s_goal:  # 到达目标点则结束
                break  # 跳出回溯循环

        return list(path)  # 返回起点到目标点的路径

    def plot_path(self, path):
        px = [x[0] for x in path]  # 路径点的 x 坐标序列
        py = [x[1] for x in path]  # 路径点的 y 坐标序列
        plt.plot(px, py, linewidth=2)  # 绘制路径折线
        plt.plot(self.s_start[0], self.s_start[1], "bs")  # 绘制起点蓝色方块
        plt.plot(self.s_goal[0], self.s_goal[1], "gs")  # 绘制目标绿色方块

    def plot_visited(self, visited):
        color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
                 'bisque', 'navajowhite', 'moccasin', 'wheat',
                 'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']  # 各轮扩展节点的配色表

        if self.count >= len(color) - 1:  # 配色索引越界则归零
            self.count = 0  # 重置配色索引

        for x in visited:  # 遍历本轮扩展过的节点
            plt.plot(x[0], x[1], marker='s', color=color[self.count])  # 用当前颜色画方块


def main():
    s_start = (5, 5)  # 起点坐标
    s_goal = (45, 25)  # 目标点坐标

    dstar = DStar(s_start, s_goal, "euclidean")  # 构造 D* Lite 对象
    dstar.run()  # 启动交互式搜索


if __name__ == '__main__':  # 主程序入口
    main()  # 运行主函数
