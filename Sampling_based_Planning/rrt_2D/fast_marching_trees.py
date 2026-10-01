"""
Fast Marching Trees (FMT*)
@author: huiming zhou
"""

import os
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env, plotting, utils


class Node:  # FMT* 采样节点
    def __init__(self, n):  # 由坐标元组构造节点
        self.x = n[0]  # 节点 x 坐标
        self.y = n[1]  # 节点 y 坐标
        self.parent = None  # 父指针：动态规划前驱
        self.cost = np.inf  # 到起点的代价，初始无穷


class FMT:  # FMT* 规划器
    def __init__(self, x_start, x_goal, search_radius):  # 初始化起终点与搜索半径
        self.x_init = Node(x_start)  # 起点节点
        self.x_goal = Node(x_goal)  # 目标节点
        self.search_radius = search_radius  # 邻域搜索半径基准

        self.env = env.Env()  # 加载环境地图
        self.plotting = plotting.Plotting(x_start, x_goal)  # 绘图工具实例
        self.utils = utils.Utils()  # 碰撞检测等工具

        self.fig, self.ax = plt.subplots()  # 创建画布与坐标轴
        self.delta = self.utils.delta  # 采样时保留的边界余量
        self.x_range = self.env.x_range  # x 方向采样范围
        self.y_range = self.env.y_range  # y 方向采样范围
        self.obs_circle = self.env.obs_circle  # 圆形障碍列表
        self.obs_rectangle = self.env.obs_rectangle  # 矩形障碍列表
        self.obs_boundary = self.env.obs_boundary  # 地图边界障碍

        self.V = set()  # 全部节点集合
        self.V_unvisited = set()  # 未访问节点集合
        self.V_open = set()  # 开集：当前扩展前沿
        self.V_closed = set()  # 闭集：已扩展节点
        self.sample_numbers = 1000  # 采样点数量

    def Init(self):  # 初始化：采样并设置开集
        samples = self.SampleFree()  # 在自由空间生成采样点

        self.x_init.cost = 0.0  # 起点代价置为 0
        self.V.add(self.x_init)  # 起点加入节点集
        self.V.update(samples)  # 采样点全部纳入节点集
        self.V_unvisited.update(samples)  # 采样点初始均未访问
        self.V_unvisited.add(self.x_goal)  # 目标点也加入未访问集
        self.V_open.add(self.x_init)  # 开集初始仅含起点

    def Planning(self):  # 惰性动态规划主循环
        self.Init()  # 先完成初始化
        z = self.x_init  # 从起点开始扩展
        n = self.sample_numbers  # 采样规模
        rn = self.search_radius * math.sqrt((math.log(n) / n))  # 随规模收缩的邻域半径
        Visited = []  # 记录扩展顺序用于动画

        while z is not self.x_goal:  # 直到扩展到目标点
            V_open_new = set()  # 本轮新加入开集的节点
            X_near = self.Near(self.V_unvisited, z, rn)  # 未访问集中 z 的邻居
            Visited.append(z)  # 记录本次扩展的节点

            for x in X_near:  # 遍历候选邻居
                Y_near = self.Near(self.V_open, x, rn)  # x 在开集中的邻居
                cost_list = {y: y.cost + self.Cost(y, x) for y in Y_near}  # 各开集邻居的候选代价。从起点到邻居点Y的代价加上从邻居点Y到邻居点X的代价
                y_min = min(cost_list, key=cost_list.get)  # 取代价最小的前驱

                if not self.utils.is_collision(y_min, x):  # 连线无碰撞才可行
                    x.parent = y_min  # 记录最优前驱
                    V_open_new.add(x)  # x 加入开集前沿
                    self.V_unvisited.remove(x)  # 从待访问集中移除
                    x.cost = y_min.cost + self.Cost(y_min, x)  # 更新 x 的累计代价

            self.V_open.update(V_open_new)  # 批量并入开集
            self.V_open.remove(z)  # z 扩展完毕移出开集
            self.V_closed.add(z)  # z 移入闭集

            if not self.V_open:  # 开集为空说明失败
                print("open set empty!")  # 提示搜索失败
                break  # 结束主循环

            cost_open = {y: y.cost for y in self.V_open}  # 开集各节点的代价
            z = min(cost_open, key=cost_open.get)  # 选代价最小者继续扩展

        # node_end = self.ChooseGoalPoint()
        path_x, path_y = self.ExtractPath()  # 回溯得到路径坐标
        self.animation(path_x, path_y, Visited[1: len(Visited)])  # 动画展示扩展与路径

    def ChooseGoalPoint(self):  # 从邻近节点中选目标连接点
        Near = self.Near(self.V, self.x_goal, 2.0)  # 目标半径内的节点
        cost = {y: y.cost + self.Cost(y, self.x_goal) for y in Near}  # 经各邻居抵达目标的代价

        return min(cost, key=cost.get)  # 返回最优连接节点

    def ExtractPath(self):  # 沿 parent 链回溯路径
        path_x, path_y = [], []  # 路径坐标容器
        node = self.x_goal  # 从目标点开始回溯

        while node.parent:  # 回溯到起点为止
            path_x.append(node.x)  # 记录 x 坐标
            path_y.append(node.y)  # 记录 y 坐标
            node = node.parent  # 上移到父节点

        path_x.append(self.x_init.x)  # 补上起点 x 坐标
        path_y.append(self.x_init.y)  # 补上起点 y 坐标

        return path_x, path_y  # 返回路径坐标序列

    def Cost(self, x_start, x_end):  # 边代价：碰撞则为无穷
        if self.utils.is_collision(x_start, x_end):  # 若连线穿越障碍
            return np.inf  # 该边不可通行
        else:  # 无碰撞时
            return self.calc_dist(x_start, x_end)  # 以欧氏距离作为代价

    @staticmethod
    def calc_dist(x_start, x_end):  # 计算两点欧氏距离
        return math.hypot(x_start.x - x_end.x, x_start.y - x_end.y)  # 距离公式

    @staticmethod
    def Near(nodelist, z, rn):  # 取半径 rn 内的邻居
        return {nd for nd in nodelist
                if 0 < (nd.x - z.x) ** 2 + (nd.y - z.y) ** 2 <= rn ** 2}  # 排除自身并限定半径

    def SampleFree(self):  # 拒绝采样生成自由空间点
        n = self.sample_numbers  # 目标采样数量
        delta = self.utils.delta  # 采样时保留的边界余量
        Sample = set()  # 采样点集合

        ind = 0  # 已接受样本计数
        while ind < n:  # 直到采满 n 个点
            node = Node((random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))  # 边界内均匀采样
            if self.utils.is_inside_obs(node):  # 落在障碍内则丢弃
                continue  # 重新采样
            else:  # 位于自由空间时
                Sample.add(node)  # 接受该样本
                ind += 1  # 计数加一

        return Sample  # 返回自由采样点集

    def animation(self, path_x, path_y, visited):  # 动画展示规划过程
        self.plot_grid("Fast Marching Trees (FMT*)")  # 绘制地图与障碍

        for node in self.V:  # 遍历全部采样点
            plt.plot(node.x, node.y, marker='.', color='lightgrey', markersize=3)  # 灰色散点表示采样点

        count = 0  # 绘图节流计数
        for node in visited:  # 按扩展顺序绘制
            count += 1  # 计数递增
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], '-g')  # 绘制父子连线
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])  # 按 Esc 键退出动画
            if count % 10 == 0:  # 每 10 条边刷新一次
                plt.pause(0.001)  # 短暂停顿形成动画

        plt.plot(path_x, path_y, linewidth=2, color='red')  # 红色绘制最终路径
        plt.pause(0.01)  # 稍作停顿
        plt.show()  # 显示窗口

    def plot_grid(self, name):  # 绘制地图与障碍物

        for (ox, oy, w, h) in self.obs_boundary:  # 遍历边界障碍
            self.ax.add_patch(  # 向坐标轴添加图形
                patches.Rectangle(  # 构造矩形补丁
                    (ox, oy), w, h,  # 左下角坐标与宽高
                    edgecolor='black',  # 黑色边框
                    facecolor='black',  # 填充黑色
                    fill=True  # 启用填充
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:  # 遍历矩形障碍
            self.ax.add_patch(  # 向坐标轴添加图形
                patches.Rectangle(  # 构造矩形补丁
                    (ox, oy), w, h,  # 左下角坐标与宽高
                    edgecolor='black',  # 黑色边框
                    facecolor='gray',  # 灰色填充
                    fill=True  # 启用填充
                )
            )

        for (ox, oy, r) in self.obs_circle:  # 遍历圆形障碍
            self.ax.add_patch(  # 向坐标轴添加图形
                patches.Circle(  # 构造圆形补丁
                    (ox, oy), r,  # 圆心坐标与半径
                    edgecolor='black',  # 黑色边框
                    facecolor='gray',  # 灰色填充
                    fill=True  # 启用填充
                )
            )

        plt.plot(self.x_init.x, self.x_init.y, "bs", linewidth=3)  # 蓝色方块标记起点
        plt.plot(self.x_goal.x, self.x_goal.y, "rs", linewidth=3)  # 红色方块标记目标

        plt.title(name)  # 设置图标题
        plt.axis("equal")  # 等比例坐标轴


def main():  # 示例入口
    x_start = (18, 8)  # Starting node
    x_goal = (37, 18)  # Goal node

    fmt = FMT(x_start, x_goal, 40)  # 构造 FMT* 实例
    fmt.Planning()  # 执行规划


if __name__ == '__main__':  # 脚本直接运行时的入口
    main()  # 调用主函数
