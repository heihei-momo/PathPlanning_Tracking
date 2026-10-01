"""
RRT_STAR_SMART 2D
@author: huiming zhou
"""

import os  # 操作系统路径接口
import sys  # 系统模块搜索路径
import math  # 数学函数库
import random  # 随机采样
import numpy as np  # 数值计算
import matplotlib.pyplot as plt  # 绘图
import matplotlib.patches as patches  # 障碍物图元
from scipy.spatial.transform import Rotation as Rot  # 旋转矩阵构造

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")  # 追加仓库根目录到搜索路径

from Sampling_based_Planning.rrt_2D import env, plotting, utils  # 环境/绘图/工具模块


class Node:  # 树节点
    def __init__(self, n):  # 由坐标构造节点
        self.x = n[0]  # 节点 x 坐标
        self.y = n[1]  # 节点 y 坐标
        self.parent = None  # 父节点，根为 None


class RrtStarSmart:  # RRT*-Smart 规划器
    def __init__(self, x_start, x_goal, step_len,  # 起点、终点、步长
                 goal_sample_rate, search_radius, iter_max):  # 目标偏置率、近邻半径、迭代上限
        self.x_start = Node(x_start)  # 起点节点
        self.x_goal = Node(x_goal)  # 终点节点
        self.step_len = step_len  # 扩展步长
        self.goal_sample_rate = goal_sample_rate  # 采样到终点的概率
        self.search_radius = search_radius  # 近邻半径系数
        self.iter_max = iter_max  # 最大迭代次数

        self.env = env.Env()  # 地图环境
        self.plotting = plotting.Plotting(x_start, x_goal)  # 绘图工具
        self.utils = utils.Utils()  # 碰撞检测等工具

        self.fig, self.ax = plt.subplots()  # 创建画布与坐标轴
        self.delta = self.utils.delta  # 安全膨胀距离
        self.x_range = self.env.x_range  # 地图 x 范围
        self.y_range = self.env.y_range  # 地图 y 范围
        self.obs_circle = self.env.obs_circle  # 圆形障碍
        self.obs_rectangle = self.env.obs_rectangle  # 矩形障碍
        self.obs_boundary = self.env.obs_boundary  # 边界障碍

        self.V = [self.x_start]  # 树节点集合
        self.beacons = []  # 信标点：障碍物拐角
        self.beacons_radius = 2  # 信标采样半径
        self.direct_cost_old = np.inf  # 上次优化后的路径代价
        self.obs_vertex = self.utils.get_obs_vertex()  # 障碍物顶点集合
        self.path = None  # 最终路径

    def planning(self):  # 主循环
        n = 0  # 找到初始路径的迭代序号
        b = 2  # 信标采样间隔
        InitPathFlag = False  # 是否已找到初始路径
        self.ReformObsVertex()  # 展开障碍物顶点列表

        for k in range(self.iter_max):  # 迭代扩展
            if k % 200 == 0:  # 每 200 次打印进度
                print(k)  # 输出迭代号

            if (k - n) % b == 0 and len(self.beacons) > 0:  # 到达信标采样周期
                x_rand = self.Sample(self.beacons)  # 拐角附近采样
            else:  # 否则全局采样
                x_rand = self.Sample()  # 全局随机采样

            x_nearest = self.Nearest(self.V, x_rand)  # 最近邻节点
            x_new = self.Steer(x_nearest, x_rand)  # 朝采样点扩展

            if x_new and not self.utils.is_collision(x_nearest, x_new):  # 新边无碰撞
                X_near = self.Near(self.V, x_new)  # 近邻节点集合
                self.V.append(x_new)  # 新节点加入树

                if X_near:  # 存在近邻
                    # choose parent
                    cost_list = [self.Cost(x_near) + self.Line(x_near, x_new) for x_near in X_near]  # 代价表
                    x_new.parent = X_near[int(np.argmin(cost_list))]  # 选代价最小的近邻为父

                    # rewire
                    c_min = self.Cost(x_new)  # 新节点当前代价
                    for x_near in X_near:  # 遍历近邻尝试重连
                        c_near = self.Cost(x_near)  # 近邻原代价
                        c_new = c_min + self.Line(x_new, x_near)  # 经新节点到达的代价
                        if c_new < c_near:  # 经新节点更省
                            x_near.parent = x_new  # 重连父节点

                if not InitPathFlag and self.InitialPathFound(x_new):  # 首次找到初始路径
                    InitPathFlag = True  # 置位初始路径标志
                    n = k  # 记录该迭代号

                if InitPathFlag:  # 已找到初始路径后
                    self.PathOptimization(x_new)  # 路径优化并更新信标
                if k % 5 == 0:  # 每 5 次迭代
                    self.animation()  # 刷新绘图

        self.path = self.ExtractPath()  # 回溯最终路径
        self.animation()  # 最终绘制
        plt.plot([x for x, _ in self.path], [y for _, y in self.path], '-r')  # 红色画出路径
        plt.pause(0.01)  # 短暂停顿
        plt.show()  # 保持窗口显示

    def PathOptimization(self, node):  # 贪心缩短路径
        direct_cost_new = 0.0  # 本次优化后路径长度
        node_end = self.x_goal  # 从终点开始回溯

        while node.parent:  # 沿树向上回溯
            node_parent = node.parent  # 父节点
            if not self.utils.is_collision(node_parent, node_end):  # 可直连更远节点
                node_end.parent = node_parent  # 跳过中间节点
            else:  # 有障碍阻挡
                direct_cost_new += self.Line(node, node_end)  # 累加该段直连代价
                node_end = node  # 以当前节点为新端点

            node = node_parent  # 上溯

        if direct_cost_new < self.direct_cost_old:  # 路径变短
            self.direct_cost_old = direct_cost_new  # 记录新代价
            self.UpdateBeacons()  # 更新障碍拐角信标

    def UpdateBeacons(self):  # 更新信标点
        node = self.x_goal  # 从终点开始
        beacons = []  # 信标列表

        while node.parent:  # 沿路径回溯
            near_vertex = [v for v in self.obs_vertex  # 遍历障碍顶点
                           if (node.x - v[0]) ** 2 + (node.y - v[1]) ** 2 < 9]  # 距路径点 3 以内的顶点
            if len(near_vertex) > 0:  # 存在邻近顶点
                for v in near_vertex:  # 逐个加入
                    beacons.append(v)  # 记录为信标

            node = node.parent  # 上溯

        self.beacons = beacons  # 更新信标集合

    def ReformObsVertex(self):  # 展平障碍顶点
        obs_vertex = []  # 顶点列表

        for obs in self.obs_vertex:  # 遍历每个障碍
            for vertex in obs:  # 遍历其顶点
                obs_vertex.append(vertex)  # 收集顶点

        self.obs_vertex = obs_vertex  # 替换为展平列表

    def Steer(self, x_start, x_goal):  # 朝目标方向扩展一步
        dist, theta = self.get_distance_and_angle(x_start, x_goal)  # 距离与方向
        dist = min(self.step_len, dist)  # 步长截断
        node_new = Node((x_start.x + dist * math.cos(theta),  # 新节点 x 坐标
                         x_start.y + dist * math.sin(theta)))  # 新节点 y 坐标
        node_new.parent = x_start  # 暂定父节点

        return node_new  # 返回新节点

    def Near(self, nodelist, node):  # 近邻搜索
        n = len(self.V) + 1  # 当前节点数
        r = 50 * math.sqrt((math.log(n) / n))  # 随节点数衰减的邻域半径

        dist_table = [(nd.x - node.x) ** 2 + (nd.y - node.y) ** 2 for nd in nodelist]  # 距离平方表
        X_near = [nodelist[ind] for ind in range(len(dist_table)) if dist_table[ind] <= r ** 2 and  # 半径内
                  not self.utils.is_collision(node, nodelist[ind])]  # 且无碰撞

        return X_near  # 近邻节点列表

    def Sample(self, goal=None):  # 采样函数
        if goal is None:  # 无信标时全局采样
            delta = self.utils.delta  # 安全边距
            goal_sample_rate = self.goal_sample_rate  # 目标偏置率

            if np.random.random() > goal_sample_rate:  # 非目标偏置
                return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),  # 随机 x
                             np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))  # 随机 y

            return self.x_goal  # 直接返回终点
        else:  # 有信标时拐角采样
            R = self.beacons_radius  # 信标采样半径
            r = random.uniform(0, R)  # 随机半径
            theta = random.uniform(0, 2 * math.pi)  # 随机方向角
            ind = random.randint(0, len(goal) - 1)  # 随机选一个信标

            return Node((goal[ind][0] + r * math.cos(theta),  # 信标周围 x
                         goal[ind][1] + r * math.sin(theta)))  # 信标周围 y

    def SampleFreeSpace(self):  # 全局采样（未使用）
        delta = self.delta  # 安全边距

        if np.random.random() > self.goal_sample_rate:  # 非目标偏置
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),  # 随机 x
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))  # 随机 y

        return self.x_goal  # 直接返回终点

    def ExtractPath(self):  # 回溯路径
        path = []  # 路径点列表
        node = self.x_goal  # 从终点开始

        while node.parent:  # 沿父指针上溯
            path.append([node.x, node.y])  # 记录当前节点
            node = node.parent  # 移动到父节点

        path.append([self.x_start.x, self.x_start.y])  # 补上起点

        return path  # 返回路径

    def InitialPathFound(self, node):  # 是否找到初始路径
        if self.Line(node, self.x_goal) < self.step_len:  # 距终点小于一步
            return True  # 视为连通

        return False  # 未连通

    @staticmethod  # 静态方法
    def Nearest(nodelist, n):  # 最近邻节点
        return nodelist[int(np.argmin([(nd.x - n.x) ** 2 + (nd.y - n.y) ** 2  # 距离平方最小
                                       for nd in nodelist]))]  # 遍历所有节点

    @staticmethod  # 静态方法
    def Line(x_start, x_goal):  # 两点欧氏距离
        return math.hypot(x_goal.x - x_start.x, x_goal.y - x_start.y)  # 直线代价

    @staticmethod  # 静态方法
    def Cost(node):  # 节点到起点代价
        cost = 0.0  # 累计代价
        if node.parent is None:  # 根节点
            return cost  # 代价为零

        while node.parent:  # 沿父指针累加
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)  # 累加边长
            node = node.parent  # 上溯

        return cost  # 返回总代价

    @staticmethod  # 静态方法
    def get_distance_and_angle(node_start, node_end):  # 距离与方位角
        dx = node_end.x - node_start.x  # 横向差
        dy = node_end.y - node_start.y  # 纵向差
        return math.hypot(dx, dy), math.atan2(dy, dx)  # 距离与夹角

    def animation(self):  # 动态绘制
        plt.cla()  # 清空画布
        self.plot_grid("rrt*-Smart, N = " + str(self.iter_max))  # 绘制栅格与障碍
        plt.gcf().canvas.mpl_connect(  # 绑定按键
            'key_release_event',  # 按键释放事件
            lambda event: [exit(0) if event.key == 'escape' else None])  # ESC 退出

        for node in self.V:  # 遍历树
            if node.parent:  # 非根节点
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")  # 绿色画树边

        if self.beacons:  # 存在信标
            theta = np.arange(0, 2 * math.pi, 0.1)  # 参数角序列
            r = self.beacons_radius  # 信标半径

            for v in self.beacons:  # 遍历信标
                x = v[0] + r * np.cos(theta)  # 圆周点 x
                y = v[1] + r * np.sin(theta)  # 圆周点 y
                plt.plot(x, y, linestyle='--', linewidth=2, color='darkorange')  # 橙色虚线画信标圆

        plt.pause(0.01)  # 停顿刷新

    def plot_grid(self, name):  # 绘制地图

        for (ox, oy, w, h) in self.obs_boundary:  # 边界障碍
            self.ax.add_patch(  # 添加图元
                patches.Rectangle(  # 矩形图元
                    (ox, oy), w, h,  # 左下角与宽高
                    edgecolor='black',  # 黑色边框
                    facecolor='black',  # 黑色填充
                    fill=True  # 实心填充
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:  # 矩形障碍
            self.ax.add_patch(  # 添加图元
                patches.Rectangle(  # 矩形图元
                    (ox, oy), w, h,  # 左下角与宽高
                    edgecolor='black',  # 黑色边框
                    facecolor='gray',  # 灰色填充
                    fill=True  # 实心填充
                )
            )

        for (ox, oy, r) in self.obs_circle:  # 圆形障碍
            self.ax.add_patch(  # 添加图元
                patches.Circle(  # 圆形图元
                    (ox, oy), r,  # 圆心与半径
                    edgecolor='black',  # 黑色边框
                    facecolor='gray',  # 灰色填充
                    fill=True  # 实心填充
                )
            )

        plt.plot(self.x_start.x, self.x_start.y, "bs", linewidth=3)  # 蓝色方块标起点
        plt.plot(self.x_goal.x, self.x_goal.y, "rs", linewidth=3)  # 红色方块标终点

        plt.title(name)  # 图标题
        plt.axis("equal")  # 等比例坐标


def main():  # 主函数
    x_start = (18, 8)  # Starting node
    x_goal = (37, 18)  # Goal node

    rrt = RrtStarSmart(x_start, x_goal, 1.5, 0.10, 0, 1000)  # 构造规划器
    rrt.planning()  # 开始规划


if __name__ == '__main__':  # 脚本入口
    main()  # 运行主函数
