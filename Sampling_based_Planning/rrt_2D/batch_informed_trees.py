"""
Batch Informed Trees (BIT*)
@author: huiming zhou
"""

import os
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.spatial.transform import Rotation as Rot

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env, plotting, utils


class Node:
    def __init__(self, x, y):
        self.x = x  # 节点横坐标
        self.y = y  # 节点纵坐标
        self.parent = None  # 树中父节点


class Tree:
    def __init__(self, x_start, x_goal):
        self.x_start = x_start  # 起点节点
        self.goal = x_goal  # 目标节点

        self.r = 4.0  # 近邻连接半径
        self.V = set()  # 树顶点集合
        self.E = set()  # 树边集合
        self.QE = set()  # 待处理边队列
        self.QV = set()  # 待扩展顶点队列

        self.V_old = set()  # 上一批次的顶点集


class BITStar:
    def __init__(self, x_start, x_goal, eta, iter_max):
        self.x_start = Node(x_start[0], x_start[1])  # 起点节点
        self.x_goal = Node(x_goal[0], x_goal[1])  # 目标节点
        self.eta = eta  # 近邻半径系数
        self.iter_max = iter_max  # 最大迭代次数

        self.env = env.Env()  # 环境障碍信息
        self.plotting = plotting.Plotting(x_start, x_goal)  # 绘图工具
        self.utils = utils.Utils()  # 碰撞检测等工具

        self.fig, self.ax = plt.subplots()  # 创建画布与坐标轴

        self.delta = self.utils.delta  # 碰撞检测安全裕度
        self.x_range = self.env.x_range  # 采样横坐标范围
        self.y_range = self.env.y_range  # 采样纵坐标范围

        self.obs_circle = self.env.obs_circle  # 圆形障碍列表
        self.obs_rectangle = self.env.obs_rectangle  # 矩形障碍列表
        self.obs_boundary = self.env.obs_boundary  # 边界障碍列表

        self.Tree = Tree(self.x_start, self.x_goal)  # 搜索树结构
        self.X_sample = set()  # 批量采样点集合
        self.g_T = dict()  # g 值表：节点 -> 到起点代价

    def init(self):
        self.Tree.V.add(self.x_start)  # 起点加入树
        self.X_sample.add(self.x_goal)  # 目标先作为采样点

        self.g_T[self.x_start] = 0.0  # 起点代价为零
        self.g_T[self.x_goal] = np.inf  # 目标代价初始无穷

        cMin, theta = self.calc_dist_and_angle(self.x_start, self.x_goal)  # 直线距离与方向角
        C = self.RotationToWorldFrame(self.x_start, self.x_goal, cMin)  # 椭球坐标系旋转矩阵
        xCenter = np.array([[(self.x_start.x + self.x_goal.x) / 2.0],  # 椭球中心横坐标
                            [(self.x_start.y + self.x_goal.y) / 2.0], [0.0]])  # 椭球中心纵坐标

        return theta, cMin, xCenter, C  # 供椭球采样使用

    def planning(self):
        theta, cMin, xCenter, C = self.init()  # 初始化并取得椭球参数

        for k in range(500):  # 主循环：每轮一个批次
            if not self.Tree.QE and not self.Tree.QV:  # 两个队列都空则开新批次
                if k == 0:  # 首批采样数量
                    m = 350
                else:
                    m = 200

                if self.x_goal.parent is not None:  # 已有可行路径
                    path_x, path_y = self.ExtractPath()  # 取出当前最优路径
                    plt.plot(path_x, path_y, linewidth=2, color='r')
                    plt.pause(0.5)

                self.Prune(self.g_T[self.x_goal])  # 按当前最优代价剪枝
                self.X_sample.update(self.Sample(m, self.g_T[self.x_goal], cMin, xCenter, C))  # 采样
                self.Tree.V_old = {v for v in self.Tree.V}  # 记录本批前顶点集
                self.Tree.QV = {v for v in self.Tree.V}  # 顶点队列置为本批起点
                # self.Tree.r = self.radius(len(self.Tree.V) + len(self.X_sample))

            while self.BestVertexQueueValue() <= self.BestEdgeQueueValue():  # 顶点队列更优则先扩展
                self.ExpandVertex(self.BestInVertexQueue())  # 扩展最优顶点

            vm, xm = self.BestInEdgeQueue()  # 取最优候选边
            self.Tree.QE.remove((vm, xm))  # 从边队列移除

            if self.g_T[vm] + self.calc_dist(vm, xm) + self.h_estimated(xm) < self.g_T[self.x_goal]:  # 有望
                actual_cost = self.cost(vm, xm)  # 边的真实代价
                if self.g_estimated(vm) + actual_cost + self.h_estimated(xm) < self.g_T[self.x_goal]:  # 有望
                    if self.g_T[vm] + actual_cost < self.g_T[xm]:  # 可降低 xm 的代价
                        if xm in self.Tree.V:  # xm 已在树中
                            # remove edges
                            edge_delete = set()  # 待删除的旧边
                            for v, x in self.Tree.E:
                                if x == xm:  # 指向 xm 的旧边
                                    edge_delete.add((v, x))

                            for edge in edge_delete:
                                self.Tree.E.remove(edge)  # 删除旧边以更新父节点
                        else:
                            self.X_sample.remove(xm)  # 采样点转为树顶点
                            self.Tree.V.add(xm)
                            self.Tree.QV.add(xm)  # 新顶点待扩展

                        self.g_T[xm] = self.g_T[vm] + actual_cost  # 更新 xm 的 g 值
                        self.Tree.E.add((vm, xm))  # 新边入树
                        xm.parent = vm  # 重设父节点

                        set_delete = set()  # 队列中失效的候选边
                        for v, x in self.Tree.QE:
                            if x == xm and self.g_T[v] + self.calc_dist(v, xm) >= self.g_T[xm]:  # 无改进
                                set_delete.add((v, x))

                        for edge in set_delete:
                            self.Tree.QE.remove(edge)  # 从边队列剔除
            else:
                self.Tree.QE = set()  # 本批边队列清空
                self.Tree.QV = set()  # 本批顶点队列清空

            if k % 5 == 0:  # 每五轮刷新一次动画
                self.animation(xCenter, self.g_T[self.x_goal], cMin, theta)

        path_x, path_y = self.ExtractPath()  # 迭代结束取最优路径
        plt.plot(path_x, path_y, linewidth=2, color='r')
        plt.pause(0.01)
        plt.show()

    def ExtractPath(self):
        node = self.x_goal  # 从目标节点回溯
        path_x, path_y = [node.x], [node.y]  # 路径坐标序列

        while node.parent:  # 沿父指针回溯到起点
            node = node.parent
            path_x.append(node.x)
            path_y.append(node.y)

        return path_x, path_y  # 返回路径坐标

    def Prune(self, cBest):
        self.X_sample = {x for x in self.X_sample if self.f_estimated(x) < cBest}  # 剔除无望采样点
        self.Tree.V = {v for v in self.Tree.V if self.f_estimated(v) <= cBest}  # 剔除无望顶点
        self.Tree.E = {(v, w) for v, w in self.Tree.E
                       if self.f_estimated(v) <= cBest and self.f_estimated(w) <= cBest}  # 剔除无望边
        self.X_sample.update({v for v in self.Tree.V if self.g_T[v] == np.inf})  # 未连接顶点退回采样集
        self.Tree.V = {v for v in self.Tree.V if self.g_T[v] < np.inf}  # 树只留已连接顶点

    def cost(self, start, end):
        if self.utils.is_collision(start, end):  # 边与障碍相交
            return np.inf  # 不可通行

        return self.calc_dist(start, end)  # 否则代价为欧氏距离

    def f_estimated(self, node):
        return self.g_estimated(node) + self.h_estimated(node)  # 启发式 f 值

    def g_estimated(self, node):
        return self.calc_dist(self.x_start, node)  # 到起点的启发式代价

    def h_estimated(self, node):
        return self.calc_dist(node, self.x_goal)  # 到目标的启发式代价

    def Sample(self, m, cMax, cMin, xCenter, C):
        if cMax < np.inf:  # 已有路径则椭球内采样
            return self.SampleEllipsoid(m, cMax, cMin, xCenter, C)
        else:
            return self.SampleFreeSpace(m)  # 否则全空间均匀采样

    def SampleEllipsoid(self, m, cMax, cMin, xCenter, C):
        r = [cMax / 2.0,  # 椭球长半轴
             math.sqrt(cMax ** 2 - cMin ** 2) / 2.0,  # 椭球短半轴
             math.sqrt(cMax ** 2 - cMin ** 2) / 2.0]  # 椭球短半轴
        L = np.diag(r)  # 椭球缩放矩阵

        ind = 0  # 已生成样本数
        delta = self.delta  # 采样安全裕度
        Sample = set()  # 样本集合

        while ind < m:  # 直到生成 m 个有效样本
            xBall = self.SampleUnitNBall()  # 单位球内随机点
            x_rand = np.dot(np.dot(C, L), xBall) + xCenter  # 变换到椭球内
            node = Node(x_rand[(0, 0)], x_rand[(1, 0)])  # 转成节点
            in_obs = self.utils.is_inside_obs(node)  # 是否落在障碍内
            in_x_range = self.x_range[0] + delta <= node.x <= self.x_range[1] - delta  # 横坐标在范围内
            in_y_range = self.y_range[0] + delta <= node.y <= self.y_range[1] - delta  # 纵坐标在范围内

            if not in_obs and in_x_range and in_y_range:  # 样本合法
                Sample.add(node)
                ind += 1

        return Sample  # 返回椭球采样集

    def SampleFreeSpace(self, m):
        delta = self.utils.delta  # 采样安全裕度
        Sample = set()  # 样本集合

        ind = 0  # 已生成样本数
        while ind < m:  # 直到生成 m 个有效样本
            node = Node(random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                        random.uniform(self.y_range[0] + delta, self.y_range[1] - delta))  # 全空间随机点
            if self.utils.is_inside_obs(node):  # 落在障碍内则丢弃
                continue
            else:
                Sample.add(node)
                ind += 1

        return Sample  # 返回自由空间采样集

    def radius(self, q):
        cBest = self.g_T[self.x_goal]  # 当前最优代价
        lambda_X = len([1 for v in self.Tree.V if self.f_estimated(v) <= cBest])  # 有效顶点数
        radius = 2 * self.eta * (1.5 * lambda_X / math.pi * math.log(q) / q) ** 0.5  # 连接半径

        return radius  # 返回半径

    def ExpandVertex(self, v):
        self.Tree.QV.remove(v)  # 顶点出队
        X_near = {x for x in self.X_sample if self.calc_dist(x, v) <= self.Tree.r}  # 半径内采样点

        for x in X_near:  # 为采样点建立候选边
            if self.g_estimated(v) + self.calc_dist(v, x) + self.h_estimated(x) < self.g_T[self.x_goal]:
                self.g_T[x] = np.inf  # 采样点代价先设无穷
                self.Tree.QE.add((v, x))  # 候选边入队

        if v not in self.Tree.V_old:  # 新顶点才需与旧顶点连边
            V_near = {w for w in self.Tree.V if self.calc_dist(w, v) <= self.Tree.r}  # 半径内树顶点

            for w in V_near:
                if (v, w) not in self.Tree.E and \
                        self.g_estimated(v) + self.calc_dist(v, w) + self.h_estimated(w) < self.g_T[self.x_goal] and \
                        self.g_T[v] + self.calc_dist(v, w) < self.g_T[w]:  # 可降 w 的 g 值
                    self.Tree.QE.add((v, w))  # 候选边入队
                    if w not in self.g_T:  # 尚未记录代价
                        self.g_T[w] = np.inf

    def BestVertexQueueValue(self):
        if not self.Tree.QV:  # 顶点队列为空
            return np.inf  # 视为无穷大

        return min(self.g_T[v] + self.h_estimated(v) for v in self.Tree.QV)  # 队列最小 f 值

    def BestEdgeQueueValue(self):
        if not self.Tree.QE:  # 边队列为空
            return np.inf  # 视为无穷大

        return min(self.g_T[v] + self.calc_dist(v, x) + self.h_estimated(x)
                   for v, x in self.Tree.QE)  # 边队列最小 g+c+h

    def BestInVertexQueue(self):
        if not self.Tree.QV:  # 顶点队列为空
            print("QV is Empty!")
            return None

        v_value = {v: self.g_T[v] + self.h_estimated(v) for v in self.Tree.QV}  # 各顶点 f 值

        return min(v_value, key=v_value.get)  # 返回 f 最小的顶点

    def BestInEdgeQueue(self):
        if not self.Tree.QE:  # 边队列为空
            print("QE is Empty!")
            return None

        e_value = {(v, x): self.g_T[v] + self.calc_dist(v, x) + self.h_estimated(x)
                   for v, x in self.Tree.QE}  # 各候选边的 g+c+h

        return min(e_value, key=e_value.get)  # 返回 g+c+h 最小的边

    @staticmethod
    def SampleUnitNBall():
        while True:  # 拒绝采样直到落入单位球
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)  # 单位正方形内随机点
            if x ** 2 + y ** 2 < 1:  # 落在单位圆内
                return np.array([[x], [y], [0.0]])  # 返回齐次球内点

    @staticmethod
    def RotationToWorldFrame(x_start, x_goal, L):
        a1 = np.array([[(x_goal.x - x_start.x) / L],
                       [(x_goal.y - x_start.y) / L], [0.0]])  # 起点到目标的单位向量
        e1 = np.array([[1.0], [0.0], [0.0]])  # 参考轴单位向量
        M = a1 @ e1.T  # 待分解的旋转矩阵
        U, _, V_T = np.linalg.svd(M, True, True)  # 奇异值分解
        C = U @ np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(V_T.T)]) @ V_T  # 修正为纯旋转

        return C  # 返回世界坐标旋转矩阵

    @staticmethod
    def calc_dist(start, end):
        return math.hypot(start.x - end.x, start.y - end.y)  # 两点欧氏距离

    @staticmethod
    def calc_dist_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x  # 横坐标增量
        dy = node_end.y - node_start.y  # 纵坐标增量
        return math.hypot(dx, dy), math.atan2(dy, dx)  # 距离与方向角

    def animation(self, xCenter, cMax, cMin, theta):
        plt.cla()  # 清空画布
        self.plot_grid("Batch Informed Trees (BIT*)")  # 重绘环境

        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])  # 按 Esc 键退出

        for v in self.X_sample:  # 画所有采样点
            plt.plot(v.x, v.y, marker='.', color='lightgrey', markersize='2')

        if cMax < np.inf:  # 已有路径则画 informed 椭球
            self.draw_ellipse(xCenter, cMax, cMin, theta)

        for v, w in self.Tree.E:  # 画树中所有边
            plt.plot([v.x, w.x], [v.y, w.y], '-g')

        plt.pause(0.001)  # 短暂暂停刷新

    def plot_grid(self, name):
        for (ox, oy, w, h) in self.obs_boundary:  # 画边界矩形
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:  # 画矩形障碍
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.obs_circle:  # 画圆形障碍
            self.ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.plot(self.x_start.x, self.x_start.y, "bs", linewidth=3)  # 标出起点
        plt.plot(self.x_goal.x, self.x_goal.y, "rs", linewidth=3)  # 标出目标点

        plt.title(name)  # 设置图标题
        plt.axis("equal")  # 等比例坐标轴

    @staticmethod
    def draw_ellipse(x_center, c_best, dist, theta):
        a = math.sqrt(c_best ** 2 - dist ** 2) / 2.0  # 椭球短半轴
        b = c_best / 2.0  # 椭球长半轴
        angle = math.pi / 2.0 - theta  # 椭圆参数角
        cx = x_center[0]  # 中心横坐标
        cy = x_center[1]  # 中心纵坐标
        t = np.arange(0, 2 * math.pi + 0.1, 0.2)  # 参数角序列
        x = [a * math.cos(it) for it in t]  # 椭圆横坐标
        y = [b * math.sin(it) for it in t]  # 椭圆纵坐标
        rot = Rot.from_euler('z', -angle).as_matrix()[0:2, 0:2]  # 绕 z 轴旋转
        fx = rot @ np.array([x, y])  # 旋转到世界坐标
        px = np.array(fx[0, :] + cx).flatten()  # 平移后横坐标
        py = np.array(fx[1, :] + cy).flatten()  # 平移后纵坐标
        plt.plot(cx, cy, marker='.', color='darkorange')  # 标出椭球中心
        plt.plot(px, py, linestyle='--', color='darkorange', linewidth=2)  # 画出 informed 椭球


def main():
    x_start = (18, 8)  # Starting node
    x_goal = (37, 18)  # Goal node
    eta = 2  # 近邻半径系数
    iter_max = 200  # 最大迭代次数
    print("start!!!")
    bit = BITStar(x_start, x_goal, eta, iter_max)  # 构造 BIT* 求解器
    # bit.animation("Batch Informed Trees (BIT*)")
    bit.planning()  # 开始批量搜索


if __name__ == '__main__':
    main()
