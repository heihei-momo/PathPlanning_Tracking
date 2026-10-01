"""
INFORMED_RRT_STAR 2D
@author: huiming zhou
"""

import os  # 操作系统路径接口
import sys  # 系统模块搜索路径
import math  # 数学函数库
import random  # 随机采样
import numpy as np  # 数值计算
import matplotlib.pyplot as plt  # 绘图
from scipy.spatial.transform import Rotation as Rot  # 旋转矩阵构造
import matplotlib.patches as patches  # 障碍物图元

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")  # 追加仓库根目录到搜索路径

from Sampling_based_Planning.rrt_2D import env, plotting, utils  # 环境/绘图/工具模块


class Node:  # 树节点
    def __init__(self, n):  # 由坐标构造节点
        self.x = n[0]  # 节点 x 坐标
        self.y = n[1]  # 节点 y 坐标
        self.parent = None  # 父节点，根为 None


class IRrtStar:  # Informed RRT* 规划器
    def __init__(self, x_start, x_goal, step_len,  # 起点、终点、步长
                 goal_sample_rate, search_radius, iter_max):  # 目标偏置率、近邻半径、迭代上限
        self.x_start = Node(x_start)  # 起点节点
        self.x_goal = Node(x_goal)  # 终点节点
        self.step_len = step_len  # 每次扩展的步长
        self.goal_sample_rate = goal_sample_rate  # 采样到终点的概率
        self.search_radius = search_radius  # 近邻搜索半径系数
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
        self.X_soln = set()  # 已到达终点的解节点
        self.path = None  # 最终路径

    def init(self):  # 初始化椭球参数
        cMin, theta = self.get_distance_and_angle(self.x_start, self.x_goal)  # 起终点直线距离与方向角
        C = self.RotationToWorldFrame(self.x_start, self.x_goal, cMin)  # 世界系到椭球系的旋转矩阵
        xCenter = np.array([[(self.x_start.x + self.x_goal.x) / 2.0],  # 椭球中心 x 坐标
                            [(self.x_start.y + self.x_goal.y) / 2.0], [0.0]])  # 椭球中心 y 与占位 z
        x_best = self.x_start  # 当前最优解节点

        return theta, cMin, xCenter, C, x_best  # 返回椭球参数与最优节点

    def planning(self):  # 主循环
        theta, dist, x_center, C, x_best = self.init()  # 初始化椭球参数
        c_best = np.inf  # 当前最优路径代价，初始为无穷

        for k in range(self.iter_max):  # 迭代扩展
            if self.X_soln:  # 已存在可行解时
                cost = {node: self.Cost(node) for node in self.X_soln}  # 各解节点到起点的代价,给 X_soln 里的每一个候选解节点，计算它从起点走到这个节点的累计路径代价
                x_best = min(cost, key=cost.get)  # 取代价最小的解节点
                c_best = cost[x_best]  # 更新最优代价，用于椭球采样

            x_rand = self.Sample(c_best, dist, x_center, C)  # 椭球子集采样
            x_nearest = self.Nearest(self.V, x_rand)  # 最近邻节点
            x_new = self.Steer(x_nearest, x_rand)  # 向采样点扩展新节点

            if x_new and not self.utils.is_collision(x_nearest, x_new):  # 新边无碰撞
                X_near = self.Near(self.V, x_new)  # 新节点的近邻集合
                c_min = self.Cost(x_nearest) + self.Line(x_nearest, x_new)  # 初始候选代价
                self.V.append(x_new)  # 新节点加入树

                # choose parent
                for x_near in X_near:  # 遍历近邻挑选更优父节点
                    c_new = self.Cost(x_near) + self.Line(x_near, x_new)  # 经该近邻到达的代价
                    if c_new < c_min:  # 代价更小则改父
                        x_new.parent = x_near  # 更新父节点
                        c_min = c_new  # 更新最小代价

                # rewire
                for x_near in X_near:  # 遍历近邻尝试重连
                    c_near = self.Cost(x_near)  # 近邻原代价
                    c_new = self.Cost(x_new) + self.Line(x_new, x_near)  # 经新节点到达的代价
                    if c_new < c_near:  # 经新节点更省
                        x_near.parent = x_new  # 重连父节点

                if self.InGoalRegion(x_new):  # 新节点进入终点区域
                    if not self.utils.is_collision(x_new, self.x_goal):  # 到终点无碰撞
                        self.X_soln.add(x_new)  # 记录为可行解
                        # new_cost = self.Cost(x_new) + self.Line(x_new, self.x_goal)
                        # if new_cost < c_best:
                        #     c_best = new_cost
                        #     x_best = x_new

            if k % 20 == 0:  # 每 20 次迭代刷新一次
                self.animation(x_center=x_center, c_best=c_best, dist=dist, theta=theta)  # 刷新树与椭球

        self.path = self.ExtractPath(x_best)  # 回溯最优解路径
        self.animation(x_center=x_center, c_best=c_best, dist=dist, theta=theta)  # 最终绘制
        plt.plot([x for x, _ in self.path], [y for _, y in self.path], '-r')  # 红色画出路径
        plt.pause(0.01)  # 短暂停顿刷新画面
        plt.show()  # 保持窗口显示

    def Steer(self, x_start, x_goal):  # 朝目标方向扩展一步
        dist, theta = self.get_distance_and_angle(x_start, x_goal)  # 距离与方向
        dist = min(self.step_len, dist)  # 步长截断
        node_new = Node((x_start.x + dist * math.cos(theta),  # 新节点 x 坐标
                         x_start.y + dist * math.sin(theta)))  # 新节点 y 坐标
        node_new.parent = x_start  # 暂定父节点

        return node_new  # 返回新节点

    def Near(self, nodelist, node):  # 近邻搜索
        n = len(nodelist) + 1  # 当前节点数
        r = 50 * math.sqrt((math.log(n) / n))  # 随节点数衰减的邻域半径

        dist_table = [(nd.x - node.x) ** 2 + (nd.y - node.y) ** 2 for nd in nodelist]  # 距离平方表
        X_near = [nodelist[ind] for ind in range(len(dist_table)) if dist_table[ind] <= r ** 2 and  # 半径内
                  not self.utils.is_collision(nodelist[ind], node)]  # 且无碰撞

        return X_near  # 近邻节点列表

    def Sample(self, c_max, c_min, x_center, C):  # 椭球子集采样
        if c_max < np.inf:  # 已有解时用椭球约束
            r = [c_max / 2.0,  # 椭球长半轴
                 math.sqrt(c_max ** 2 - c_min ** 2) / 2.0,  # 短半轴之一
                 math.sqrt(c_max ** 2 - c_min ** 2) / 2.0]  # 短半轴之一
            L = np.diag(r)  # 椭球缩放对角阵

            while True:  # 拒绝采样直到落入地图内
                x_ball = self.SampleUnitBall()  # 单位球内均匀采样
                x_rand = np.dot(np.dot(C, L), x_ball) + x_center  # 变换到世界系椭球内
                if self.x_range[0] + self.delta <= x_rand[0] <= self.x_range[1] - self.delta and \
                        self.y_range[0] + self.delta <= x_rand[1] <= self.y_range[1] - self.delta:  # y 界内
                    break  # 接受该采样点
            x_rand = Node((x_rand[(0, 0)], x_rand[(1, 0)]))  # 转为节点对象
        else:  # 尚无解时全局采样
            x_rand = self.SampleFreeSpace()  # 自由空间采样

        return x_rand  # 返回采样节点

    @staticmethod  # 静态方法
    def SampleUnitBall():  # 单位圆内均匀采样
        while True:  # 拒绝采样
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)  # 正方形内随机点
            if x ** 2 + y ** 2 < 1:  # 落在单位圆内
                return np.array([[x], [y], [0.0]])  # 返回三维齐次坐标

    def SampleFreeSpace(self):  # 全局采样
        delta = self.delta  # 安全边距

        if np.random.random() > self.goal_sample_rate:  # 非目标偏置
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),  # 随机 x
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))  # 随机 y

        return self.x_goal  # 直接返回终点

    def ExtractPath(self, node):  # 回溯路径
        path = [[self.x_goal.x, self.x_goal.y]]  # 从终点开始

        while node.parent:  # 沿父指针上溯
            path.append([node.x, node.y])  # 记录当前节点
            node = node.parent  # 移动到父节点

        path.append([self.x_start.x, self.x_start.y])  # 补上起点

        return path  # 返回路径点列

    def InGoalRegion(self, node):  # 是否进入终点区域
        if self.Line(node, self.x_goal) < self.step_len:  # 距终点小于一步
            return True  # 视为到达

        return False  # 未到达

    @staticmethod  # 静态方法
    def RotationToWorldFrame(x_start, x_goal, L):  # 构造旋转到世界系的矩阵
        a1 = np.array([[(x_goal.x - x_start.x) / L],  # 起终点单位方向 x
                       [(x_goal.y - x_start.y) / L], [0.0]])  # 单位方向 y 与占位 z
        e1 = np.array([[1.0], [0.0], [0.0]])  # 椭球坐标系长轴基向量
        M = a1 @ e1.T  # 两方向的投影矩阵
        U, _, V_T = np.linalg.svd(M, True, True)  # 奇异值分解求旋转
        C = U @ np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(V_T.T)]) @ V_T  # 右手系旋转矩阵

        return C  # 返回旋转矩阵

    @staticmethod  # 静态方法
    def Nearest(nodelist, n):  # 最近邻节点
        return nodelist[int(np.argmin([(nd.x - n.x) ** 2 + (nd.y - n.y) ** 2  # 距离平方最小
                                       for nd in nodelist]))]  # 遍历所有节点

    @staticmethod  # 静态方法
    def Line(x_start, x_goal):  # 两点欧氏距离
        return math.hypot(x_goal.x - x_start.x, x_goal.y - x_start.y)  # 直线代价

    def Cost(self, node):  # 计算到起点的累计代价
        if node == self.x_start:  # 起点本身
            return 0.0  # 代价为零

        if node.parent is None:  # 孤立节点
            return np.inf  # 不可达

        cost = 0.0  # 累计代价
        while node.parent:  # 沿父指针累加
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)  # 累加边长
            node = node.parent  # 上溯

        return cost  # 返回总代价

    @staticmethod  # 静态方法
    def get_distance_and_angle(node_start, node_end):  # 计算距离与方位角
        dx = node_end.x - node_start.x  # 横向差
        dy = node_end.y - node_start.y  # 纵向差
        return math.hypot(dx, dy), math.atan2(dy, dx)  # 距离与夹角

    def animation(self, x_center=None, c_best=None, dist=None, theta=None):  # 动态绘制
        plt.cla()  # 清空画布
        self.plot_grid("Informed rrt*, N = " + str(self.iter_max))  # 绘制栅格与障碍
        plt.gcf().canvas.mpl_connect(  # 绑定按键
            'key_release_event',  # 按键释放事件
            lambda event: [exit(0) if event.key == 'escape' else None])  # ESC 退出

        for node in self.V:  # 遍历树
            if node.parent:  # 非根节点
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")  # 绿色画树边

        if c_best != np.inf:  # 已有解
            self.draw_ellipse(x_center, c_best, dist, theta)  # 画 Informed 椭球

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

    @staticmethod  # 静态方法
    def draw_ellipse(x_center, c_best, dist, theta):  # 绘制 Informed 椭圆
        a = math.sqrt(c_best ** 2 - dist ** 2) / 2.0  # 椭球短半轴
        b = c_best / 2.0  # 椭球长半轴（沿起终点方向）
        angle = math.pi / 2.0 - theta  # 椭圆旋转角
        cx = x_center[0]  # 中心 x 坐标
        cy = x_center[1]  # 中心 y 坐标
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)  # 参数角序列
        x = [a * math.cos(it) for it in t]  # 椭圆点 x
        y = [b * math.sin(it) for it in t]  # 椭圆点 y
        rot = Rot.from_euler('z', -angle).as_matrix()[0:2, 0:2]  # 二维旋转矩阵
        fx = rot @ np.array([x, y])  # 旋转到世界系
        px = np.array(fx[0, :] + cx).flatten()  # 平移后 x 坐标
        py = np.array(fx[1, :] + cy).flatten()  # 平移后 y 坐标
        plt.plot(cx, cy, ".b")  # 标记椭圆中心
        plt.plot(px, py, linestyle='--', color='darkorange', linewidth=2)  # 橙色虚线画椭圆


def main():  # 主函数
    x_start = (18, 8)  # Starting node
    x_goal = (37, 18)  # Goal node

    rrt_star = IRrtStar(x_start, x_goal, 1, 0.10, 12, 1000)  # 构造规划器
    rrt_star.planning()  # 开始规划


if __name__ == '__main__':  # 脚本入口
    main()  # 运行主函数
