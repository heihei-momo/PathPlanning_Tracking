"""
DYNAMIC_RRT_2D
@author: huiming zhou
"""

import os
import sys
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env, plotting, utils


class Node:
    def __init__(self, n):
        self.x = n[0]  # 节点横坐标
        self.y = n[1]  # 节点纵坐标
        self.parent = None  # 树中的父节点
        self.flag = "VALID"  # 节点有效标志


class Edge:
    def __init__(self, n_p, n_c):
        self.parent = n_p  # 边的起点节点
        self.child = n_c  # 边的终点节点
        self.flag = "VALID"  # 边有效标志


class DynamicRrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, waypoint_sample_rate, iter_max):
        self.s_start = Node(s_start)  # 起点节点
        self.s_goal = Node(s_goal)  # 目标节点
        self.step_len = step_len  # 单次扩展步长
        self.goal_sample_rate = goal_sample_rate  # 目标偏置采样概率
        self.waypoint_sample_rate = waypoint_sample_rate  # 路标点采样概率
        self.iter_max = iter_max  # 最大迭代次数
        self.vertex = [self.s_start]  # 树中全部节点
        self.vertex_old = []  # 变化前保留的节点
        self.vertex_new = []  # 重规划新增节点
        self.edges = []  # 树中全部边

        self.env = env.Env()  # 环境障碍信息
        self.plotting = plotting.Plotting(s_start, s_goal)  # 绘图工具
        self.utils = utils.Utils()  # 碰撞检测等工具
        self.fig, self.ax = plt.subplots()  # 创建画布与坐标轴

        self.x_range = self.env.x_range  # 采样横坐标范围
        self.y_range = self.env.y_range  # 采样纵坐标范围
        self.obs_circle = self.env.obs_circle  # 圆形障碍列表
        self.obs_rectangle = self.env.obs_rectangle  # 矩形障碍列表
        self.obs_boundary = self.env.obs_boundary  # 边界障碍列表
        self.obs_add = [0, 0, 0]  # 新加圆形障碍参数

        self.path = []  # 当前路径点序列
        self.waypoint = []  # 当前路径路标点

    def planning(self):
        for i in range(self.iter_max):  # 迭代生长搜索树
            node_rand = self.generate_random_node(self.goal_sample_rate)  # 采样随机点
            node_near = self.nearest_neighbor(self.vertex, node_rand)  # 找最近邻节点
            node_new = self.new_state(node_near, node_rand)  # 按步长扩展新节点

            if node_new and not self.utils.is_collision(node_near, node_new):  # 新边无碰撞
                self.vertex.append(node_new)  # 新节点加入树
                self.edges.append(Edge(node_near, node_new))  # 记录新边
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)  # 到目标距离

                if dist <= self.step_len:  # 已可一步到达目标
                    self.new_state(node_new, self.s_goal)  # 扩展出目标节点

                    path = self.extract_path(node_new)  # 回溯得到路径
                    self.plot_grid("Dynamic_RRT")  # 绘制环境
                    self.plot_visited()  # 绘制搜索树
                    self.plot_path(path)  # 绘制路径
                    self.path = path  # 保存路径
                    self.waypoint = self.extract_waypoint(node_new)  # 保存路标点
                    self.fig.canvas.mpl_connect('button_press_event', self.on_press)  # 绑定点击
                    plt.show()  # 显示并等待交互

                    return  # 首次规划结束

        return None  # 未找到路径

    def on_press(self, event):
        x, y = event.xdata, event.ydata  # 鼠标点击坐标
        if x < 0 or x > 50 or y < 0 or y > 30:  # 超出地图范围
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)  # 取整作为圆心
            print("Add circle obstacle at: s =", x, ",", "y =", y)
            self.obs_add = [x, y, 2]  # 记录新障碍圆心与半径
            self.obs_circle.append([x, y, 2])  # 加入圆形障碍列表
            self.utils.update_obs(self.obs_circle, self.obs_boundary, self.obs_rectangle)  # 更新障碍
            self.InvalidateNodes()  # 标记被新障碍破坏的节点

            if self.is_path_invalid():  # 当前路径已不可行
                print("Path is Replanning ...")
                path, waypoint = self.replanning()  # 重新生长搜索树

                print("len_vertex: ", len(self.vertex))
                print("len_vertex_old: ", len(self.vertex_old))
                print("len_vertex_new: ", len(self.vertex_new))

                plt.cla()  # 清空画布
                self.plot_grid("Dynamic_RRT")  # 重绘环境
                self.plot_vertex_old()  # 画保留的旧树
                self.plot_path(self.path, color='blue')  # 画原路径
                self.plot_vertex_new()  # 画新增树枝
                self.vertex_new = []  # 清空新增节点记录
                self.plot_path(path)  # 画新路径
                self.path = path  # 更新路径
                self.waypoint = waypoint  # 更新路标点
            else:  # 路径仍可行只需修剪
                print("Trimming Invalid Nodes ...")
                self.TrimRRT()  # 剔除失效节点

                plt.cla()  # 清空画布
                self.plot_grid("Dynamic_RRT")  # 重绘环境
                self.plot_visited(animation=False)  # 静态画树
                self.plot_path(self.path)  # 画当前路径

            self.fig.canvas.draw_idle()  # 刷新显示

    def InvalidateNodes(self):
        for edge in self.edges:  # 遍历树中每条边
            if self.is_collision_obs_add(edge.parent, edge.child):  # 边被新障碍切断
                edge.child.flag = "INVALID"  # 标记子节点失效

    def is_path_invalid(self):
        for node in self.waypoint:  # 检查路径上的路标点
            if node.flag == "INVALID":  # 存在失效点
                return True  # 路径不可行

    def is_collision_obs_add(self, start, end):
        delta = self.utils.delta  # 碰撞检测安全裕度
        obs_add = self.obs_add  # 新加障碍参数

        if math.hypot(start.x - obs_add[0], start.y - obs_add[1]) <= obs_add[2] + delta:  # 起点在障碍内
            return True

        if math.hypot(end.x - obs_add[0], end.y - obs_add[1]) <= obs_add[2] + delta:  # 终点在障碍内
            return True

        o, d = self.utils.get_ray(start, end)  # 线段的射线表示
        if self.utils.is_intersect_circle(o, d, [obs_add[0], obs_add[1]], obs_add[2]):  # 与圆相交
            return True

        return False  # 未发生碰撞

    def replanning(self):
        self.TrimRRT()  # 先剔除失效节点

        for i in range(self.iter_max):  # 迭代重生长搜索树
            node_rand = self.generate_random_node_replanning(self.goal_sample_rate, self.waypoint_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)  # 找最近邻节点
            node_new = self.new_state(node_near, node_rand)  # 按步长扩展新节点

            if node_new and not self.utils.is_collision(node_near, node_new):  # 新边无碰撞
                self.vertex.append(node_new)  # 新节点加入树
                self.vertex_new.append(node_new)  # 记为新增节点
                self.edges.append(Edge(node_near, node_new))  # 记录新边
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)  # 到目标距离

                if dist <= self.step_len:  # 已可一步到达目标
                    self.new_state(node_new, self.s_goal)  # 扩展出目标节点
                    path = self.extract_path(node_new)  # 回溯得到路径
                    waypoint = self.extract_waypoint(node_new)  # 回溯得到路标点
                    print("path: ", len(path))
                    print("waypoint: ", len(waypoint))

                    return path, waypoint  # 返回重规划结果

        return None  # 重规划失败

    def TrimRRT(self):
        for i in range(1, len(self.vertex)):  # 跳过起点遍历其余节点
            node = self.vertex[i]
            node_p = node.parent  # 当前节点的父节点
            if node_p.flag == "INVALID":  # 父节点失效
                node.flag = "INVALID"  # 子节点随之失效

        self.vertex = [node for node in self.vertex if node.flag == "VALID"]  # 仅留有效节点
        self.vertex_old = copy.deepcopy(self.vertex)  # 备份修剪后节点
        self.edges = [Edge(node.parent, node) for node in self.vertex[1:len(self.vertex)]]  # 重建边集

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta  # 采样安全裕度

        if np.random.random() > goal_sample_rate:  # 普通随机采样
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),  # 横坐标采样
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))  # 纵坐标采样

        return self.s_goal  # 目标偏置采样

    def generate_random_node_replanning(self, goal_sample_rate, waypoint_sample_rate):
        delta = self.utils.delta  # 采样安全裕度
        p = np.random.random()  # 采样概率

        if p < goal_sample_rate:  # 目标偏置采样
            return self.s_goal
        elif goal_sample_rate < p < goal_sample_rate + waypoint_sample_rate:  # 路标点偏置采样
            return self.waypoint[np.random.randint(0, len(self.waypoint) - 1)]  # 随机取一路标点
        else:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),  # 横坐标采样
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))  # 纵坐标采样

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)  # 逐个计算到采样点距离
                                        for nd in node_list]))]  # 取距离最小的节点

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)  # 距离与方向角

        dist = min(self.step_len, dist)  # 扩展不超过步长
        node_new = Node((node_start.x + dist * math.cos(theta),  # 沿方向角求横坐标
                         node_start.y + dist * math.sin(theta)))  # 沿方向角求纵坐标
        node_new.parent = node_start  # 记录父节点

        return node_new  # 返回扩展出的节点

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]  # 路径从目标点开始
        node_now = node_end  # 回溯游标

        while node_now.parent is not None:  # 沿父指针回溯到起点
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))  # 记录路径点

        return path  # 返回坐标序列

    def extract_waypoint(self, node_end):
        waypoint = [self.s_goal]  # 路标点从目标开始
        node_now = node_end  # 回溯游标

        while node_now.parent is not None:  # 沿父指针回溯到起点
            node_now = node_now.parent
            waypoint.append(node_now)  # 记录路标节点

        return waypoint  # 返回路标节点列表

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x  # 横坐标增量
        dy = node_end.y - node_start.y  # 纵坐标增量
        return math.hypot(dx, dy), math.atan2(dy, dx)  # 距离与方向角

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

        plt.plot(self.s_start.x, self.s_start.y, "bs", linewidth=3)  # 标出起点
        plt.plot(self.s_goal.x, self.s_goal.y, "gs", linewidth=3)  # 标出目标点

        plt.title(name)  # 设置图标题
        plt.axis("equal")  # 等比例坐标轴

    def plot_visited(self, animation=True):
        if animation:  # 逐条动画绘制
            count = 0  # 已绘制边计数
            for node in self.vertex:  # 遍历树中节点
                count += 1
                if node.parent:  # 有父节点的画出父子连线
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:  # 每十条暂停一下
                        plt.pause(0.001)
        else:  # 一次性静态绘制
            for node in self.vertex:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    def plot_vertex_old(self):
        for node in self.vertex_old:  # 遍历保留的旧节点
            if node.parent:
                plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    def plot_vertex_new(self):
        count = 0  # 已绘制边计数

        for node in self.vertex_new:  # 遍历重规划新增节点
            count += 1
            if node.parent:
                plt.plot([node.parent.x, node.x], [node.parent.y, node.y], color='darkorange')
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event:
                                             [exit(0) if event.key == 'escape' else None])
                if count % 10 == 0:  # 每十条暂停一下
                    plt.pause(0.001)

    @staticmethod
    def plot_path(path, color='red'):
        plt.plot([x[0] for x in path], [x[1] for x in path], linewidth=2, color=color)  # 连线成路径
        plt.pause(0.01)  # 短暂暂停刷新


def main():
    x_start = (2, 2)  # Starting node
    x_goal = (49, 24)  # Goal node

    drrt = DynamicRrt(x_start, x_goal, 0.5, 0.1, 0.6, 5000)  # 步长 0.5，目标偏置 0.1
    drrt.planning()  # 开始首次规划


if __name__ == '__main__':
    main()
