"""
EXTENDED_RRT_2D
@author: huiming zhou
"""

import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env, plotting, utils


class Node:  # 树节点：坐标与父指针
    def __init__(self, n):  # 由坐标元组构造节点
        self.x = n[0]  # 节点 x 坐标
        self.y = n[1]  # 节点 y 坐标
        self.parent = None  # 父节点指针：回溯路径


class ExtendedRrt:  # Extended-RRT 规划器
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, waypoint_sample_rate, iter_max):
        self.s_start = Node(s_start)  # 起点节点
        self.s_goal = Node(s_goal)  # 目标节点
        self.step_len = step_len  # 单步扩展长度
        self.goal_sample_rate = goal_sample_rate  # 目标偏置采样概率
        self.waypoint_sample_rate = waypoint_sample_rate  # 路点偏置采样概率
        self.iter_max = iter_max  # 最大迭代次数
        self.vertex = [self.s_start]  # 树节点集合，初始仅起点

        self.env = env.Env()  # 加载环境地图
        self.plotting = plotting.Plotting(s_start, s_goal)  # 绘图工具实例
        self.utils = utils.Utils()  # 碰撞检测等工具
        self.fig, self.ax = plt.subplots()  # 创建画布与坐标轴

        self.x_range = self.env.x_range  # x 方向采样范围
        self.y_range = self.env.y_range  # y 方向采样范围
        self.obs_circle = self.env.obs_circle  # 圆形障碍列表
        self.obs_rectangle = self.env.obs_rectangle  # 矩形障碍列表
        self.obs_boundary = self.env.obs_boundary  # 地图边界障碍

        self.path = []  # 当前规划路径
        self.waypoint = []  # 路径路点，重规划时复用

    def planning(self):  # 主循环：扩展直到接近目标
        for i in range(self.iter_max):  # 迭代扩展次数
            node_rand = self.generate_random_node(self.goal_sample_rate)  # 采样点：随机或目标偏置
            node_near = self.nearest_neighbor(self.vertex, node_rand)  # 树中距采样点最近节点
            node_new = self.new_state(node_near, node_rand)  # 朝采样点扩展一步

            if node_new and not self.utils.is_collision(node_near, node_new):  # 新边无碰撞才纳入树
                self.vertex.append(node_new)  # 新节点加入搜索树
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)  # 新节点到目标的距离

                if dist <= self.step_len:  # 已可一步到达目标
                    self.new_state(node_new, self.s_goal)  # 朝目标再扩展一步并接边

                    path = self.extract_path(node_new)  # 沿 parent 链回溯路径
                    self.plot_grid("Extended_RRT")  # 绘制地图与障碍
                    self.plot_visited()  # 绘制搜索树扩展过程
                    self.plot_path(path)  # 绘制路径折线
                    self.path = path  # 保存当前路径
                    self.waypoint = self.extract_waypoint(node_new)  # 提取路点供重规划偏置
                    self.fig.canvas.mpl_connect('button_press_event', self.on_press)  # 绑定鼠标点击事件回调
                    plt.show()  # 显示交互窗口

                    return  # 规划成功，直接返回

        return None  # 达到迭代上限仍未成功

    def on_press(self, event):  # 点击回调：加障碍并重规划
        x, y = event.xdata, event.ydata  # 获取点击位置坐标
        if x < 0 or x > 50 or y < 0 or y > 30:  # 点击超出地图范围
            print("Please choose right area!")  # 提示点击越界
        else:
            x, y = int(x), int(y)  # 坐标取整
            print("Add circle obstacle at: s =", x, ",", "y =", y)  # 打印新增障碍的位置
            self.obs_circle.append([x, y, 2])  # 添加半径 2 的圆形障碍
            self.utils.update_obs(self.obs_circle, self.obs_boundary, self.obs_rectangle)  # 刷新碰撞检测障碍
            path, waypoint = self.replanning()  # 触发重规划

            plt.cla()  # 清空画布
            self.plot_grid("Extended_RRT")  # 重绘地图与障碍
            self.plot_path(self.path, color='blue')  # 蓝色绘制原路径
            self.plot_visited()  # 重绘搜索树
            self.plot_path(path)  # 红色绘制新路径
            self.path = path  # 更新当前路径
            self.waypoint = waypoint  # 更新路点
            self.fig.canvas.draw_idle()  # 刷新画布显示

    def replanning(self):  # 环境变化后重新规划
        self.vertex = [self.s_start]  # 搜索树重置为仅含起点

        for i in range(self.iter_max):  # 迭代扩展次数
            node_rand = self.generate_random_node_replanning(self.goal_sample_rate, self.waypoint_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)  # 树中距采样点最近节点
            node_new = self.new_state(node_near, node_rand)  # 朝采样点扩展一步

            if node_new and not self.utils.is_collision(node_near, node_new):  # 新边无碰撞才纳入树
                self.vertex.append(node_new)  # 新节点加入搜索树
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)  # 新节点到目标的距离

                if dist <= self.step_len:  # 已可一步到达目标
                    self.new_state(node_new, self.s_goal)  # 朝目标扩展并接边
                    path = self.extract_path(node_new)  # 沿 parent 链回溯路径
                    waypoint = self.extract_waypoint(node_new)  # 提取路点

                    return path, waypoint  # 返回新路径与路点

        return None  # 重规划失败

    def generate_random_node(self, goal_sample_rate):  # 采样：按概率偏向目标
        delta = self.utils.delta  # 采样时保留的边界余量

        if np.random.random() > goal_sample_rate:  # 未命中目标偏置分支
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))  # 自由空间采样

        return self.s_goal  # 直接返回目标点

    def generate_random_node_replanning(self, goal_sample_rate, waypoint_sample_rate):  # 重规划采样
        delta = self.utils.delta  # 采样时保留的边界余量
        p = np.random.random()  # 取一个随机概率值

        if p < goal_sample_rate:  # 低于阈值则取目标点
            return self.s_goal  # 目标偏置采样
        elif goal_sample_rate < p < goal_sample_rate + waypoint_sample_rate:  # 落在路点采样区间
            return self.waypoint[np.random.randint(0, len(self.path) - 1)]  # 随机选取旧路径路点
        else:  # 其余情况
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))  # 自由空间采样


    @staticmethod
    def nearest_neighbor(node_list, n):  # 返回树中距 n 最近的节点
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]  # 按欧氏距离取最小者

    def new_state(self, node_start, node_end):  # 从起点朝终点扩展一步
        dist, theta = self.get_distance_and_angle(node_start, node_end)  # 计算两点距离与方位角

        dist = min(self.step_len, dist)  # 步长截断：不越过目标
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))  # 沿射线前进得到新节点
        node_new.parent = node_start  # 记录父节点以形成边

        return node_new  # 返回扩展出的新节点

    def extract_path(self, node_end):  # 回溯路径点序列
        path = [(self.s_goal.x, self.s_goal.y)]  # 路径以目标点开头
        node_now = node_end  # 从目标父节点开始回溯

        while node_now.parent is not None:  # 回溯到起点为止
            node_now = node_now.parent  # 上移到父节点
            path.append((node_now.x, node_now.y))  # 记录路径点

        return path  # 返回路径点列

    def extract_waypoint(self, node_end):  # 提取路径上的节点对象
        waypoint = [self.s_goal]  # 路点列表含目标节点
        node_now = node_end  # 从目标父节点开始回溯

        while node_now.parent is not None:  # 回溯到起点为止
            node_now = node_now.parent  # 上移到父节点
            waypoint.append(node_now)  # 收集该节点作为路点

        return waypoint  # 返回路点列表

    @staticmethod
    def get_distance_and_angle(node_start, node_end):  # 计算两点距离与方位角
        dx = node_end.x - node_start.x  # x 方向增量
        dy = node_end.y - node_start.y  # y 方向增量
        return math.hypot(dx, dy), math.atan2(dy, dx)  # 返回欧氏距离与方向角

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

        plt.plot(self.s_start.x, self.s_start.y, "bs", linewidth=3)  # 蓝色方块标记起点
        plt.plot(self.s_goal.x, self.s_goal.y, "gs", linewidth=3)  # 绿色方块标记目标

        plt.title(name)  # 设置图标题
        plt.axis("equal")  # 等比例坐标轴

    def plot_visited(self):  # 绘制搜索树扩展过程
        animation = True  # 是否启用动画
        if animation:  # 启用动画分支
            count = 0  # 绘图节流计数
            for node in self.vertex:  # 遍历搜索树节点
                count += 1  # 计数递增
                if node.parent:  # 非根节点才有边
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")  # 绘制父子连线
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])  # 按 Esc 退出
                    if count % 10 == 0:  # 每 10 条边刷新一次
                        plt.pause(0.001)  # 短暂停顿形成动画
        else:  # 不启用动画时
            for node in self.vertex:  # 遍历搜索树节点
                if node.parent:  # 非根节点才有边
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")  # 一次性绘制父子连线

    @staticmethod
    def plot_path(path, color='red'):  # 绘制路径折线
        plt.plot([x[0] for x in path], [x[1] for x in path], linewidth=2, color=color)  # 拆点后绘制路径
        plt.pause(0.01)  # 稍作停顿


def main():  # 示例入口
    x_start = (2, 2)  # Starting node
    x_goal = (49, 24)  # Goal node

    errt = ExtendedRrt(x_start, x_goal, 0.5, 0.1, 0.6, 5000)  # 构造扩展 RRT 实例
    errt.planning()  # 开始规划


if __name__ == '__main__':  # 脚本直接运行时的入口
    main()  # 调用主函数
