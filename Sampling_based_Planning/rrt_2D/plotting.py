"""
Plotting tools for Sampling-based algorithms
@author: huiming zhou
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env


class Plotting:  # 绘图工具类
    def __init__(self, x_start, x_goal):  # 初始化起终点与障碍
        self.xI, self.xG = x_start, x_goal  # 起点与终点坐标
        self.env = env.Env()  # 环境实例
        self.obs_bound = self.env.obs_boundary  # 边界墙
        self.obs_circle = self.env.obs_circle  # 圆形障碍
        self.obs_rectangle = self.env.obs_rectangle  # 矩形障碍

    def animation(self, nodelist, path, name, animation=False):  # 绘制搜索树与路径
        self.plot_grid(name)  # 画地图与障碍
        self.plot_visited(nodelist, animation)  # 画搜索树
        self.plot_path(path)  # 画最终路径

    def animation_connect(self, V1, V2, path, name):  # 双向搜索动画
        self.plot_grid(name)  # 画地图与障碍
        self.plot_visited_connect(V1, V2)  # 同时画两棵树
        self.plot_path(path)  # 画最终路径

    def plot_grid(self, name):  # 绘制地图、障碍与起终点
        fig, ax = plt.subplots()  # 创建画布与坐标轴

        for (ox, oy, w, h) in self.obs_bound:  # 绘制边界墙
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )  # 黑色实心矩形

        for (ox, oy, w, h) in self.obs_rectangle:  # 绘制矩形障碍
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )  # 灰色实心矩形

        for (ox, oy, r) in self.obs_circle:  # 绘制圆形障碍
            ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )  # 灰色实心圆

        plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3)  # 起点：蓝色方块
        plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3)  # 终点：绿色方块

        plt.title(name)  # 设置图标题
        plt.axis("equal")  # 等比例显示坐标

    @staticmethod
    def plot_visited(nodelist, animation):  # 绘制搜索树的边
        if animation:  # 动画模式
            count = 0  # 已绘制边计数
            for node in nodelist:  # 遍历树节点
                count += 1  # 计数加一
                if node.parent:  # 有父节点才连线
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")  # 画父子连线
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:  # 每10条刷新一次
                        plt.pause(0.001)  # 暂停以显示动画
        else:  # 非动画模式
            for node in nodelist:  # 遍历树节点
                if node.parent:  # 有父节点才连线
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")  # 一次性画出所有边

    @staticmethod
    def plot_visited_connect(V1, V2):  # 同时绘制两棵搜索树
        len1, len2 = len(V1), len(V2)  # 两棵树节点数

        for k in range(max(len1, len2)):  # 同步遍历两棵树
            if k < len1:  # 第一棵树仍有节点
                if V1[k].parent:  # 有父节点才连线
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")  # 画树1的边
            if k < len2:  # 第二棵树仍有节点
                if V2[k].parent:  # 有父节点才连线
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")  # 画树2的边

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 2 == 0:  # 隔一次刷新画面
                plt.pause(0.001)  # 暂停以显示动画

        plt.pause(0.01)  # 末尾稍作停顿

    @staticmethod
    def plot_path(path):  # 绘制最终路径
        if len(path) != 0:  # 路径非空
            plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)  # 红色折线表示路径
            plt.pause(0.01)  # 短暂停顿
        plt.show()  # 显示图像窗口
