"""
Plot tools 2D
@author: huiming zhou
"""

import os  # 路径处理
import sys  # 模块搜索路径
import matplotlib.pyplot as plt  # 绘图与动画库

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")  # 把算法根目录加入搜索路径

from Search_2D import env  # 导入 2D 环境


class Plotting:  # 绘图与动画工具类
    def __init__(self, xI, xG):  # 记录起点与终点
        self.xI, self.xG = xI, xG  # 起点、终点坐标
        self.env = env.Env()  # 创建 2D 环境
        self.obs = self.env.obs_map()  # 获取障碍物集合

    def update_obs(self, obs):  # 替换绘图用障碍集合
        self.obs = obs  # 更新障碍物集合

    def animation(self, path, visited, name):  # 通用动画：网格+访问+路径
        self.plot_grid(name)  # 绘制网格与障碍
        self.plot_visited(visited)  # 动态绘制访问顺序
        self.plot_path(path)  # 绘制最终路径
        plt.show()  # 显示图像窗口

    def animation_lrta(self, path, visited, name):  # LRTA* 多段动画
        self.plot_grid(name)  # 绘制网格与障碍
        cl = self.color_list_2()  # 获取多段配色
        path_combine = []  # 汇总所有分段路径

        for k in range(len(path)):  # 逐段播放
            self.plot_visited(visited[k], cl[k])  # 画第 k 段访问点
            plt.pause(0.2)  # 停顿显示动画
            self.plot_path(path[k])  # 画第 k 段路径
            path_combine += path[k]  # 合并进总路径
            plt.pause(0.2)  # 停顿显示动画
        if self.xI in path_combine:  # 总路径包含起点
            path_combine.remove(self.xI)  # 去掉重复的起点
        self.plot_path(path_combine)  # 绘制合并后的路径
        plt.show()  # 显示图像窗口

    def animation_ara_star(self, path, visited, name):  # ARA* 多权重动画
        self.plot_grid(name)  # 绘制网格与障碍
        cl_v, cl_p = self.color_list()  # 访问点与路径两组配色

        for k in range(len(path)):  # 逐个权重绘制
            self.plot_visited(visited[k], cl_v[k])  # 画第 k 次访问点
            self.plot_path(path[k], cl_p[k], True)  # 用指定颜色画路径
            plt.pause(0.5)  # 停顿显示动画

        plt.show()  # 显示图像窗口

    def animation_bi_astar(self, path, v_fore, v_back, name):  # 双向 A* 动画
        self.plot_grid(name)  # 绘制网格与障碍
        self.plot_visited_bi(v_fore, v_back)  # 画双向访问点
        self.plot_path(path)  # 绘制最终路径
        plt.show()  # 显示图像窗口

    def plot_grid(self, name):  # 绘制网格底图
        obs_x = [x[0] for x in self.obs]  # 障碍物 x 坐标
        obs_y = [x[1] for x in self.obs]  # 障碍物 y 坐标

        plt.plot(self.xI[0], self.xI[1], "bs")  # 蓝色方块标起点
        plt.plot(self.xG[0], self.xG[1], "gs")  # 绿色方块标终点
        plt.plot(obs_x, obs_y, "sk")  # 黑色方块画障碍
        plt.title(name)  # 设置图标题
        plt.axis("equal")  # 等比例坐标轴

    def plot_visited(self, visited, cl='gray'):  # 按顺序逐点绘制访问点
        if self.xI in visited:  # 访问列表含起点
            visited.remove(self.xI)  # 去掉起点避免重复绘制

        if self.xG in visited:  # 访问列表含终点
            visited.remove(self.xG)  # 去掉终点避免重复绘制

        count = 0  # 已绘制点数计数

        for x in visited:  # 按访问顺序遍历
            count += 1  # 计数加一
            plt.plot(x[0], x[1], color=cl, marker='o')  # 画当前访问点
            plt.gcf().canvas.mpl_connect('key_release_event',  # 绑定按键事件
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if count < len(visited) / 3:  # 前 1/3 访问点
                length = 20  # 刷新间隔 20
            elif count < len(visited) * 2 / 3:  # 中间 1/3 访问点
                length = 30  # 刷新间隔 30
            else:  # 后 1/3 访问点
                length = 40  # 刷新间隔 40
            #
            # length = 15

            if count % length == 0:  # 每 length 个点刷新一次
                plt.pause(0.001)  # 短暂停顿刷新画面
        plt.pause(0.01)  # 收尾停顿

    def plot_path(self, path, cl='r', flag=False):  # 绘制规划路径
        path_x = [path[i][0] for i in range(len(path))]  # 路径点 x 坐标
        path_y = [path[i][1] for i in range(len(path))]  # 路径点 y 坐标

        if not flag:  # 使用默认红色
            plt.plot(path_x, path_y, linewidth='3', color='r')  # 画红色路径
        else:  # 使用指定颜色
            plt.plot(path_x, path_y, linewidth='3', color=cl)  # 画指定颜色路径

        plt.plot(self.xI[0], self.xI[1], "bs")  # 重画起点标记
        plt.plot(self.xG[0], self.xG[1], "gs")  # 重画终点标记

        plt.pause(0.01)  # 短暂停顿刷新画面

    def plot_visited_bi(self, v_fore, v_back):  # 双向搜索访问点动画
        if self.xI in v_fore:  # 前向访问含起点
            v_fore.remove(self.xI)  # 去掉起点

        if self.xG in v_back:  # 后向访问含终点
            v_back.remove(self.xG)  # 去掉终点

        len_fore, len_back = len(v_fore), len(v_back)  # 两侧访问点数量

        for k in range(max(len_fore, len_back)):  # 按较长一侧遍历
            if k < len_fore:  # 前向还有第 k 个点
                plt.plot(v_fore[k][0], v_fore[k][1], linewidth='3', color='gray', marker='o')  # 前向访问点
            if k < len_back:  # 后向还有第 k 个点
                plt.plot(v_back[k][0], v_back[k][1], linewidth='3', color='cornflowerblue', marker='o')

            plt.gcf().canvas.mpl_connect('key_release_event',  # 绑定按键事件
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 10 == 0:  # 每 10 个点刷新一次
                plt.pause(0.001)  # 短暂停顿刷新画面
        plt.pause(0.01)  # 收尾停顿

    @staticmethod  # 静态方法，无需实例
    def color_list():  # ARA* 使用的两组配色
        cl_v = ['silver',  # 访问点颜色列表（由浅到深）
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']
        cl_p = ['gray',  # 路径颜色列表（对应不同权重）
                'orange',
                'deepskyblue',
                'red',
                'm']
        return cl_v, cl_p  # 返回访问点与路径配色

    @staticmethod  # 静态方法，无需实例
    def color_list_2():  # LRTA* 使用的多段配色
        cl = ['silver',  # 分段路径颜色列表
              'steelblue',
              'dimgray',
              'cornflowerblue',
              'dodgerblue',
              'royalblue',
              'plum',
              'mediumslateblue',
              'mediumpurple',
              'blueviolet',
              ]
        return cl  # 返回配色列表
