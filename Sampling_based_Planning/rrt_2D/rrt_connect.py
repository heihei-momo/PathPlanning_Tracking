"""
RRT_CONNECT_2D
@author: huiming zhou
"""

import os
import sys
import math
import copy
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env, plotting, utils


class Node:  # 树节点：坐标与父指针
    def __init__(self, n):  # 由坐标元组构造节点
        self.x = n[0]  # 节点 x 坐标
        self.y = n[1]  # 节点 y 坐标
        self.parent = None  # 父节点指针：用于回溯路径


class RrtConnect:  # RRT-Connect 双向搜索规划器
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):  # 初始化起终点与扩展参数
        self.s_start = Node(s_start)  # 起点：树1的根节点
        self.s_goal = Node(s_goal)  # 终点：树2的根节点
        self.step_len = step_len  # 每次扩展的固定步长
        self.goal_sample_rate = goal_sample_rate  # 目标偏置采样概率
        self.iter_max = iter_max  # 最大迭代次数
        self.V1 = [self.s_start]  # 树1：从起点生长
        self.V2 = [self.s_goal]  # 树2：从终点生长

        self.env = env.Env()  # 加载环境地图
        self.plotting = plotting.Plotting(s_start, s_goal)  # 绘图工具实例
        self.utils = utils.Utils()  # 碰撞检测等工具

        self.x_range = self.env.x_range  # x 方向采样范围
        self.y_range = self.env.y_range  # y 方向采样范围
        self.obs_circle = self.env.obs_circle  # 圆形障碍列表
        self.obs_rectangle = self.env.obs_rectangle  # 矩形障碍列表
        self.obs_boundary = self.env.obs_boundary  # 地图边界障碍

    def planning(self):  # 主循环：两树交替扩展至相遇
        for i in range(self.iter_max):  # 迭代次数上限
            node_rand = self.generate_random_node(self.s_goal, self.goal_sample_rate)  # 随机点或目标偏置
            node_near = self.nearest_neighbor(self.V1, node_rand)  # 在树1中找最近邻节点
            node_new = self.new_state(node_near, node_rand)  # 从最近邻朝采样点扩展一步

            if node_new and not self.utils.is_collision(node_near, node_new):  # 新边无碰撞才有效
                self.V1.append(node_new)  # 新节点加入树1
                node_near_prim = self.nearest_neighbor(self.V2, node_new)  # 在树2中找距新节点最近者
                node_new_prim = self.new_state(node_near_prim, node_new)  # 树2朝树1方向扩展一步

                if node_new_prim and not self.utils.is_collision(node_new_prim, node_near_prim):  # 校验树2边
                    self.V2.append(node_new_prim)  # 树2新节点入树

                    while True:  # connect：贪心连续朝对方直冲
                        node_new_prim2 = self.new_state(node_new_prim, node_new)  # 再向树1新节点迈进一步
                        if node_new_prim2 and not self.utils.is_collision(node_new_prim2, node_new_prim):
                            self.V2.append(node_new_prim2)  # 扩展节点加入树2
                            node_new_prim = self.change_node(node_new_prim, node_new_prim2)  # 游标前移
                        else:  # 遇障碍或无法前进时
                            break  # 结束直冲循环

                        if self.is_node_same(node_new_prim, node_new):  # 树2已推进到树1新节点
                            break  # 两树相遇，退出直冲

                if self.is_node_same(node_new_prim, node_new):  # 若两树重合则拼接路径
                    return self.extract_path(node_new, node_new_prim)  # 回溯两棵树得到完整路径

            if len(self.V2) < len(self.V1):  # 下一轮扩展节点更少的树
                list_mid = self.V2  # 暂存树2引用
                self.V2 = self.V1  # 树1转为待扩展树
                self.V1 = list_mid  # 树2接管原树1，完成互换

        return None  # 迭代耗尽仍未连通

    @staticmethod
    def change_node(node_new_prim, node_new_prim2):  # 由已扩展点生成前移节点
        node_new = Node((node_new_prim2.x, node_new_prim2.y))  # 复制最新点的坐标
        node_new.parent = node_new_prim  # 父节点指向原游标节点

        return node_new  # 返回前移后的新节点

    @staticmethod
    def is_node_same(node_new_prim, node_new):  # 判断两节点坐标是否重合
        if node_new_prim.x == node_new.x and \
                node_new_prim.y == node_new.y:  # 坐标一致即视为相遇
            return True  # 判定为两树相遇

        return False  # 坐标不同则未相遇

    def generate_random_node(self, sample_goal, goal_sample_rate):  # 采样：按概率偏向目标点
        delta = self.utils.delta  # 采样时保留的边界余量

        if np.random.random() > goal_sample_rate:  # 未命中目标偏置分支
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))  # 自由空间采样

        return sample_goal  # 直接以目标点作为采样点

    @staticmethod
    def nearest_neighbor(node_list, n):  # 返回树中距 n 最近的节点
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]  # 按欧氏距离取最小者

    def new_state(self, node_start, node_end):  # 从起点朝终点扩展一步
        dist, theta = self.get_distance_and_angle(node_start, node_end)  # 计算两点距离与方位角

        dist = min(self.step_len, dist)  # 步长截断：不越过目标点
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))  # 沿射线前进得到新节点
        node_new.parent = node_start  # 记录父节点以形成树的边

        return node_new  # 返回扩展出的新节点

    @staticmethod
    def extract_path(node_new, node_new_prim):  # 相遇后拼接两棵树的路径
        path1 = [(node_new.x, node_new.y)]  # 树1侧路径以相遇节点开头
        node_now = node_new  # 从相遇节点开始回溯

        while node_now.parent is not None:  # 沿 parent 链回溯到树1根
            node_now = node_now.parent  # 上移到父节点
            path1.append((node_now.x, node_now.y))  # 收集路径点

        path2 = [(node_new_prim.x, node_new_prim.y)]  # 树2侧路径以相遇节点开头
        node_now = node_new_prim  # 从树2相遇节点回溯

        while node_now.parent is not None:  # 沿 parent 链回溯到树2根
            node_now = node_now.parent  # 上移到父节点
            path2.append((node_now.x, node_now.y))  # 收集路径点

        return list(list(reversed(path1)) + path2)  # 树1路径反转后接树2路径

    @staticmethod
    def get_distance_and_angle(node_start, node_end):  # 计算两点距离与方位角
        dx = node_end.x - node_start.x  # x 方向增量
        dy = node_end.y - node_start.y  # y 方向增量
        return math.hypot(dx, dy), math.atan2(dy, dx)  # 返回欧氏距离与方向角


def main():  # 示例入口：设置起终点并规划
    x_start = (2, 2)  # Starting node
    x_goal = (49, 24)  # Goal node

    rrt_conn = RrtConnect(x_start, x_goal, 0.8, 0.05, 5000)  # 构造 RRT-Connect 实例
    path = rrt_conn.planning()  # 执行双向搜索

    rrt_conn.plotting.animation_connect(rrt_conn.V1, rrt_conn.V2, path, "RRT_CONNECT")  # 动画展示两树与路径


if __name__ == '__main__':  # 脚本直接运行时的入口
    main()  # 调用主函数
