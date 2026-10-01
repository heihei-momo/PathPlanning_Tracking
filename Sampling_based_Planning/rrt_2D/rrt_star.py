"""
RRT_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env, plotting, utils, queue


class Node:  # RRT* 树节点
    def __init__(self, n):  # 由坐标元组构造节点
        self.x = n[0]  # 节点 x 坐标
        self.y = n[1]  # 节点 y 坐标
        self.parent = None  # 父指针：构成树的边


class RrtStar:  # RRT* 渐近最优规划器
    def __init__(self, x_start, x_goal, step_len,
                 goal_sample_rate, search_radius, iter_max):  # 初始化起终点与规划参数
        self.s_start = Node(x_start)  # 起点节点
        self.s_goal = Node(x_goal)  # 目标节点
        self.step_len = step_len  # 扩展步长
        self.goal_sample_rate = goal_sample_rate  # 目标偏置采样概率
        self.search_radius = search_radius  # 邻域搜索半径系数
        self.iter_max = iter_max  # 最大采样迭代次数
        self.vertex = [self.s_start]  # 树节点集合，初始仅起点
        self.path = []  # 最终路径容器

        self.env = env.Env()  # 加载环境地图
        self.plotting = plotting.Plotting(x_start, x_goal)  # 绘图工具实例
        self.utils = utils.Utils()  # 碰撞检测等工具

        self.x_range = self.env.x_range  # x 方向采样范围
        self.y_range = self.env.y_range  # y 方向采样范围
        self.obs_circle = self.env.obs_circle  # 圆形障碍列表
        self.obs_rectangle = self.env.obs_rectangle  # 矩形障碍列表
        self.obs_boundary = self.env.obs_boundary  # 地图边界障碍

    def planning(self):  # 主循环：采样-扩展-选父-重连
        for k in range(self.iter_max):  # 迭代采样次数
            node_rand = self.generate_random_node(self.goal_sample_rate)  # 采样点：随机或目标偏置
            node_near = self.nearest_neighbor(self.vertex, node_rand)  # 树中距采样点最近节点
            node_new = self.new_state(node_near, node_rand)  # 朝采样点扩展一步

            if k % 500 == 0:  # 每 500 次打印迭代进度
                print(k)  # 输出当前迭代序号

            if node_new and not self.utils.is_collision(node_near, node_new):  # 新边无碰撞才纳入树
                neighbor_index = self.find_near_neighbor(node_new)  # 查找新节点的邻域索引
                self.vertex.append(node_new)  # 新节点先加入树

                if neighbor_index:  # 存在邻域候选时
                    self.choose_parent(node_new, neighbor_index)  # 选累计代价最小的父节点
                    self.rewire(node_new, neighbor_index)  # 重连：用新节点改进邻居

        index = self.search_goal_parent()  # 选可直达目标的节点
        self.path = self.extract_path(self.vertex[index])  # 沿 parent 链回溯路径

        self.plotting.animation(self.vertex, self.path, "rrt*, N = " + str(self.iter_max))  # 动画展示树与路径

    def new_state(self, node_start, node_goal):  # 由起点朝目标扩展一步
        dist, theta = self.get_distance_and_angle(node_start, node_goal)  # 计算两点距离与方位角

        dist = min(self.step_len, dist)  # 步长截断：不越过目标
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))  # 沿射线前进得到新节点

        node_new.parent = node_start  # 暂以最近邻作为父节点

        return node_new  # 返回扩展出的新节点

    def choose_parent(self, node_new, neighbor_index):  # 选父：最小化累计代价
        cost = [self.get_new_cost(self.vertex[i], node_new) for i in neighbor_index]  # 各候选父节点的路径代价

        cost_min_index = neighbor_index[int(np.argmin(cost))]  # 取代价最小的候选索引
        node_new.parent = self.vertex[cost_min_index]  # 重设新节点的父节点

    def rewire(self, node_new, neighbor_index):  # 重连：缩短邻居的代价
        for i in neighbor_index:  # 遍历邻域节点索引
            node_neighbor = self.vertex[i]  # 取出该邻域节点

            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):  # 经新节点更省
                node_neighbor.parent = node_new  # 邻居改挂到新节点下

    def search_goal_parent(self):  # 选可连接到目标的父节点
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.vertex]  # 到目标距离
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]  # 可一步直达目标

        if len(node_index) > 0:  # 存在候选节点时
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index  # 总代价含到目标距离
                         if not self.utils.is_collision(self.vertex[i], self.s_goal)]
            return node_index[int(np.argmin(cost_list))]  # 返回总代价最小的索引

        return len(self.vertex) - 1  # 兜底：返回最新节点

    def get_new_cost(self, node_start, node_end):  # 计算经某父节点的新代价
        dist, _ = self.get_distance_and_angle(node_start, node_end)  # 两节点间的距离

        return self.cost(node_start) + dist  # 父节点代价加边长

    def generate_random_node(self, goal_sample_rate):  # 采样：按概率偏向目标
        delta = self.utils.delta  # 采样时保留的边界余量

        if np.random.random() > goal_sample_rate:  # 未命中目标偏置分支
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))  # 自由空间采样

        return self.s_goal  # 直接返回目标点

    def find_near_neighbor(self, node_new):  # 查找半径 r 内的邻居
        n = len(self.vertex) + 1  # 当前节点规模（含新节点）
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)  # 随规模衰减的邻域半径

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]  # 到新节点距离
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.utils.is_collision(node_new, self.vertex[ind])]  # 保留半径内且无碰撞者

        return dist_table_index  # 返回邻域索引列表

    @staticmethod
    def nearest_neighbor(node_list, n):  # 返回树中距 n 最近的节点
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]  # 按欧氏距离取最小者

    @staticmethod
    def cost(node_p):  # 沿 parent 链累计路径代价
        node = node_p  # 从该节点开始回溯
        cost = 0.0  # 代价初值为 0

        while node.parent:  # 回溯至根节点为止
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)  # 累加每段边长
            node = node.parent  # 上移到父节点

        return cost  # 返回该节点的累计代价

    def update_cost(self, parent_node):  # 按层更新子树累计代价
        OPEN = queue.QueueFIFO()  # 先进先出队列
        OPEN.put(parent_node)  # 以父节点为传播起点

        while not OPEN.empty():  # 队列非空则继续
            node = OPEN.get()  # 取出一个待更新节点

            if len(node.child) == 0:  # 叶节点无需更新
                continue  # 跳过

            for node_c in node.child:  # 遍历其子节点
                node_c.Cost = self.get_new_cost(node, node_c)  # 重算子节点的累计代价
                OPEN.put(node_c)  # 子节点入队继续传播

    def extract_path(self, node_end):  # 从目标沿 parent 链回溯
        path = [[self.s_goal.x, self.s_goal.y]]  # 路径以目标点开头
        node = node_end  # 从目标父节点开始回溯

        while node.parent is not None:  # 回溯至起点为止
            path.append([node.x, node.y])  # 加入路径点
            node = node.parent  # 上移到父节点
        path.append([node.x, node.y])  # 补上起点

        return path  # 返回路径（目标到起点）

    @staticmethod
    def get_distance_and_angle(node_start, node_end):  # 计算两点距离与方位角
        dx = node_end.x - node_start.x  # x 方向增量
        dy = node_end.y - node_start.y  # y 方向增量
        return math.hypot(dx, dy), math.atan2(dy, dx)  # 返回欧氏距离与方向角


def main():  # 示例入口
    x_start = (18, 8)  # Starting node
    x_goal = (37, 18)  # Goal node

    rrt_star = RrtStar(x_start, x_goal, 10, 0.10, 20, 10000)  # 构造 RRT* 实例
    rrt_star.planning()  # 开始规划


if __name__ == '__main__':  # 脚本直接运行时的入口
    main()  # 调用主函数
