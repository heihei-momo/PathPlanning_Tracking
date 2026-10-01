"""
DUBINS_RRT_STAR 2D
@author: huiming zhou
"""

import os  # 操作系统路径接口
import sys  # 系统模块搜索路径
import math  # 数学函数库
import random  # 随机采样
import numpy as np  # 数值计算
import matplotlib.pyplot as plt  # 绘图
import matplotlib.patches as patches  # 障碍物图元
from scipy.spatial.transform import Rotation as Rot  # 旋转矩阵（导入未使用）

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")  # 追加仓库根目录到搜索路径

from Sampling_based_Planning.rrt_2D import env, plotting, utils  # 环境/绘图/工具模块
import CurvesGenerator.dubins_path as dubins  # Dubins 曲线求解
import CurvesGenerator.draw as draw  # 箭头绘制工具


class Node:  # 树节点
    def __init__(self, x, y, yaw):  # 由位姿构造节点
        self.x = x  # 节点 x 坐标
        self.y = y  # 节点 y 坐标
        self.yaw = yaw  # 节点 yaw 航向角
        self.parent = None  # 父节点
        self.cost = 0.0  # 到起点累计代价
        self.path_x = []  # 父到本节点的路径 x
        self.path_y = []  # 父到本节点的路径 y
        self.paty_yaw = []  # 路径航向角序列


class DubinsRRTStar:  # Dubins-RRT* 规划器
    def __init__(self, sx, sy, syaw, gx, gy, gyaw, vehicle_radius, step_len,  # 起点终点位姿与步长
                 goal_sample_rate, search_radius, iter_max):  # 目标偏置率、近邻半径、迭代上限
        self.s_start = Node(sx, sy, syaw)  # 起点节点
        self.s_goal = Node(gx, gy, gyaw)  # 终点节点
        self.vr = vehicle_radius  # 车辆半径
        self.step_len = step_len  # 扩展步长上限
        self.goal_sample_rate = goal_sample_rate  # 采样到终点的概率
        self.search_radius = search_radius  # 近邻半径系数
        self.iter_max = iter_max  # 最大迭代次数
        self.curv = 1  # Dubins 曲线最大曲率

        self.env = env.Env()  # 地图环境
        self.utils = utils.Utils()  # 碰撞检测工具

        self.fig, self.ax = plt.subplots()  # 创建画布与坐标轴
        self.delta = self.utils.delta  # 安全膨胀距离
        self.x_range = self.env.x_range  # 地图 x 范围
        self.y_range = self.env.y_range  # 地图 y 范围
        self.obs_circle = self.obs_circle()  # 圆形障碍列表
        self.obs_boundary = self.env.obs_boundary  # 边界障碍
        self.utils.update_obs(self.obs_circle, self.obs_boundary, [])  # 更新障碍集合

        self.V = [self.s_start]  # 树节点集合
        self.path = None  # 最终路径

    def planning(self):  # 主循环

        for i in range(self.iter_max):  # 迭代扩展
            print("Iter:", i, ", number of nodes:", len(self.V))  # 打印迭代与节点数
            rnd = self.Sample()  # 随机采样
            node_nearest = self.Nearest(self.V, rnd)  # 最近邻节点
            new_node = self.Steer(node_nearest, rnd)  # Dubins 曲线扩展

            if new_node and not self.is_collision(new_node):  # 新曲线无碰撞
                near_indexes = self.Near(self.V, new_node)  # 近邻索引
                new_node = self.choose_parent(new_node, near_indexes)  # 选最优父节点

                if new_node:  # 选父成功
                    self.V.append(new_node)  # 新节点加入树
                    self.rewire(new_node, near_indexes)  # 重连近邻

            if i % 5 == 0:  # 每 5 次迭代
                self.draw_graph()  # 刷新绘图

        last_index = self.search_best_goal_node()  # 找最优终点节点

        path = self.generate_final_course(last_index)  # 生成最终路径
        print("get!")  # 完成提示
        px = [s[0] for s in path]  # 路径 x 序列
        py = [s[1] for s in path]  # 路径 y 序列
        plt.plot(px, py, '-r')  # 红色画出路径
        plt.pause(0.01)  # 短暂停顿
        plt.show()  # 保持窗口显示

    def draw_graph(self, rnd=None):  # 动态绘图
        plt.cla()  # 清空画布
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',  # 绑定按键事件
                                     lambda event: [exit(0) if event.key == 'escape' else None])  # ESC 退出
        for node in self.V:  # 遍历树
            if node.parent:  # 非根节点
                plt.plot(node.path_x, node.path_y, "-g")  # 绿色画 Dubins 边

        self.plot_grid("dubins rrt*")  # 绘制栅格与障碍
        plt.plot(self.s_start.x, self.s_start.y, "xr")  # 红色叉标起点
        plt.plot(self.s_goal.x, self.s_goal.y, "xr")  # 红色叉标终点
        plt.grid(True)  # 显示网格
        self.plot_start_goal_arrow()  # 绘制起终点朝向箭头
        plt.pause(0.01)  # 停顿刷新

    def plot_start_goal_arrow(self):  # 绘制起终点箭头
        draw.Arrow(self.s_start.x, self.s_start.y, self.s_start.yaw, 2, "darkorange")  # 起点朝向箭头
        draw.Arrow(self.s_goal.x, self.s_goal.y, self.s_goal.yaw, 2, "darkorange")  # 终点朝向箭头

    def generate_final_course(self, goal_index):  # 回溯最终路径
        print("final")  # 提示开始回溯
        path = [[self.s_goal.x, self.s_goal.y]]  # 从终点开始
        node = self.V[goal_index]  # 最优终点节点
        while node.parent:  # 沿父指针上溯
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):  # 逆序取 Dubins 路径点
                path.append([ix, iy])  # 加入路径
            node = node.parent  # 上溯
        path.append([self.s_start.x, self.s_start.y])  # 补上起点
        return path  # 返回路径

    def calc_dist_to_goal(self, x, y):  # 到终点距离
        dx = x - self.s_goal.x  # 横向差
        dy = y - self.s_goal.y  # 纵向差
        return math.hypot(dx, dy)  # 欧氏距离

    def search_best_goal_node(self):  # 搜索最优终点节点
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.V]  # 各节点到终点距离
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.step_len]  # 候选索引

        safe_goal_inds = []  # 可安全连接终点的索引
        for goal_ind in goal_inds:  # 遍历候选索引
            t_node = self.Steer(self.V[goal_ind], self.s_goal)  # 试连终点
            if t_node and not self.is_collision(t_node):  # 曲线有效且无碰撞
                safe_goal_inds.append(goal_ind)  # 记为安全候选

        if not safe_goal_inds:  # 无可行候选
            return None  # 返回空

        min_cost = min([self.V[i].cost for i in safe_goal_inds])  # 候选最小代价
        for i in safe_goal_inds:  # 遍历安全候选
            if self.V[i].cost == min_cost:  # 找到最小代价者
                return i  # 返回其索引

        return None  # 未找到

    def rewire(self, new_node, near_inds):  # 重连近邻
        for i in near_inds:  # 遍历近邻索引
            near_node = self.V[i]  # 近邻节点
            edge_node = self.Steer(new_node, near_node)  # 用 Dubins 曲线连接
            if not edge_node:  # 连接失败
                continue  # 跳过
            edge_node.cost = self.calc_new_cost(new_node, near_node)  # 新边代价

            no_collision = ~self.is_collision(edge_node)  # 无碰撞标志
            improved_cost = near_node.cost > edge_node.cost  # 代价是否降低

            if no_collision and improved_cost:  # 无碰撞且更优
                self.V[i] = edge_node  # 替换近邻节点
                self.propagate_cost_to_leaves(new_node)  # 更新后代代价

    def choose_parent(self, new_node, near_inds):  # 选择最优父节点
        if not near_inds:  # 无近邻
            return None  # 返回空

        costs = []  # 各近邻连接代价
        for i in near_inds:  # 遍历近邻
            near_node = self.V[i]  # 近邻节点
            t_node = self.Steer(near_node, new_node)  # Dubins 连接
            if t_node and not self.is_collision(t_node):  # 曲线有效
                costs.append(self.calc_new_cost(near_node, new_node))  # 记录代价
            else:  # 连接失败
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)  # 最小代价

        if min_cost == float("inf"):  # 全部不可连
            print("There is no good path.(min_cost is inf)")  # 提示无可行路径
            return None  # 返回空

        min_ind = near_inds[costs.index(min_cost)]  # 最优近邻索引
        new_node = self.Steer(self.V[min_ind], new_node)  # 重连到最优父节点

        return new_node  # 返回新节点

    def calc_new_cost(self, from_node, to_node):  # 计算新边代价
        d, _ = self.get_distance_and_angle(from_node, to_node)  # 直线距离
        return from_node.cost + d  # 父代价加边长

    def propagate_cost_to_leaves(self, parent_node):  # 向叶节点传播代价
        for node in self.V:  # 遍历所有节点
            if node.parent == parent_node:  # 是其后代
                node.cost = self.calc_new_cost(parent_node, node)  # 更新代价
                self.propagate_cost_to_leaves(node)  # 递归更新

    @staticmethod  # 静态方法
    def get_distance_and_angle(node_start, node_end):  # 距离与方位角
        dx = node_end.x - node_start.x  # 横向差
        dy = node_end.y - node_start.y  # 纵向差
        return math.hypot(dx, dy), math.atan2(dy, dx)  # 距离与夹角

    def Near(self, nodelist, node):  # 近邻搜索
        n = len(nodelist) + 1  # 当前节点数
        r = min(self.search_radius * math.sqrt((math.log(n)) / n), self.step_len)  # 半径取小者

        dist_table = [(nd.x - node.x) ** 2 + (nd.y - node.y) ** 2 for nd in nodelist]  # 距离平方表
        node_near_ind = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r ** 2]  # 半径内索引

        return node_near_ind  # 返回索引列表

    def Steer(self, node_start, node_end):  # 用 Dubins 曲线扩展
        sx, sy, syaw = node_start.x, node_start.y, node_start.yaw  # 起点位姿
        gx, gy, gyaw = node_end.x, node_end.y, node_end.yaw  # 目标位姿
        maxc = self.curv  # 最大曲率

        path = dubins.calc_dubins_path(sx, sy, syaw, gx, gy, gyaw, maxc)  # 求解 Dubins 曲线

        if len(path.x) <= 1:  # 无有效路径
            return None  # 返回空

        node_new = Node(path.x[-1], path.y[-1], path.yaw[-1])  # 曲线终点作为新节点
        node_new.path_x = path.x  # 记录路径 x
        node_new.path_y = path.y  # 记录路径 y
        node_new.path_yaw = path.yaw  # 记录路径航向
        node_new.cost = node_start.cost + path.L  # 累计代价加曲线长度
        node_new.parent = node_start  # 设置父节点

        return node_new  # 返回新节点

    def Sample(self):  # 随机采样
        delta = self.utils.delta  # 安全边距

        if random.random() > self.goal_sample_rate:  # 非目标偏置
            return Node(random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),  # 随机 x
                        random.uniform(self.y_range[0] + delta, self.y_range[1] - delta),  # 随机 y
                        random.uniform(-math.pi, math.pi))  # 随机航向角
        else:  # 目标偏置
            return self.s_goal  # 返回终点

    @staticmethod  # 静态方法
    def Nearest(nodelist, n):  # 最近邻节点
        return nodelist[int(np.argmin([(nd.x - n.x) ** 2 + (nd.y - n.y) ** 2  # 距离平方最小
                                       for nd in nodelist]))]  # 遍历所有节点

    def is_collision(self, node):  # 曲线碰撞检测
        for ox, oy, r in self.obs_circle:  # 遍历圆形障碍
            dx = [ox - x for x in node.path_x]  # 各点到圆心横向差
            dy = [oy - y for y in node.path_y]  # 各点到圆心纵向差
            dist = np.hypot(dx, dy)  # 各点到圆心距离

            if min(dist) < r + self.delta:  # 进入膨胀障碍
                return True  # 判定碰撞

        return False  # 无碰撞

    def animation(self):  # 最终绘图
        self.plot_grid("dubins rrt*")  # 绘制栅格与障碍
        self.plot_arrow()  # 绘制箭头
        plt.show()  # 显示窗口

    def plot_arrow(self):  # 绘制起终点箭头
        draw.Arrow(self.s_start.x, self.s_start.y, self.s_start.yaw, 2.5, "darkorange")  # 起点朝向
        draw.Arrow(self.s_goal.x, self.s_goal.y, self.s_goal.yaw, 2.5, "darkorange")  # 终点朝向

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

        for (ox, oy, r) in self.obs_circle:  # 圆形障碍
            self.ax.add_patch(  # 添加图元
                patches.Circle(  # 圆形图元
                    (ox, oy), r,  # 圆心与半径
                    edgecolor='black',  # 黑色边框
                    facecolor='gray',  # 灰色填充
                    fill=True  # 实心填充
                )
            )

        plt.plot(self.s_start.x, self.s_start.y, "bs", linewidth=3)  # 蓝色方块标起点
        plt.plot(self.s_goal.x, self.s_goal.y, "gs", linewidth=3)  # 绿色方块标终点

        plt.title(name)  # 图标题
        plt.axis("equal")  # 等比例坐标

    @staticmethod  # 静态方法
    def obs_circle():  # 圆形障碍定义
        obs_cir = [  # 圆形障碍列表
            [10, 10, 3],  # 圆心与半径
            [15, 22, 3],  # 圆心与半径
            [22, 8, 2.5],  # 圆心与半径
            [26, 16, 2],  # 圆心与半径
            [37, 10, 3],  # 圆心与半径
            [37, 23, 3],  # 圆心与半径
            [45, 15, 2]  # 圆心与半径
        ]

        return obs_cir  # 返回障碍列表


def main():  # 主函数
    sx, sy, syaw = 5, 5, np.deg2rad(90)  # 起点位姿（朝上）
    gx, gy, gyaw = 45, 25, np.deg2rad(0)  # 终点位姿（朝右）
    goal_sample_rate = 0.1  # 目标偏置率
    search_radius = 50.0  # 近邻半径系数
    step_len = 30.0  # 步长上限
    iter_max = 250  # 最大迭代次数
    vehicle_radius = 2.0  # 车辆半径

    drrtstar = DubinsRRTStar(sx, sy, syaw, gx, gy, gyaw, vehicle_radius, step_len,  # 构造规划器
                             goal_sample_rate, search_radius, iter_max)  # 其余参数
    drrtstar.planning()  # 开始规划


if __name__ == '__main__':  # 脚本入口
    main()  # 运行主函数
