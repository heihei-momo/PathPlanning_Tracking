
import os  # 路径处理
import sys  # 模块搜索路径
import math  # 无穷大等数学运算
import heapq  # 用堆实现 OPEN 表

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")  # 把算法根目录加入搜索路径

from Search_2D import plotting, env  # 绘图与环境模块
from Search_2D.Astar import AStar  # 继承 A* 基类

class DFS(AStar):  # 深度优先搜索
    """DFS add the new visited node in the front of the openset
    """
    def searching(self):  # 执行 DFS 搜索
        """
        Breadth-first Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start  # 起点父节点指向自身
        self.g[self.s_start] = 0  # 起点代价为 0
        self.g[self.s_goal] = math.inf  # 终点代价先置为无穷
        heapq.heappush(self.OPEN,
                       (0, self.s_start))  # 起点以优先级 0 入队

        while self.OPEN:  # OPEN 表非空则继续
            _, s = heapq.heappop(self.OPEN)  # 弹出优先级最小的节点
            self.CLOSED.append(s)  # 记录访问顺序

            if s == self.s_goal:  # 到达终点
                break  # 结束搜索

            for s_n in self.get_neighbor(s):  # 遍历 8 邻域邻居
                new_cost = self.g[s] + self.cost(s, s_n)  # 经 s 到邻居的新代价

                if s_n not in self.g:  # 邻居首次被访问
                    self.g[s_n] = math.inf  # 代价初始化为无穷

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost  # 更新邻居的 g 值
                    self.PARENT[s_n] = s  # 记录父节点用于回溯

                    # dfs, add new node to the front of the openset
                    prior = self.OPEN[0][0]-1 if len(self.OPEN)>0 else 0  # 优先级取队首减一
                    heapq.heappush(self.OPEN, (prior, s_n))  # 新节点排到队首，保证后进先出

        return self.extract_path(self.PARENT), self.CLOSED  # 返回路径与访问顺序


def main():  # 测试入口
    s_start = (5, 5)  # 起点坐标
    s_goal = (45, 25)  # 终点坐标

    dfs = DFS(s_start, s_goal, 'None')  # 构造 DFS 实例，不用启发式
    plot = plotting.Plotting(s_start, s_goal)  # 构造绘图对象

    path, visited = dfs.searching()  # 执行搜索得到路径与访问顺序
    visited = list(dict.fromkeys(visited))  # 按原顺序去重访问点
    plot.animation(path, visited, "Depth-first Searching (DFS)")  # animation


if __name__ == '__main__':  # 直接运行本文件时
    main()  # 调用主函数
