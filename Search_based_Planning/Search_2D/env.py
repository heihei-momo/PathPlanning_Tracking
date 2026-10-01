"""
Env 2D
@author: huiming zhou
"""


class Env:  # 2D 网格环境
    def __init__(self):  # 初始化尺寸、运动集合与障碍物
        self.x_range = 51  # size of background
        self.y_range = 31  # 网格 y 方向高度
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),  # 8 邻域运动增量（前 4 个）
                        (1, 0), (1, -1), (0, -1), (-1, -1)]  # 8 邻域运动增量（后 4 个）
        self.obs = self.obs_map()  # 生成障碍物坐标集合

    def update_obs(self, obs):  # 用外部集合替换障碍地图
        self.obs = obs  # 更新障碍物集合

    def obs_map(self):  # 构造障碍物地图
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        x = self.x_range  # 网格 x 方向宽度
        y = self.y_range  # 网格 y 方向高度
        obs = set()  # 障碍物坐标集合

        for i in range(x):  # 沿 x 方向遍历
            obs.add((i, 0))  # 填满下边界
        for i in range(x):  # 沿 x 方向遍历
            obs.add((i, y - 1))  # 填满上边界

        for i in range(y):  # 沿 y 方向遍历
            obs.add((0, i))  # 填满左边界
        for i in range(y):  # 沿 y 方向遍历
            obs.add((x - 1, i))  # 填满右边界

        for i in range(10, 21):  # 中间横墙 x 范围 10~20
            obs.add((i, 15))  # 添加中间横墙
        for i in range(15):  # 左侧竖墙 y 范围 0~14
            obs.add((20, i))  # 添加左侧竖墙

        for i in range(15, 30):  # 中间竖墙 y 范围 15~29
            obs.add((30, i))  # 添加中间竖墙
        for i in range(16):  # 右侧竖墙 y 范围 0~15
            obs.add((40, i))  # 添加右侧竖墙

        return obs  # 返回障碍物集合
