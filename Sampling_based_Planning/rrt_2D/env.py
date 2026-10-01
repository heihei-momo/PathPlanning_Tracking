"""
Environment for rrt_2D
@author: huiming zhou
"""


class Env:  # 环境类：定义地图范围与障碍
    def __init__(self):  # 初始化地图与障碍
        self.x_range = (0, 50)  # x 方向范围
        self.y_range = (0, 30)  # y 方向范围
        self.obs_boundary = self.obs_boundary()  # 边界墙障碍
        self.obs_circle = self.obs_circle()  # 圆形障碍
        self.obs_rectangle = self.obs_rectangle()  # 矩形障碍

    @staticmethod
    def obs_boundary():  # 四周边界墙
        obs_boundary = [  # [x, y, w, h]
            [0, 0, 1, 30],  # 左边界墙
            [0, 30, 50, 1],  # 上边界墙
            [1, 0, 50, 1],  # 下边界墙
            [50, 1, 1, 30]  # 右边界墙
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():  # 矩形障碍列表
        obs_rectangle = [  # [x, y, w, h]
            [14, 12, 8, 2],  # 中部偏下矩形障碍
            [18, 22, 8, 3],  # 上方矩形障碍
            [26, 7, 2, 12],  # 竖直长条障碍
            [32, 14, 10, 2]  # 右侧矩形障碍
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():  # 圆形障碍列表
        obs_cir = [  # [x, y, r]
            [7, 12, 3],  # 左侧圆形障碍
            [46, 20, 2],  # 右上圆形障碍
            [15, 5, 2],  # 左下圆形障碍
            [37, 7, 3],  # 右下圆形障碍
            [37, 23, 3]  # 右上方圆形障碍
        ]

        return obs_cir
