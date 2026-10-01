"""
utils for collision check
@author: huiming zhou
"""

import math
import numpy as np
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env
from Sampling_based_Planning.rrt_2D.rrt import Node


class Utils:  # 碰撞检测工具类
    def __init__(self):  # 初始化环境与安全裕度
        self.env = env.Env()  # 环境实例：提供障碍信息

        self.delta = 0.5  # 碰撞检测安全裕度
        self.obs_circle = self.env.obs_circle  # 圆形障碍
        self.obs_rectangle = self.env.obs_rectangle  # 矩形障碍
        self.obs_boundary = self.env.obs_boundary  # 边界墙

    def update_obs(self, obs_cir, obs_bound, obs_rec):  # 更新障碍物集合
        self.obs_circle = obs_cir  # 更新圆形障碍
        self.obs_boundary = obs_bound  # 更新边界墙
        self.obs_rectangle = obs_rec  # 更新矩形障碍

    def get_obs_vertex(self):  # 矩形障碍外扩后的顶点
        delta = self.delta  # 安全裕度
        obs_list = []  # 各矩形的顶点列表

        for (ox, oy, w, h) in self.obs_rectangle:  # 遍历矩形障碍
            vertex_list = [[ox - delta, oy - delta],  # 左下顶点
                           [ox + w + delta, oy - delta],  # 右下顶点
                           [ox + w + delta, oy + h + delta],  # 右上顶点
                           [ox - delta, oy + h + delta]]  # 左上顶点
            obs_list.append(vertex_list)  # 保存该矩形四顶点

        return obs_list

    def is_intersect_rec(self, start, end, o, d, a, b):  # 射线与矩形边相交判断
        v1 = [o[0] - a[0], o[1] - a[1]]  # 边起点到射线起点
        v2 = [b[0] - a[0], b[1] - a[1]]  # 矩形边的方向向量
        v3 = [-d[1], d[0]]  # 射线方向的法向量

        div = np.dot(v2, v3)  # 分母：判断是否平行

        if div == 0:  # 平行则无交点
            return False  # 无交点

        t1 = np.linalg.norm(np.cross(v2, v1)) / div  # 射线参数 t1
        t2 = np.dot(v1, v3) / div  # 边参数 t2

        if t1 >= 0 and 0 <= t2 <= 1:  # 交点落在线段上
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))  # 构造交点节点
            dist_obs = self.get_dist(start, shot)  # 起点到交点距离
            dist_seg = self.get_dist(start, end)  # 待检线段长度
            if dist_obs <= dist_seg:  # 交点在待检线段内
                return True

        return False

    def is_intersect_circle(self, o, d, a, r):  # 射线与圆形障碍相交判断
        d2 = np.dot(d, d)  # 射线方向长度平方
        delta = self.delta  # 安全裕度

        if d2 == 0:  # 零长度射线
            return False  # 无交点

        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2  # 圆心在射线上的投影参数

        if 0 <= t <= 1:  # 投影点在线段上
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))  # 射线上离圆心最近点
            if self.get_dist(shot, Node(a)) <= r + delta:  # 最近点落在圆内
                return True

        return False

    def is_collision(self, start, end):  # 线段是否与障碍碰撞
        if self.is_inside_obs(start) or self.is_inside_obs(end):  # 端点已在障碍内
            return True

        o, d = self.get_ray(start, end)  # 射线起点与方向
        obs_vertex = self.get_obs_vertex()  # 矩形障碍外扩顶点

        for (v1, v2, v3, v4) in obs_vertex:  # 逐矩形检查四条边
            if self.is_intersect_rec(start, end, o, d, v1, v2):  # 检查边 v1v2
                return True  # 相交即碰撞
            if self.is_intersect_rec(start, end, o, d, v2, v3):  # 检查边 v2v3
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):  # 检查边 v3v4
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):  # 检查边 v4v1
                return True

        for (x, y, r) in self.obs_circle:  # 逐个圆形障碍检查
            if self.is_intersect_circle(o, d, [x, y], r):  # 与圆相交则碰撞
                return True

        return False

    def is_inside_obs(self, node):  # 节点是否落在障碍内
        delta = self.delta

        for (x, y, r) in self.obs_circle:  # 圆形障碍
            if math.hypot(node.x - x, node.y - y) <= r + delta:  # 到圆心距离小于半径
                return True

        for (x, y, w, h) in self.obs_rectangle:  # 矩形障碍
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:  # 节点在矩形障碍内
                return True

        for (x, y, w, h) in self.obs_boundary:  # 边界墙
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:  # 节点在边界墙内
                return True

        return False

    @staticmethod
    def get_ray(start, end):  # 求线段起点与方向
        orig = [start.x, start.y]  # 射线起点
        direc = [end.x - start.x, end.y - start.y]  # 射线方向向量
        return orig, direc

    @staticmethod
    def get_dist(start, end):  # 两点欧氏距离
        return math.hypot(end.x - start.x, end.y - start.y)
