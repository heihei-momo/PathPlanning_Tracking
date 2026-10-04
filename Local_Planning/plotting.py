"""
局部路径规划 - 可视化工具
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Plotting:  # 局部路径规划的统一绘图器
    def __init__(self, env, utils):  # 绑定环境与工具对象
        self.env = env                      # 环境对象
        self.utils = utils                  # 工具对象
        self.fig = None                     # 画布句柄
        self.ax = None                      # 坐标轴句柄
        self.artists = []                   # 动态图元列表，每帧清除

    def init_figure(self, name="Local Planning"):  # 创建画布并绘制静态环境
        """创建画布并画好静态环境（障碍、参考路径、起终点）"""
        plt.rcParams['figure.raise_window'] = False                     # 禁止窗口抢焦点
        self.fig, self.ax = plt.subplots(figsize=(12.8, 7.2))           # 新建 16:9 画布
        x0, x1 = self.env.x_range                                       # 场地 x 范围
        y0, y1 = self.env.y_range                                       # 场地 y 范围
        self.ax.set_xlim(x0 - 1.0, x1 + 1.0)                            # 留一点横向边距
        self.ax.set_ylim(y0 - 1.0, y1 + 1.0)                            # 留一点纵向边距
        self.ax.set_aspect('equal')                                     # 等比例，避免变形
        self.ax.set_title(name)                                         # 图标题
        self.ax.set_xlabel('x [m]')                                     # x 轴标签
        self.ax.set_ylabel('y [m]')                                     # y 轴标签
        self.ax.grid(True, linestyle=':', alpha=0.5)                    # 画淡网格

        self.ax.add_patch(patches.Rectangle((x0, y0), x1 - x0, y1 - y0,  # 场地外边界
                                            fill=False, edgecolor='k', linewidth=1.5))
        for (ox, oy, w, h) in self.env.obs_rect:                        # 逐个矩形障碍
            self.ax.add_patch(patches.Rectangle((ox, oy), w, h, facecolor='gray',
                                                edgecolor='k', alpha=0.85))
        for (ox, oy, r) in self.env.obs_circle:                         # 逐个圆形障碍
            self.ax.add_patch(patches.Circle((ox, oy), r, facecolor='gray',
                                             edgecolor='k', alpha=0.85))

        self.ax.plot(self.env.path[:, 0], self.env.path[:, 1], '--',    # 画参考路径
                     color='tab:blue', linewidth=2.0, label='Reference Path')
        self.ax.plot([self.env.start[0]], [self.env.start[1]], 'o',     # 画起点
                     color='tab:green', markersize=11, label='Start')
        self.ax.plot([self.env.goal[0]], [self.env.goal[1]], '*',       # 画终点
                     color='tab:red', markersize=18, label='Goal')
        self.ax.legend(loc='upper left', fontsize=9)                    # 显示图例
        self.bind_quit()                                                # 绑定退出按键
        plt.show(block=False)                                           # 只在此处显示一次窗口
        return self.fig, self.ax                                        # 返回画布与坐标轴

    def bind_quit(self):  # 绑定退出按键：ESC 或 Ctrl+C
        """按 ESC 或 Ctrl+C 直接关闭窗口"""
        def on_key(event):                                              # 键盘事件回调
            if event.key in ('escape', 'ctrl+c'):                       # 命中退出键
                print("\nWindow closed by user.")                       # 英文提示
                exit(0)                                                 # 直接退出进程
        self.fig.canvas.mpl_connect('key_release_event', on_key)         # 监听按键抬起事件

    def clear(self):  # 清除上一帧的动态图元
        """清除上一帧的动态图元"""
        for a in self.artists:                                          # 逐个移除图元
            try:                                                        # 有些图元可能已被移除
                a.remove()                                              # 从画布上摘掉
            except Exception:                                           # 忽略重复移除的报错
                pass                                                    # 直接跳过
        self.artists = []                                               # 清空图元列表

    def draw_robot(self, x, y, yaw, color='tab:red', length=1.7, width=0.9):  # 画机器人
        """画机器人：车身矩形 + 车头方向箭头"""
        body = np.array([[-length / 2, -width / 2],                     # 车身左后角（车体坐标）
                         [length / 2, -width / 2],                      # 车身右后角
                         [length / 2, width / 2],                       # 车身右前角
                         [-length / 2, width / 2]])                     # 车身左前角
        rot = np.array([[np.cos(yaw), -np.sin(yaw)],                    # 旋转矩阵第一行
                        [np.sin(yaw), np.cos(yaw)]])                    # 旋转矩阵第二行
        pts = body @ rot.T + np.array([x, y])                           # 车身转到世界坐标
        poly = patches.Polygon(pts, closed=True, facecolor=color,       # 构造车身多边形
                               edgecolor='k', linewidth=1.2, alpha=0.9, zorder=6)
        self.ax.add_patch(poly)                                         # 把车身加入画布
        self.artists.append(poly)                                       # 记入动态图元

        head = self.ax.plot([x, x + 1.3 * np.cos(yaw)],                 # 车头指向线的起点与终点
                            [y, y + 1.3 * np.sin(yaw)],                 # 车头指向线的 y 坐标
                            color='k', linewidth=2.0, zorder=7)[0]
        self.artists.append(head)                                       # 记入动态图元

    def draw_line(self, xs, ys, color='tab:red', lw=2.0, ls='-', alpha=1.0, zorder=4):  # 画折线
        """画一条折线，坐标可以是数组或列表"""
        line = self.ax.plot(xs, ys, color=color, linewidth=lw,          # 按给定样式画线
                            linestyle=ls, alpha=alpha, zorder=zorder)[0]
        self.artists.append(line)                                       # 记入动态图元
        return line                                                     # 返回线条对象

    def draw_points(self, pts, color='tab:orange', size=4.0, alpha=0.9):  # 画散点
        """画一串散点，pts 为 (N, 2) 数组"""
        pts = np.asarray(pts, dtype=float)                              # 统一转成数组
        dots = self.ax.plot(pts[:, 0], pts[:, 1], 'o', color=color,     # 逐个画出圆点
                            markersize=size, alpha=alpha, zorder=4)[0]
        self.artists.append(dots)                                       # 记入动态图元
        return dots                                                     # 返回散点对象

    def draw_text(self, x, y, text, color='k', size=10):  # 在指定位置写提示文字
        """在指定位置写一行提示文字"""
        t = self.ax.text(x, y, text, color=color, fontsize=size,        # 绘制文字对象
                         zorder=8, ha='left', va='top')
        self.artists.append(t)                                          # 记入动态图元
        return t                                                        # 返回文字对象

    def refresh(self, pause=0.02):  # 刷新画布并停顿，形成动画
        """刷新画布并短暂停顿；不用 plt.pause，避免窗口被反复抬到最前抢焦点"""
        self.fig.canvas.draw_idle()                                     # 请求重绘
        self.fig.canvas.flush_events()                                  # 处理 GUI 事件但不抬升窗口
        if pause > 0.0:                                                 # 需要停顿时
            time.sleep(pause)                                           # 用 sleep 控制动画节奏

    def hold(self):  # 跑完后保持窗口，等待用户退出
        """保持窗口不动，直到用户按 Ctrl+C（或窗口内 ESC / Ctrl+C）"""
        if self.fig is None:                                            # 没有画布就直接返回
            return                                                      # 无需保持
        print("Finished. Press Ctrl+C to exit "                         # 英文提示（第一段）
              "(or ESC / Ctrl+C in the window).", flush=True)           # 英文提示（第二段）
        while True:                                                     # 无限循环保持窗口
            self.fig.canvas.flush_events()                              # 处理 GUI 事件但不抢焦点
            time.sleep(0.1)                                             # 降低 CPU 占用
