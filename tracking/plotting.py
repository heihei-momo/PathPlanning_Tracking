"""
路径跟踪 - 可视化工具
左侧俯视图 + 右侧横向误差曲线
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Plotting:  # 路径跟踪的统一绘图器
    def __init__(self, env, utils):  # 绑定环境与工具对象
        self.env = env                      # 环境对象
        self.utils = utils                  # 工具对象
        self.fig = None                     # 画布句柄
        self.ax = None                      # 俯视图坐标轴
        self.ax_err = None                  # 误差图坐标轴
        self.artists = []                   # 俯视图动态图元列表

    def init_figure(self, name="Tracking"):  # 创建画布并绘制静态内容
        """创建双面板画布：左边俯视图，右边横向误差曲线"""
        plt.rcParams['figure.raise_window'] = False              # 禁止窗口抢焦点
        self.fig, (self.ax, self.ax_err) = plt.subplots(         # 一行两列布局
            1, 2, figsize=(14.5, 6.4),                           # 画布尺寸
            gridspec_kw={'width_ratios': [2.2, 1.0]})            # 左宽右窄
        self.fig.subplots_adjust(left=0.06, right=0.97,          # 调整左右留白
                                 top=0.92, bottom=0.10, wspace=0.22)   # 调整上下与间距

        ax = self.ax                                             # 俯视图简写
        x0, x1 = self.env.x_range                                # 场地 x 范围
        y0, y1 = self.env.y_range                                # 场地 y 范围
        ax.set_xlim(x0 - 2.0, x1 + 2.0)                          # x 轴范围
        ax.set_ylim(y0, y1)                                      # y 轴范围
        ax.set_aspect('equal')                                   # 等比例，避免变形
        ax.set_title(name)                                       # 图标题
        ax.set_xlabel('x [m]')                                   # x 轴标签
        ax.set_ylabel('y [m]')                                   # y 轴标签
        ax.grid(True, linestyle=':', alpha=0.5)                  # 淡网格

        ax.plot(self.env.path[:, 0], self.env.path[:, 1],        # 画参考轨迹
                '--', color='tab:blue', linewidth=2.0, label='Reference path')
        ax.plot([self.env.start[0]], [self.env.start[1]], 'o',   # 画起点
                color='tab:green', markersize=10, label='Start')
        ax.plot([self.env.path[-1, 0]], [self.env.path[-1, 1]],  # 画终点
                '*', color='tab:red', markersize=16, label='Goal')
        ax.legend(loc='upper left', fontsize=9)                  # 显示图例

        self.ax_err.set_title('Lateral error')                   # 误差图标题
        self.ax_err.set_xlabel('time [s]')                       # 误差图横轴
        self.ax_err.set_ylabel('error [m]')                      # 误差图纵轴
        self.ax_err.grid(True, linestyle=':', alpha=0.5)         # 淡网格
        self.ax_err.set_xlim(0.0, self.env.t_end)                # 横轴范围
        self.ax_err.set_ylim(-1.5, 1.5)                          # 纵轴范围
        self.ax_err.axhline(0.0, color='k', linewidth=0.6, alpha=0.4)   # 零误差参考线

        self.bind_quit()                                         # 绑定退出按键
        plt.show(block=False)                                    # 只在此处显示一次窗口
        return self.fig, self.ax                                 # 返回画布与坐标轴

    def bind_quit(self):  # 绑定退出按键：ESC 或 Ctrl+C
        """按 ESC 或 Ctrl+C 直接关闭窗口"""
        def on_key(event):                                              # 键盘事件回调
            if event.key in ('escape', 'ctrl+c'):                       # 命中退出键
                print("\nWindow closed by user.")                       # 英文提示
                exit(0)                                                 # 直接退出进程
        self.fig.canvas.mpl_connect('key_release_event', on_key)         # 监听按键抬起事件

    def clear(self):  # 清除俯视图上一帧的动态图元
        """清除俯视图上一帧的动态图元"""
        for a in self.artists:                                          # 逐个移除图元
            try:                                                        # 有些图元可能已被移除
                a.remove()                                              # 从画布上摘掉
            except Exception:                                           # 忽略重复移除的报错
                pass                                                    # 直接跳过
        self.artists = []                                               # 清空图元列表

    def draw_robot(self, x, y, theta, color='tab:orange'):  # 画车辆
        """画车辆：车身矩形 + 车头朝向线"""
        L, W = 4.0, 1.8                                                 # 车身长宽
        body = np.array([[-L / 2, -W / 2], [L / 2, -W / 2],             # 车身四角
                         [L / 2, W / 2], [-L / 2, W / 2]])              # 车体坐标
        rot = np.array([[np.cos(theta), -np.sin(theta)],                # 旋转矩阵
                        [np.sin(theta), np.cos(theta)]])                # 第二行
        pts = body @ rot.T + np.array([x, y])                           # 转到世界坐标
        poly = patches.Polygon(pts, closed=True, facecolor=color,       # 车身多边形
                               edgecolor='k', linewidth=1.2, alpha=0.9, zorder=6)
        self.ax.add_patch(poly)                                         # 加入画布
        self.artists.append(poly)                                       # 记入动态图元

        head = self.ax.plot([x, x + 2.6 * np.cos(theta)],               # 车头指向线
                            [y, y + 2.6 * np.sin(theta)],               # 纵坐标
                            color='k', linewidth=2.0, zorder=7)[0]      # 黑色实线
        self.artists.append(head)                                       # 记入动态图元

    def draw_line(self, xs, ys, color='tab:red', lw=2.0, ls='-', alpha=1.0, zorder=4):
        """在俯视图上画一条折线"""
        line = self.ax.plot(xs, ys, color=color, linewidth=lw,          # 绘制折线
                            linestyle=ls, alpha=alpha, zorder=zorder)[0]
        self.artists.append(line)                                       # 记入动态图元
        return line                                                     # 返回线条对象

    def draw_points(self, pts, color='tab:green', size=6.0, alpha=1.0):
        """在俯视图上画散点，pts 为 (N, 2)"""
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)               # 统一成 (N, 2)
        dots = self.ax.plot(pts[:, 0], pts[:, 1], 'o', color=color,     # 绘制散点
                            markersize=size, alpha=alpha, zorder=5)[0]
        self.artists.append(dots)                                       # 记入动态图元
        return dots                                                     # 返回散点对象

    def draw_error(self, t_hist, ey_hist):  # 画横向误差曲线
        """在右侧面板画横向误差随时间的曲线"""
        ax = self.ax_err                                                # 误差图坐标轴
        ax.clear()                                                      # 清空重画
        ax.set_title('Lateral error')                                   # 重新设置标题
        ax.set_xlabel('time [s]')                                       # 横轴标签
        ax.set_ylabel('error [m]')                                      # 纵轴标签
        ax.grid(True, linestyle=':', alpha=0.5)                         # 淡网格
        ax.axhline(0.0, color='k', linewidth=0.6, alpha=0.4)            # 零误差参考线
        ymax = max(1.0, float(np.abs(ey_hist).max()) * 1.25) if len(ey_hist) else 1.0   # 自适应纵轴
        ymax = min(ymax, 4.0)                                           # 纵轴不超过 4 m
        ax.set_ylim(-ymax, ymax)                                        # 设置纵轴范围
        ax.set_xlim(0.0, self.env.t_end)                                # 设置横轴范围
        if len(t_hist) > 0:                                             # 有数据才画曲线
            ax.plot(t_hist, ey_hist, color='tab:red', linewidth=1.6)    # 横向误差曲线
        return ax                                                       # 返回坐标轴

    def refresh(self, pause=0.02):  # 刷新画布并停顿
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
