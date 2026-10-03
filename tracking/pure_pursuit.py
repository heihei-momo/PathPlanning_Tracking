"""
路径跟踪 - Pure Pursuit 纯跟踪
"""

import math  # 数学函数
import signal  # 信号处理
import sys  # 进程退出
import numpy as np  # 数组运算

from env import Env  # 环境与参考轨迹
from utils import Utils  # 跟踪通用工具
from plotting import Plotting  # 可视化工具


LD = 4.0  # 前视距离 [m]


class PurePursuit:  # 纯跟踪控制器
    def __init__(self, env, utils, ld=LD):  # 绑定环境与工具
        self.env = env                      # 环境对象
        self.u = utils                      # 工具对象
        self.ld = ld                        # 前视距离 [m]
        self.v = env.v_ref                  # 恒定目标车速 [m/s]
        self.delta_max = env.delta_max      # 前轮最大转角 [rad]
        self.max_steps = 1500               # 最大仿真步数

    def control(self, state):  # 由当前位姿算控制量
        """Pure Pursuit 控制律，返回 (前轮转角, 前视点 x, 前视点 y)"""
        px, py = self.u.lookahead_point(state[0], state[1], self.ld)   # 沿轨迹取前视点
        dx, dy = px - state[0], py - state[1]                          # 车辆到前视点向量
        bearing = math.atan2(dy, dx)                                   # 前视点方位角
        alpha = self.u.normalize_angle(bearing - state[2])             # 前视点方位角偏差
        delta = math.atan2(2.0 * self.env.L * math.sin(alpha), self.ld)   # 纯跟踪转角公式
        delta = max(-self.delta_max, min(self.delta_max, delta))       # 前轮转角限幅
        return delta, px, py                                           # 返回转角与前视点

    def step(self, state):  # 积分一步得到新位姿
        """用 Pure Pursuit 控制将自行车模型积分一步"""
        delta, px, py = self.control(state)                            # 计算前轮转角
        nxt = self.u.bicycle_step(state, self.v, delta, self.env.L, self.env.dt)   # 模型积分一步
        return nxt, delta, px, py                                      # 返回新位姿与控制


def main():  # 程序主入口
    env = Env()                                                      # 构造环境
    utils = Utils(env)                                               # 构造工具
    plot = Plotting(env, utils)                                      # 构造绘图器
    plot.init_figure("Pure Pursuit")                                 # 初始化画布
    pp = PurePursuit(env, utils, LD)                                 # 构造纯跟踪器
    state = env.start.copy()                                         # 当前车辆位姿
    xs, ys = [state[0]], [state[1]]                                  # 实际行驶轨迹坐标
    t_hist, ey_hist = [], []                                         # 时间与横向误差序列
    reached = False                                                  # 是否到达终点标志
    max_ey = 0.0                                                     # 最大横向误差
    sum_ey2 = 0.0                                                    # 横向误差平方和
    steps = 0                                                        # 已执行步数
    for _ in range(pp.max_steps):                                    # 逐步跟踪主循环
        state, _, px, py = pp.step(state)                            # 纯跟踪积分一步
        steps += 1                                                   # 累计仿真步数
        ey, _, _ = utils.lateral_error(state)                        # 当前横向误差
        xs.append(state[0])                                          # 记录 x 坐标
        ys.append(state[1])                                          # 记录 y 坐标
        t_hist.append(steps * env.dt)                                # 记录当前时间
        ey_hist.append(ey)                                           # 记录横向误差
        max_ey = max(max_ey, abs(ey))                                # 更新最大误差
        sum_ey2 += ey * ey                                           # 累加误差平方
        plot.clear()                                                 # 清除上一帧图元
        ang = np.linspace(0.0, 2.0 * np.pi, 72)                      # 前视圆角度采样
        cx = state[0] + pp.ld * np.cos(ang)                          # 前视圆 x 坐标
        cy = state[1] + pp.ld * np.sin(ang)                          # 前视圆 y 坐标
        plot.draw_line(cx, cy, color='0.7', lw=1.0, ls=':', alpha=0.7)      # 画前视圆
        plot.draw_line(xs, ys, color='tab:red', lw=2.0)              # 画实际行驶轨迹
        plot.draw_line([state[0], px], [state[1], py], color='0.5',  # 画前视连线
                       lw=1.0, ls='--', alpha=0.7)                   # 灰色虚线
        plot.draw_points([[px, py]], color='tab:green', size=12.0)   # 画前视点
        plot.draw_robot(state[0], state[1], state[2])                # 画车辆位姿
        plot.draw_error(t_hist, ey_hist)                             # 画横向误差曲线
        plot.refresh(pause=0.02)                                     # 刷新形成动画
        if utils.path_remaining(state[0], state[1]) < 2.0:           # 到达终点判定
            reached = True                                           # 置位到达标志
            break                                                    # 结束主循环
    elapsed = steps * env.dt                                         # 统计跟踪总时长
    rms_ey = math.sqrt(sum_ey2 / max(steps, 1))                      # 横向误差均方根
    print(f"Reached end: {reached}  Max |e_y|: {max_ey:.3f} m  "    # 打印跟踪统计结果
          f"RMS e_y: {rms_ey:.3f} m  Steps: {steps}  Time: {elapsed:.1f}s")   # 误差与耗时统计
    plot.hold()                                                      # 保持窗口直到 Ctrl+C


if __name__ == '__main__':  # 脚本直接运行时
    signal.signal(signal.SIGINT, signal.default_int_handler)   # 确保 Ctrl+C 一定有效
    try:                                                    # 捕获用户中断
        main()                                              # 调用入口函数
    except KeyboardInterrupt:                               # 用户按下 Ctrl+C
        print("\nInterrupted by Ctrl+C, exited.")           # 英文提示
        sys.exit(0)                                         # 正常退出进程
