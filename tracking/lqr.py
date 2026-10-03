"""
路径跟踪 - LQR 线性二次型调节器
"""

import math                                          # 三角函数与前馈转角
import signal                                        # 处理中断信号
import sys                                           # 进程正常退出
import time                                          # 统计运行耗时
import numpy as np                                   # 数值与矩阵运算
from scipy.linalg import solve_continuous_are        # 求解代数黎卡提方程

from env import Env                                  # 环境与参考轨迹
from utils import Utils                              # 几何与运动学工具
from plotting import Plotting                        # 动画可视化工具


class LQR:                                           # LQR 路径跟踪器
    def __init__(self, env, utils):                  # 初始化误差模型
        self.env = env                               # 环境对象引用
        self.u = utils                               # 通用工具引用
        self.v = env.v_ref                           # 恒定巡航车速
        self.L = env.L                               # 车辆轴距
        self.dt = env.dt                             # 仿真积分步长
        self.delta_max = env.delta_max               # 前轮最大转角
        self.path = env.path                         # 参考轨迹数组
        self.max_steps = 1500                        # 最大仿真步数
        self.goal_tol = 2.0                          # 终点剩余弧长阈值

        self.A = np.array([[0.0, self.v],            # 误差模型状态矩阵
                           [0.0, 0.0]])              # 第二行全为零
        self.B = np.array([[0.0],                    # 误差模型输入矩阵
                           [self.v / self.L]])       # 航向对转角的灵敏度
        self.Q = np.diag([20.0, 8.0])                # 误差状态权重矩阵
        self.R = np.array([[1.0]])                   # 前轮转角权重矩阵
        P = solve_continuous_are(self.A, self.B, self.Q, self.R)   # 求解代数黎卡提方程
        self.K = np.linalg.solve(self.R, self.B.T @ P)   # 求解 LQR 反馈增益

    def control(self, state):                        # 计算跟踪控制量
        """按 LQR 反馈加曲率前馈计算前轮转角"""
        e_y, e_theta, i = self.u.lateral_error(state)   # 取误差与最近点
        kappa = self.path[i, 3]                      # 最近点参考曲率
        delta_ff = math.atan(self.L * kappa)         # 曲率前馈转角
        e = np.array([e_y, e_theta])                 # 误差状态向量
        delta = delta_ff - float(self.K @ e)         # 前馈加反馈转角
        delta = float(np.clip(delta, -self.delta_max, self.delta_max))   # 前轮转角限幅
        return delta, e_y, e_theta, i                # 返回转角与误差


def main():                                          # 脚本主入口
    env = Env()                                      # 构造环境
    utils = Utils(env)                               # 构造工具对象
    plot = Plotting(env, utils)                      # 构造绘图器
    plot.init_figure("LQR")                          # 初始化双面板画布
    lqr = LQR(env, utils)                            # 构造 LQR 跟踪器

    state = np.array(env.start, dtype=float)         # 机器人当前位姿
    traj = [state.copy()]                            # 实际走过的轨迹
    t_hist = []                                      # 时间序列
    ey_hist = []                                     # 横向误差序列
    t0 = time.time()                                 # 记录起始时刻
    reached = False                                  # 到达终点标志
    steps = 0                                        # 已执行步数

    for _ in range(lqr.max_steps):                   # 跟踪主循环
        delta, e_y, e_theta, i = lqr.control(state)  # 计算转角与跟踪误差
        state = utils.bicycle_step(state, lqr.v, delta, lqr.L, lqr.dt)   # 积分一步
        traj.append(state.copy())                    # 记录实际轨迹点
        steps += 1                                   # 步数累加
        t_hist.append(steps * lqr.dt)                # 记录当前时刻
        ey_hist.append(e_y)                          # 记录横向误差
        xr, yr = env.path[i, 0], env.path[i, 1]      # 最近参考点坐标

        plot.clear()                                 # 清除上一帧图元
        plot.draw_line([p[0] for p in traj],         # 实际轨迹横坐标
                       [p[1] for p in traj],         # 实际轨迹纵坐标
                       color='tab:red', lw=2.0)      # 红色实线
        plot.draw_line([xr, state[0]],               # 跟踪误差矢量横坐标
                       [yr, state[1]],               # 跟踪误差矢量纵坐标
                       color='tab:purple', lw=1.8)   # 紫色醒目短线
        plot.draw_points(np.array([[xr, yr]]),       # 画最近参考点
                         color='tab:purple', size=6.0)   # 紫色小圆点
        plot.draw_robot(state[0], state[1], state[2], color='tab:orange')   # 画机器人
        plot.draw_error(t_hist, ey_hist)             # 画横向误差曲线
        plot.refresh(pause=0.02)                     # 刷新形成动画

        if utils.path_remaining(state[0], state[1]) < lqr.goal_tol:   # 到达终点判定
            reached = True                           # 置位到达标志
            break                                    # 退出跟踪循环

    elapsed = time.time() - t0                       # 总运行耗时
    ey = np.array(ey_hist)                           # 转成误差数组
    max_ey = float(np.abs(ey).max())                 # 最大横向误差
    rms_ey = float(np.sqrt(np.mean(ey ** 2)))        # 均方根横向误差
    k1, k2 = lqr.K[0]                                # 取出两个反馈增益
    print(f"LQR gain K = [{k1:.3f}, {k2:.3f}]")      # 打印 LQR 增益
    print(f"Reached end: {reached}  Max |e_y|: {max_ey:.3f} m  "   # 打印统计第一段
          f"RMS e_y: {rms_ey:.3f} m  Steps: {steps}  Time: {elapsed:.1f}s")   # 统计第二段
    plot.hold()                                      # 保持窗口直到 Ctrl+C


if __name__ == '__main__':  # 脚本直接运行时
    signal.signal(signal.SIGINT, signal.default_int_handler)   # 确保 Ctrl+C 一定有效
    try:                                                    # 捕获用户中断
        main()                                              # 调用入口函数
    except KeyboardInterrupt:                               # 用户按下 Ctrl+C
        print("\nInterrupted by Ctrl+C, exited.")           # 英文提示
        sys.exit(0)                                         # 正常退出进程
