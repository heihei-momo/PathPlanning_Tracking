"""
路径跟踪 - MPC 模型预测控制
"""

import math                                          # 三角函数与前馈转角
import signal                                        # 处理中断信号
import sys                                           # 进程正常退出
import time                                          # 统计运行耗时
import warnings                                      # 屏蔽求解器边界提示
import numpy as np                                   # 数值与矩阵运算
from scipy.linalg import solve_continuous_are        # 求解代数黎卡提方程
from scipy.optimize import minimize                  # SLSQP 滚动优化求解器

from env import Env                                  # 环境与参考轨迹
from utils import Utils                              # 几何与运动学工具
from plotting import Plotting                        # 动画可视化工具

warnings.filterwarnings('ignore', message='Values in x were outside bounds')   # 忽略裁剪提示


class MPCTracker:                                    # MPC 路径跟踪器
    def __init__(self, env, utils, N=20):            # 初始化误差模型与终端代价
        self.env = env                               # 环境对象引用
        self.u = utils                               # 通用工具引用
        self.N = int(N)                              # 预测时域步数
        self.v = env.v_ref                           # 恒定巡航车速
        self.L = env.L                               # 车辆轴距
        self.dt = env.dt                             # 仿真积分步长
        self.delta_max = env.delta_max               # 前轮最大转角
        self.path = env.path                         # 参考轨迹数组
        self.Q = np.diag([20.0, 8.0])                # 横向与航向误差权重
        self.R = 1.0                                 # 转角偏差控制权重
        self.Rd = 5.0                                # 转角增量的平滑权重
        self.max_steps = 1500                        # 最大仿真步数
        self.goal_tol = 2.0                          # 终点剩余弧长阈值
        A = np.array([[0.0, self.v],                 # 误差模型状态矩阵
                      [0.0, 0.0]])                   # 第二行全为零
        B = np.array([[0.0],                         # 误差模型输入矩阵
                      [self.v / self.L]])            # 航向对偏差的灵敏度
        self.Ad = np.eye(2) + A * self.dt            # 前向欧拉离散化
        self.Bd = B.ravel() * self.dt                # 输入的离散化矩阵
        P = solve_continuous_are(A, B, self.Q, np.array([[1.0]]))   # 终端权重
        self.P = np.real(P)                          # 取实部作为终端代价
        self.u_last = 0.0                            # 上一周期施加的偏差
        self.U = np.zeros(self.N)                    # 热启动的初始解序列
        self.pred_x = np.zeros(self.N + 1)           # 预测轨迹横坐标缓冲
        self.pred_y = np.zeros(self.N + 1)           # 预测轨迹纵坐标缓冲
        self.build_cost()                            # 预计算代价二次型

    def build_cost(self):                            # 预计算预测矩阵与二次代价
        """把有限时域代价写成关于 U 的二次型，便于解析梯度快速求解"""
        N, Ad, Bd = self.N, self.Ad, self.Bd         # 取出常用量
        M = np.zeros((2 * N, 2))                     # 初始误差到各步的映射
        K = np.zeros((2 * N, N))                     # 控制序列到各步的映射
        Ak = np.eye(2)                               # Ad 的幂次累乘器
        for k in range(1, N + 1):                    # 逐预测步推进
            Ak = Ad @ Ak                             # 得到 Ad 的 k 次幂
            M[2 * (k - 1):2 * k, :] = Ak             # 填入初始误差的影响
            Pk = Bd                                  # 本步对 u_{k-1} 的影响
            K[2 * (k - 1):2 * k, k - 1] = Pk         # 写入最近一步的输入
            for j in range(k - 2, -1, -1):           # 反推更早输入的传播
                Pk = Ad @ Pk                         # 再乘一次状态矩阵
                K[2 * (k - 1):2 * k, j] = Pk         # 写入该输入的贡献
        G = np.kron(np.eye(N), self.Q)               # 各步状态代价块对角
        G[-2:, -2:] = self.P                         # 末步换成终端代价
        D = np.eye(N) - np.eye(N, k=1)               # 相邻转角差分矩阵
        H = K.T @ G @ K + self.R * np.eye(N)         # 二次项：状态加控制
        H = H + self.Rd * (D.T @ D)                  # 叠加增量平滑二次项
        self.M, self.K = M, K                        # 保存预测映射矩阵
        self.H = 0.5 * (H + H.T)                     # 强制对称便于求梯度
        self.KGM = K.T @ G @ M                       # 初始误差的线性映射
        self.e0 = np.r_[1.0, np.zeros(N - 1)]        # 首步单位向量

    def cost(self, U, f):                            # 二次型目标函数
        """有限时域代价：状态代价加控制代价与增量平滑"""
        return float(U @ (self.H @ U) + 2.0 * f @ U)   # 二次型快速求值

    def grad(self, U, f):                            # 目标函数的解析梯度
        """目标函数对决策变量的解析梯度"""
        return 2.0 * (self.H @ U + f)                # 二次型梯度公式

    def solve(self, x0):                             # 滚动优化求偏差序列
        """用 SLSQP 求解有限时域最优转角偏差序列"""
        lo, hi = -self.delta_max, self.delta_max     # 转角极限对应的偏差
        bounds = [(lo, hi)] * self.N                 # 决策变量上下界约束
        f = self.KGM @ x0 - self.Rd * self.u_last * self.e0   # 线性项系数
        U0 = np.clip(self.U, lo, hi)                 # 热启动初值先满足界
        res = minimize(self.cost, U0, args=(f,),     # 从热启动点开始优化
                       jac=self.grad,                # 使用解析梯度加速
                       method='SLSQP',               # 序列二次规划算法
                       bounds=bounds,                # 转角幅值硬约束
                       options={'maxiter': 40, 'ftol': 1e-4})   # 迭代与收敛
        U = np.asarray(res.x, dtype=float)           # 取出最优偏差序列
        self.U = np.r_[U[1:], U[-1]]                 # 左移一位便于下轮热启动
        self.u_last = float(U[0])                    # 记住本周期施加的偏差
        return U                                     # 返回最优偏差序列

    def rollout(self, state, delta_ff, U):           # 推演预测时域轨迹
        """用自行车模型推演未来 N 步，供俯视图画预测轨迹"""
        s = np.array(state, dtype=float)             # 从当前位姿出发
        xs, ys = [s[0]], [s[1]]                      # 预测轨迹的首点
        for k in range(self.N):                      # 逐步推演整个时域
            delta = delta_ff + float(U[k])           # 该步的前轮转角
            delta = float(np.clip(delta, -self.delta_max, self.delta_max))   # 转角再次限幅
            s = self.u.bicycle_step(s, self.v, delta, self.L, self.dt)   # 模型一步
            xs.append(s[0])                          # 记录预测横坐标
            ys.append(s[1])                          # 记录预测纵坐标
        self.pred_x = np.array(xs)                   # 保存预测轨迹横坐标
        self.pred_y = np.array(ys)                   # 保存预测轨迹纵坐标

    def control(self, state):                        # 一个控制周期的完整计算
        """做一次滚动优化并推演，返回前轮转角与当前跟踪误差"""
        e_y, e_theta, i = self.u.lateral_error(state)   # 当前横向与航向误差
        kappa = self.path[i, 3]                      # 最近点参考曲率
        delta_ff = math.atan(self.L * kappa)         # 曲率前馈转角
        x0 = np.array([e_y, e_theta])                # 误差状态向量
        U = self.solve(x0)                           # 求解最优偏差序列
        delta = delta_ff + float(U[0])               # 前馈加 MPC 第一步
        delta = float(np.clip(delta, -self.delta_max, self.delta_max))   # 转角限幅
        self.rollout(state, delta_ff, U)             # 推演预测时域轨迹
        return delta, e_y, e_theta, i                # 返回转角与误差


def main():                                          # 脚本主入口
    env = Env()                                      # 构造环境
    utils = Utils(env)                               # 构造工具对象
    plot = Plotting(env, utils)                      # 构造绘图器
    plot.init_figure("MPC tracking")                 # 初始化双面板画布
    mpc = MPCTracker(env, utils, N=20)               # 构造 MPC 跟踪器

    state = np.array(env.start, dtype=float)         # 机器人当前位姿
    traj = [state.copy()]                            # 实际走过的轨迹
    t_hist = []                                      # 时间序列
    ey_hist = []                                     # 横向误差序列
    solve_ms = []                                    # 单次求解耗时序列
    t0 = time.time()                                 # 记录起始时刻
    reached = False                                  # 到达终点标志
    steps = 0                                        # 已执行步数

    for _ in range(mpc.max_steps):                   # 跟踪主循环
        ts = time.time()                             # 本周期求解开始时刻
        delta, e_y, e_theta, i = mpc.control(state)  # 滚动优化得到转角
        solve_ms.append((time.time() - ts) * 1e3)    # 记录单次求解耗时
        state = utils.bicycle_step(state, mpc.v, delta, mpc.L, mpc.dt)   # 积分一步
        traj.append(state.copy())                    # 记录实际轨迹点
        steps += 1                                   # 步数累加
        t_hist.append(steps * mpc.dt)                # 记录当前时刻
        ey_hist.append(e_y)                          # 记录横向误差

        plot.clear()                                 # 清除上一帧图元
        plot.draw_line(mpc.pred_x, mpc.pred_y,       # 预测时域轨迹横坐标
                       color='tab:blue', lw=2.0,     # 醒目蓝色
                       ls='--', alpha=0.7)           # 半透明虚线
        pts = np.column_stack([mpc.pred_x, mpc.pred_y])[::2]   # 隔点取样
        plot.draw_points(pts, color='tab:blue',      # 预测轨迹上的离散点
                         size=4.0, alpha=0.7)        # 小点不遮挡主图
        plot.draw_line([p[0] for p in traj],         # 实际轨迹横坐标
                       [p[1] for p in traj],         # 实际轨迹纵坐标
                       color='tab:red', lw=2.0)      # 红色实线
        plot.draw_robot(state[0], state[1], state[2], color='tab:orange')   # 画机器人
        plot.draw_error(t_hist, ey_hist)             # 画横向误差曲线
        plot.refresh(pause=0.02)                     # 刷新形成动画

        if utils.path_remaining(state[0], state[1]) < mpc.goal_tol:   # 到达终点判定
            reached = True                           # 置位到达标志
            break                                    # 退出跟踪循环

    elapsed = time.time() - t0                       # 总运行耗时
    ey = np.array(ey_hist)                           # 转成误差数组
    max_ey = float(np.abs(ey).max())                 # 最大横向误差
    rms_ey = float(np.sqrt(np.mean(ey ** 2)))        # 均方根横向误差
    avg_ms = float(np.mean(solve_ms))                # 平均单次求解耗时
    max_ms = float(np.max(solve_ms))                 # 最大单次求解耗时
    print(f"Reached end: {reached}  Max |e_y|: {max_ey:.3f} m  "   # 打印统计第一段
          f"RMS e_y: {rms_ey:.3f} m  Steps: {steps}  Time: {elapsed:.1f}s")   # 统计第二段
    print(f"MPC solve time: avg {avg_ms:.1f} ms  max {max_ms:.1f} ms")   # 打印求解耗时
    plot.hold()                                      # 保持窗口直到 Ctrl+C


if __name__ == '__main__':  # 脚本直接运行时
    signal.signal(signal.SIGINT, signal.default_int_handler)   # 确保 Ctrl+C 一定有效
    try:                                                    # 捕获用户中断
        main()                                              # 调用入口函数
    except KeyboardInterrupt:                               # 用户按下 Ctrl+C
        print("\nInterrupted by Ctrl+C, exited.")           # 英文提示
        sys.exit(0)                                         # 正常退出进程
