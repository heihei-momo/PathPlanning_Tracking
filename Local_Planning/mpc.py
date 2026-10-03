"""
局部路径规划 - MPC 模型预测控制
"""

import time                                             # 统计求解与总耗时
import signal                                           # 信号处理
import sys                                              # 进程退出
import numpy as np                                      # 数值计算
from scipy.optimize import minimize                     # SLSQP 非线性优化器

from env import Env                                     # 环境与参考路径
from utils import Utils                                 # 碰撞检测与运动学工具
from plotting import Plotting                           # 动画可视化工具


class MPC:                                              # MPC 局部路径规划器
    def __init__(self, env, utils):                     # 初始化权重与路径缓存
        self.env = env                                  # 场地与障碍信息
        self.u = utils                                  # 工具对象引用
        self.N = 15                                     # 预测时域步数
        self.dt = 0.2                                   # 控制周期 [s]
        self.v_ref = 2.0                                # 参考巡航速度 [m/s]
        self.v_max = 3.0                                # 速度上限 [m/s]
        self.w_max = 2.0                                # 角速度上限 [rad/s]
        self.d_safe = 1.3                               # 避障安全距离 [m]

        self.w_xy = 8.0                                 # 位置跟踪误差权重
        self.w_theta = 1.5                              # 航向跟踪误差权重
        self.w_prog = 60.0                              # 纵向进度跟踪权重
        self.w_term = 4.0                               # 末端位置误差附加权重
        self.w_v = 0.05                                 # 速度幅值惩罚权重
        self.w_w = 0.20                                 # 角速度幅值惩罚权重
        self.w_dv = 0.80                                # 速度增量平滑权重
        self.w_dw = 1.00                                # 角速度增量平滑权重
        self.w_obs = 200.0                              # 避障惩罚权重

        self.max_iter = 50                              # SLSQP 最大迭代次数
        self.max_steps = 800                            # 最大控制步数
        self.goal_tol = 1.0                             # 到达终点的距离阈值

        self.path_x = env.path[:, 0]                    # 参考路径横坐标
        self.path_y = env.path[:, 1]                    # 参考路径纵坐标
        self.path_s = utils.path_lengths()              # 参考路径累计弧长
        self.prev_ctrl = np.zeros(2)                    # 上一周期施加的控制

        self.bounds = [(0.0, self.v_max)] * self.N      # 速度上下界
        self.bounds += [(-self.w_max, self.w_max)] * self.N   # 角速度上下界

    def build_reference(self, x, y):  # 生成时域参考位姿序列
        """构造预测时域内的参考位姿序列与进度目标弧长"""
        i, _ = self.u.nearest_on_path(x, y)             # 当前最近路径点
        s0 = self.path_s[i]                             # 当前参考弧长
        s_ref = s0 + self.v_ref * self.dt * np.arange(1, self.N + 1)   # 各步参考弧长
        s_ref = np.minimum(s_ref, self.path_s[-1])      # 弧长不超过路径末端
        px = np.interp(s_ref, self.path_s, self.path_x)           # 参考点横坐标
        py = np.interp(s_ref, self.path_s, self.path_y)           # 参考点纵坐标
        s_f = np.minimum(s_ref + 0.3, self.path_s[-1])  # 前向差分弧长
        s_b = np.maximum(s_ref - 0.3, 0.0)              # 后向差分弧长
        xf = np.interp(s_f, self.path_s, self.path_x)   # 前向采样点横坐标
        xb = np.interp(s_b, self.path_s, self.path_x)   # 后向采样点横坐标
        yf = np.interp(s_f, self.path_s, self.path_y)   # 前向采样点纵坐标
        yb = np.interp(s_b, self.path_s, self.path_y)   # 后向采样点纵坐标
        th = np.arctan2(yf - yb, xf - xb)               # 参考航向角
        s_tgt = min(s0 + self.v_ref * self.dt * self.N, self.path_s[-1])   # 时域末端目标弧长
        return np.column_stack([px, py, th]), s_tgt     # 返回参考序列与目标弧长

    def rollout_cost(self, z, state, ref, prev, s_tgt):  # 前向推演并累加代价
        """把控制序列 z 前向推演并累加代价，供 SLSQP 调用"""
        vs = z[:self.N]                                 # 拆出速度序列
        ws = z[self.N:]                                 # 拆出角速度序列
        st = state                                      # 从当前状态开始推演
        cost = 0.0                                      # 代价累加器
        ex = ey = 0.0                                   # 末端误差初始化
        for k in range(self.N):                         # 逐预测步推演
            st = self.u.motion(st, vs[k], ws[k], self.dt)         # 单车模型积分一步
            ex = st[0] - ref[k, 0]                      # 横向位置误差
            ey = st[1] - ref[k, 1]                      # 纵向位置误差
            cost += self.w_xy * (ex * ex + ey * ey)     # 位置跟踪误差
            eth = self.u.angle_diff(st[2], ref[k, 2])   # 航向跟踪误差
            cost += self.w_theta * eth * eth            # 航向跟踪误差
            d = self.u.obstacle_distance(st[0], st[1])  # 预测点到障碍的距离
            if d < self.d_safe:                         # 进入安全距离才惩罚
                cost += self.w_obs * (self.d_safe - d) ** 2         # 避障惩罚
        if s_tgt is not None:                           # 有进度目标时加进度项
            i, _ = self.u.nearest_on_path(st[0], st[1])  # 末端点的路径投影
            cost += self.w_prog * (s_tgt - self.path_s[i]) ** 2     # 纵向进度误差
        cost += self.w_term * (ex * ex + ey * ey)       # 末端位置误差加权
        cost += self.w_v * float(np.sum(vs * vs))       # 速度幅值惩罚
        cost += self.w_w * float(np.sum(ws * ws))       # 角速度幅值惩罚
        dv = np.diff(np.r_[prev[0], vs])                # 相对上一周期的速度增量
        dw = np.diff(np.r_[prev[1], ws])                # 角速度增量序列
        cost += self.w_dv * float(np.sum(dv * dv))      # 速度平滑惩罚
        cost += self.w_dw * float(np.sum(dw * dw))      # 角速度平滑惩罚
        return cost                                     # 返回总代价

    def initial_guess(self):  # 生成首个周期的控制初值
        """第一个控制周期的初值：匀速直行"""
        return np.r_[np.full(self.N, self.v_ref),       # 速度全取参考速度
                     np.zeros(self.N)]                  # 角速度全取零

    def warm_start(self, z):  # 上周期解左移作为热启动
        """把上一周期最优解左移一位，末位重复，作为热启动"""
        vs = np.r_[z[1:self.N], z[self.N - 1]]          # 速度序列左移
        ws = np.r_[z[self.N + 1:], z[-1]]               # 角速度序列左移
        return np.r_[vs, ws]                            # 拼成新初值

    def solve(self, state, ref, z0, s_tgt):  # 求解单周期优化问题
        """求解一个控制周期的有限时域优化问题"""
        res = minimize(self.rollout_cost, z0,                 # 目标函数与初值
                       args=(state, ref, self.prev_ctrl, s_tgt),   # 附加参数
                       method='SLSQP', bounds=self.bounds,    # 优化器与变量界
                       options={'maxiter': self.max_iter, 'ftol': 1e-4})   # 迭代设置
        z = np.asarray(res.x, dtype=float)              # 取优化器返回的最优解
        if not np.all(np.isfinite(z)):                  # 数值异常时回退初值
            z = np.array(z0, dtype=float)               # 使用热启动初值兜底
        return np.clip(z, [b[0] for b in self.bounds],  # 速度分量裁剪到界内
                       [b[1] for b in self.bounds])     # 角速度分量裁剪到界内

    def predict(self, state, z):  # 推演预测轨迹用于绘图
        """用给定控制序列推演预测轨迹 (N+1, 3)，用于可视化"""
        traj = [np.array(state, dtype=float)]           # 轨迹首元素为当前状态
        for k in range(self.N):                         # 逐步前向推演
            traj.append(self.u.motion(traj[-1], z[k], z[self.N + k], self.dt))   # 追加预测点
        return np.array(traj)                           # 转成数组返回

    def run(self):  # 滚动执行 MPC 主循环
        """滚动执行 MPC，返回是否到达、是否碰撞与总步数"""
        env, u = self.env, self.u                       # 简写引用
        plot = Plotting(env, u)                         # 构造可视化对象
        self.plot = plot                                # 保存供主入口调用
        plot.init_figure("MPC - Model Predictive Control")   # 初始化静态画布

        state = np.array(env.start, dtype=float)        # 机器人当前位姿
        traj = [state.copy()]                           # 实际走过的轨迹
        z = self.initial_guess()                        # 控制序列初值
        reached = False                                 # 到达标志
        collided = False                                # 碰撞标志
        steps = 0                                       # 已执行步数
        solve_times = []                                # 记录每步求解耗时

        for _ in range(self.max_steps):                 # 滚动控制主循环
            ref, s_tgt = self.build_reference(state[0], state[1])   # 参考点与目标弧长
            t1 = time.time()                            # 开始计时
            z = self.solve(state, ref, z, s_tgt)        # 求解有限时域优化
            solve_times.append(time.time() - t1)        # 记录单次求解耗时
            pred = self.predict(state, z)               # 用最优序列推演预测轨迹
            v, w = float(z[0]), float(z[self.N])        # 取最优序列的第一步控制
            z = self.warm_start(z)                      # 解序列左移作为下步初值
            prev_state = state.copy()                   # 保存推演前位姿
            state = u.motion(state, v, w, self.dt)      # 只施加第一步控制
            self.prev_ctrl = np.array([v, w])           # 记录本周期施加的控制
            traj.append(state.copy())                   # 记录实际轨迹点
            steps += 1                                  # 步数累加

            if u.is_segment_collision(prev_state, state, u.robot_radius):   # 本步发生碰撞
                collided = True                         # 置位碰撞标志
            if u.obstacle_distance(state[0], state[1]) < 0.0:               # 位姿落入障碍
                collided = True                         # 置位碰撞标志

            plot.clear()                                # 清除上一帧动态图元
            plot.draw_line(pred[:, 0], pred[:, 1],      # 画预测时域轨迹
                           color='tab:blue', lw=1.6, alpha=0.6)             # 细蓝线
            plot.draw_points(ref[:, :2], color='tab:purple', size=5.0)      # 画当前参考点
            plot.draw_line([p[0] for p in traj],        # 画实际走过的轨迹
                           [p[1] for p in traj], color='tab:red', lw=2.0)   # 红色实线
            plot.draw_robot(state[0], state[1], state[2], color='tab:orange')   # 画机器人
            plot.refresh(pause=0.01)                    # 刷新画布

            if u.dist(state, env.goal) < self.goal_tol:  # 到达终点判定
                reached = True                          # 置位到达标志
                break                                   # 结束滚动循环

        self.solve_times = solve_times                  # 保存求解耗时序列
        return reached, collided, steps                 # 返回自测统计量


def main():  # 脚本主入口
    env = Env()                                         # 构造环境
    utils = Utils(env)                                  # 构造工具对象
    mpc = MPC(env, utils)                               # 构造 MPC 规划器
    t0 = time.time()                                    # 记录总开始时间
    reached, collided, steps = mpc.run()                # 滚动执行 MPC
    print(f"Reached: {reached}  Collided: {collided}  Steps: {steps}  Time: {time.time()-t0:.1f}s")
    if len(mpc.solve_times) > 0:                        # 有求解记录时打印耗时
        t = np.array(mpc.solve_times) * 1000.0          # 转成毫秒
        print(f"Solve time: avg {t.mean():.1f} ms  max {t.max():.1f} ms  "   # 求解耗时统计
              f"total {t.sum() / 1000.0:.1f} s")       # 打印总优化时间
    mpc.plot.hold()                                 # 保持窗口直到 Ctrl+C


if __name__ == '__main__':  # 直接运行本文件时
    signal.signal(signal.SIGINT, signal.default_int_handler)   # 确保 Ctrl+C 一定有效
    try:                                        # 捕获用户中断
        main()                                  # 调用主入口
    except KeyboardInterrupt:                   # 用户按下 Ctrl+C
        print("\nInterrupted by Ctrl+C, exited.")   # 英文提示
        sys.exit(0)                             # 正常退出进程
