"""
局部路径规划 - TEB 时间弹性带
"""

import math                                                        # 角度与三角函数
import time                                                        # 计时与动画节拍
import numpy as np                                                 # 数值数组运算
import signal                                                      # 信号处理
import sys                                                         # 进程退出
from scipy.optimize import minimize                                # SLSQP 非线性求解

from env import Env                                                # 环境定义
from utils import Utils                                            # 通用工具
from plotting import Plotting                                      # 可视化工具


class TEB:
    """时间弹性带局部规划器：优化一条可变形位姿带子"""

    def __init__(self, env, utils):
        self.env = env                                             # 保存环境对象
        self.utils = utils                                         # 保存工具对象
        self.n_band = 20                                           # 带子位姿个数
        self.ds_band = 0.9                                         # 相邻位姿名义间距
        self.d_safe = 1.2                                          # 避障安全距离
        self.v_max = 1.5                                           # 最大线速度
        self.w_max = 1.2                                           # 最大角速度
        self.dt_sim = 0.2                                          # 控制周期
        self.radius = utils.robot_radius                           # 机器人碰撞半径
        self.reopt_k = 5                                           # 重优化间隔步数
        self.max_steps = 800                                       # 最大控制步数
        self.maxiter = 22                                          # 首次优化迭代上限
        self.maxiter_fast = 6                                      # 滚动重优化的迭代上限
        self.w_path = 0.35                                         # 路径贴合权重
        self.w_obs = 55.0                                          # 障碍惩罚权重
        self.w_smooth = 10.0                                       # 平滑惩罚权重
        self.w_time = 0.2                                          # 时间最优权重
        self.w_vel = 4.0                                           # 速度上限权重
        self.w_omega = 1.0                                         # 角速度约束权重
        self.lookahead = 1.0                                       # 跟踪前瞻距离
        self.k_theta = 1.6                                         # 航向控制增益
        self.opt_time = 0.0                                        # 最近一次优化耗时
        self.path = np.asarray(self.utils.path, dtype=float)       # 参考路径数组
        self.px = self.path[:, 0].copy()                           # 路径 x 坐标列
        self.py = self.path[:, 1].copy()                           # 路径 y 坐标列
        self.s_path = self.utils.path_lengths()                    # 路径累计弧长

    def point_at_arc(self, s):
        """按弧长在参考路径上线性插值取点"""
        x = float(np.interp(s, self.s_path, self.px))              # 按弧长插值横坐标
        y = float(np.interp(s, self.s_path, self.py))              # 按弧长插值纵坐标
        return x, y                                                # 返回该点坐标

    def nominal_band(self, state):
        """沿参考路径等弧长采样出名义带子，末端避开障碍"""
        x, y = float(state[0]), float(state[1])                    # 机器人当前位置
        i0, _ = self.utils.nearest_on_path(x, y)                   # 最近路径点下标
        s0 = float(self.s_path[i0])                                # 机器人所在弧长
        s_end = min(s0 + (self.n_band - 1) * self.ds_band,         # 名义末端弧长
                    float(self.s_path[-1]))                        # 不超过终点
        while s_end > s0 + 0.5:                                    # 末端贴障碍就回退
            ex, ey = self.point_at_arc(s_end)                      # 末端点坐标
            if self.utils.obstacle_distance(ex, ey) >= self.d_safe:  # 已满足安全距离
                break                                              # 停止回退
            s_end -= 0.3                                           # 弧长后退一小步
        arcs = np.linspace(s0, s_end, self.n_band)                 # 等弧长采样位置
        bx = np.interp(arcs, self.s_path, self.px)                 # 采样点横坐标
        by = np.interp(arcs, self.s_path, self.py)                 # 采样点纵坐标
        band = np.column_stack([bx, by])                           # 拼成名义带子
        band[0] = (x, y)                                           # 首点锁定机器人
        return band                                                # 返回名义带子

    def init_dts(self, band):
        """按段长与最大速度给出时间间隔初值"""
        seg = np.hypot(np.diff(band[:, 0]), np.diff(band[:, 1]))   # 各段长度
        dts = np.clip(seg / self.v_max, 0.05, 1.0)                 # 段长比速度
        return dts                                                 # 返回时间间隔

    def band_cost(self, z, p_start, p_goal):
        """弹性带总代价，z 为决策向量"""
        nf = self.n_band - 2                                       # 自由位姿个数
        band = np.empty((self.n_band, 2))                          # 组装完整带子
        band[0] = p_start                                          # 固定起点
        band[1:-1, 0] = z[:nf]                                     # 中间点横坐标
        band[1:-1, 1] = z[nf:2 * nf]                               # 中间点纵坐标
        band[-1] = p_goal                                          # 固定终点
        dts = z[2 * nf:]                                           # 各段时间间隔
        dx = self.px[None, :] - band[:, 0:1]                       # 到路径点横差
        dy = self.py[None, :] - band[:, 1:2]                       # 到路径点纵差
        d2 = dx * dx + dy * dy                                     # 距离平方矩阵
        idx = np.argmin(d2, axis=1)                                # 最近路径点下标
        rows = np.arange(self.n_band)                              # 行号索引
        d_path = np.sqrt(d2[rows, idx])                            # 最近点距离
        cost = self.w_path * float(np.sum(d_path ** 2))            # 路径贴合代价
        d_obs = np.empty(self.n_band - 1)                          # 段中点障碍距离
        for i in range(self.n_band - 1):                           # 逐段取中点
            mx = 0.5 * (band[i, 0] + band[i + 1, 0])               # 中点横坐标
            my = 0.5 * (band[i, 1] + band[i + 1, 1])               # 中点纵坐标
            d_obs[i] = self.utils.obstacle_distance(mx, my)        # 记录障碍距离
        d_v = np.empty(nf)                                         # 自由点障碍距离
        for i in range(1, self.n_band - 1):                        # 遍历自由位姿
            d_v[i - 1] = self.utils.obstacle_distance(band[i, 0],  # 该点障碍距离
                                                      band[i, 1])  # 该点纵坐标
        d_all = np.r_[d_v, d_obs]                                  # 合并两类距离
        viol = np.maximum(0.0, self.d_safe - d_all)                # 安全距离缺口
        cost += self.w_obs * float(np.sum(viol ** 2))              # 障碍惩罚代价
        dd = band[2:] - 2.0 * band[1:-1] + band[:-2]               # 位置二阶差分
        cost += self.w_smooth * float(np.sum(dd * dd))             # 平滑惩罚代价
        cost += self.w_time * float(np.sum(dts))                   # 时间最优代价
        seg = np.hypot(np.diff(band[:, 0]), np.diff(band[:, 1]))   # 各段长度
        over = np.maximum(0.0, seg / dts - self.v_max)             # 超出速度上限的量
        cost += self.w_vel * float(np.sum(over ** 2))              # 速度上限代价
        th = np.arctan2(np.diff(band[:, 1]), np.diff(band[:, 0]))  # 各段航向角
        dth = np.diff(th)                                          # 相邻航向差
        dth = (dth + np.pi) % (2.0 * np.pi) - np.pi                # 归一化角度差
        dt_mid = 0.5 * (dts[1:] + dts[:-1])                        # 相邻段平均时间
        rate = np.abs(dth) / dt_mid                                # 航向角速度
        over_w = np.maximum(0.0, rate - self.w_max)                # 超角速度量
        cost += self.w_omega * float(np.sum(over_w ** 2))          # 角速度约束代价
        return cost                                                # 返回总代价

    def optimize(self, state, warm_band, warm_dts, maxiter=None):
        """用 SLSQP 优化弹性带，返回新带子与新时间"""
        t_start = time.time()                                      # 优化计时开始
        nf = self.n_band - 2                                       # 自由位姿个数
        p_start = np.array([state[0], state[1]], dtype=float)      # 固定起点坐标
        p_goal = warm_band[-1].copy()                              # 固定局部终点
        z0 = np.r_[warm_band[1:-1, 0], warm_band[1:-1, 1],         # 热启动初值
                   np.clip(warm_dts, 0.05, 1.0)]                   # 时间间隔初值
        lo = np.r_[np.full(nf, self.env.x_range[0]),               # 横坐标下界
                   np.full(nf, self.env.y_range[0]),               # 纵坐标下界
                   np.full(nf + 1, 0.05)]                          # 时间下界
        hi = np.r_[np.full(nf, self.env.x_range[1]),               # 横坐标上界
                   np.full(nf, self.env.y_range[1]),               # 纵坐标上界
                   np.full(nf + 1, 1.0)]                           # 时间上界
        res = minimize(self.band_cost, z0, args=(p_start, p_goal), # 调用 SLSQP 求解
                       method='SLSQP', bounds=list(zip(lo, hi)),   # 指定求解器与边界
                       options={'maxiter': maxiter or self.maxiter,   # 迭代次数上限
                                'ftol': 1e-4})                     # 收敛容差
        band = np.empty((self.n_band, 2))                          # 还原最优带子
        band[0] = p_start                                          # 固定起点
        band[1:-1, 0] = res.x[:nf]                                 # 最优中间点横坐标
        band[1:-1, 1] = res.x[nf:2 * nf]                           # 最优中间点纵坐标
        band[-1] = p_goal                                          # 固定局部终点
        dts = np.clip(res.x[2 * nf:], 0.05, 1.0)                   # 最优时间间隔
        self.opt_time = time.time() - t_start                      # 记录优化耗时
        return band, dts                                           # 返回优化结果

    def project_on_band(self, band, state):
        """把机器人投影到带子上，返回弧长坐标与顶点弧长"""
        seg = np.diff(band, axis=0)                                # 各段向量
        L = np.hypot(seg[:, 0], seg[:, 1])                         # 各段长度
        s = np.r_[0.0, np.cumsum(L)]                               # 顶点累计弧长
        p = np.array([state[0], state[1]], dtype=float)            # 机器人位置
        rel = p - band[:-1]                                        # 相对段起点
        t = np.sum(rel * seg, axis=1) / np.maximum(L ** 2, 1e-9)   # 投影参数
        t = np.clip(t, 0.0, 1.0)                                   # 限制在段内
        proj = band[:-1] + t[:, None] * seg                        # 各投影点
        d = np.hypot(proj[:, 0] - p[0], proj[:, 1] - p[1])         # 投影点距离
        k = int(np.argmin(d))                                      # 最近段下标
        return float(s[k] + t[k] * L[k]), s                        # 返回弧长信息

    def band_point_ahead(self, band, s_robot):
        """沿带子向前取前瞻距离处的跟踪目标点"""
        seg = np.diff(band, axis=0)                                # 各段向量
        L = np.hypot(seg[:, 0], seg[:, 1])                         # 各段长度
        s = np.r_[0.0, np.cumsum(L)]                               # 顶点累计弧长
        st = min(s_robot + self.lookahead, float(s[-1]))           # 目标弧长
        tx = float(np.interp(st, s, band[:, 0]))                   # 目标点横坐标
        ty = float(np.interp(st, s, band[:, 1]))                   # 目标点纵坐标
        return tx, ty                                              # 返回目标点

    def shift_band(self, band, dts, state):
        """把上一轮带子整体前移并补新点，作为热启动"""
        s_robot, s_band = self.project_on_band(band, state)        # 机器人弧长
        shift = int(np.searchsorted(s_band, s_robot))              # 已走过的格数
        shift = min(max(shift, 0), self.n_band - 2)                # 限制前移范围
        nominal = self.nominal_band(state)                         # 名义带子兜底
        new_band = nominal.copy()                                  # 复制名义带子
        new_dts = self.init_dts(nominal)                           # 默认时间初值
        for i in range(1, self.n_band - 1):                        # 逐点继承旧解
            j = i + shift                                          # 对应旧点下标
            if j < self.n_band - 1:                                # 旧点仍在带内
                new_band[i] = band[j]                              # 继承旧位姿
        for i in range(self.n_band - 1):                           # 逐段继承旧时间
            j = i + shift                                          # 对应旧段下标
            if j < self.n_band - 1:                                # 旧段仍在带内
                new_dts[i] = dts[j]                                # 继承旧时间
        new_band[0] = (state[0], state[1])                         # 首点锁定机器人
        new_band[-1] = nominal[-1]                                 # 末点用名义终点
        return new_band, new_dts                                   # 返回热启动带子

    def tracking_control(self, state, band):
        """用纯跟踪法求当前线速度与角速度"""
        s_robot, _ = self.project_on_band(band, state)             # 机器人弧长
        tx, ty = self.band_point_ahead(band, s_robot)              # 前瞻目标点
        yaw_ref = math.atan2(ty - state[1], tx - state[0])         # 期望航向角
        err = self.utils.angle_diff(yaw_ref, state[2])             # 航向误差
        w = float(np.clip(self.k_theta * err, -self.w_max,         # 角速度控制量
                          self.w_max))                             # 限制在角速度上限
        v = self.v_max * max(0.0, 1.0 - abs(err) / 1.2)            # 误差大就减速
        return v, w                                                # 返回控制量

    def brake(self, state, v, w):
        """若下一步会撞障碍就按比例减速，保证执行安全"""
        p = (state[0], state[1])                                   # 当前平面位置
        for k in (1.0, 0.6, 0.3, 0.1, 0.0):                        # 逐步缩小速度
            nxt = self.utils.motion(state, v * k, w, self.dt_sim)  # 试算下一步
            q = (nxt[0], nxt[1])                                   # 下一步位置
            if not self.utils.is_segment_collision(p, q, self.radius):  # 该段安全才采用
                return v * k                                       # 安全就采用
        return 0.0                                                 # 全不安全就停车

    def draw(self, plot, band, traj, state, steps, v):
        """刷新一帧动画：弹性带、实际轨迹、机器人"""
        plot.clear()                                               # 清除上一帧
        plot.draw_line(band[:, 0], band[:, 1], color='tab:orange',  # 画出弹性带折线
                       lw=1.5, ls='--', alpha=0.75)                # 虚线半透明
        plot.draw_points(band, color='tab:orange', size=5.0)       # 画出带子位姿点
        plot.draw_line(traj[:, 0], traj[:, 1], color='tab:red',    # 画出实际轨迹
                       lw=2.0)                                     # 红色实线
        plot.draw_robot(state[0], state[1], state[2],              # 画出机器人位姿
                        color='tab:red')                           # 红色车身
        plot.refresh(pause=0.02)                                   # 刷新画面

    def plan(self, plot=None):
        """滚动执行 TEB：反复优化带子并跟踪前进"""
        state = np.array(self.env.start, dtype=float)              # 机器人初始位姿
        traj = [state.copy()]                                      # 实际轨迹列表
        band = self.nominal_band(state)                            # 初始名义带子
        dts = self.init_dts(band)                                  # 初始时间间隔
        reached = False                                            # 到达终点标志
        collided = False                                           # 发生碰撞标志
        steps = 0                                                  # 已执行步数
        for step in range(self.max_steps):                         # 主控制循环
            steps = step + 1                                       # 更新步数
            if step % self.reopt_k == 0:                           # 到达重优化周期
                warm_band, warm_dts = self.shift_band(band, dts,   # 带子前移热启动
                                                      state)       # 传入当前位姿
                it = self.maxiter if step == 0 else self.maxiter_fast   # 首轮用满迭代，之后用快速档
                band, dts = self.optimize(state, warm_band,        # 重新优化带子
                                          warm_dts, it)            # 沿用热启动时间
            v, w = self.tracking_control(state, band)              # 求跟踪控制量
            v = self.brake(state, v, w)                            # 碰撞前减速处理
            state = self.utils.motion(state, v, w, self.dt_sim)    # 积分一步位姿
            traj.append(state.copy())                              # 记录新位姿
            p_prev = traj[-2][:2]                                  # 上一位置
            p_now = traj[-1][:2]                                   # 当前位置
            if self.utils.is_segment_collision(p_prev, p_now, self.radius):  # 检查轨迹碰撞
                collided = True                                    # 标记发生碰撞
            if plot is not None:                                   # 需要可视化时
                self.draw(plot, band, np.array(traj), state,       # 刷新一帧动画
                          steps, v)                                # 传入步数与速度
            if self.utils.dist(state, self.env.goal) < 1.0:        # 到达终点判定
                reached = True                                     # 标记到达终点
                break                                              # 退出主循环
        if plot is not None:                                       # 收尾补画一帧
            self.draw(plot, band, np.array(traj), state, steps, 0.0)  # 最终画面
        dist_goal = self.utils.dist(state, self.env.goal)          # 终点残余距离
        return reached, collided, steps, dist_goal, np.array(traj)  # 返回统计结果


def main():
    """构造环境、执行 TEB 滚动规划并输出收尾统计"""
    env = Env()                                                    # 构造环境
    utils = Utils(env)                                             # 构造工具
    teb = TEB(env, utils)                                          # 构造规划器
    plot = Plotting(env, utils)                                    # 构造绘图器
    plot.init_figure("TEB - Timed Elastic Band")                   # 初始化画布
    t0 = time.time()                                               # 总计时开始
    reached, collided, steps, dist_goal, traj = teb.plan(plot)     # 执行滚动规划
    print(f"Reached: {reached}  Collided: {collided}  Steps: {steps}  Time: {time.time()-t0:.1f}s")
    plot.hold()                                                    # 保持窗口直到 Ctrl+C
    return reached, collided, steps, dist_goal                     # 返回统计结果


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.default_int_handler)   # 确保 Ctrl+C 一定有效
    try:                                                    # 捕获用户中断
        main()                                              # 调用主入口
    except KeyboardInterrupt:                               # 用户按下 Ctrl+C
        print("\nInterrupted by Ctrl+C, exited.")           # 英文提示
        sys.exit(0)                                         # 正常退出进程
