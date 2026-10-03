"""
局部路径规划 - DWA 动态窗口法
"""

import math  # 数学函数
import signal  # 信号处理
import sys  # 进程退出
import time  # 计时工具
import numpy as np  # 数组运算

from env import Env  # 环境定义
from utils import Utils  # 通用工具
from plotting import Plotting  # 可视化工具


class DWA:  # 动态窗口法局部规划器
    def __init__(self, env, utils, plot=None):  # 构造规划器
        self.env = env                              # 环境对象
        self.u = utils                              # 工具对象
        self.plot = plot                            # 可视化对象
        self.dt = 0.1                               # 积分时间步长
        self.predict_time = 2.5                     # 轨迹推演时长
        self.v_min = 0.0                            # 最小线速度
        self.v_max = 2.0                            # 最大线速度
        self.w_max = 1.2                            # 最大角速度
        self.a_max = 2.0                            # 最大线加速度
        self.alpha_max = 6.0                        # 最大角加速度
        self.v_reso = 0.1                           # 线速度采样间隔
        self.w_reso = 0.05                          # 角速度采样间隔
        self.margin = utils.robot_radius            # 碰撞检测车身半径
        self.safe_gap = 0.3                         # 额外安全间隙
        self.clear_margin = self.margin + self.safe_gap  # 轨迹碰撞检测裕度
        self.clear_thresh = 1.2                     # 避障评分饱和距离
        self.w_heading = 0.5                        # 朝向项权重
        self.w_clear = 1.5                          # 避障项权重
        self.w_vel = 0.3                            # 速度项权重
        self.w_path = 0.4                           # 路径项权重
        self.w_prog = 5.0                           # 前进项权重
        self.lookahead = 3.0                        # 前瞻点距离
        self.lookahead_try = 16                     # 前瞻点外扩次数
        self.lookahead_step = 0.6                   # 每次外扩的弧长
        self.lookahead_margin = self.clear_margin   # 前瞻点安全余量
        self.goal_tol = 1.0                         # 到达终点的距离阈值
        self.max_steps = 800                        # 最大仿真步数
        self.top_k = 24                             # 可视化候选轨迹条数
        self.frame_skip = 1                         # 每隔几步画一帧

    def dynamic_window(self, v, w):  # 计算动态窗口
        """由当前速度与加速度限制算出下一步可达的速度区间"""
        v_lo = max(self.v_min, v - self.a_max * self.dt)        # 线速度下界
        v_hi = min(self.v_max, v + self.a_max * self.dt)        # 线速度上界
        w_lo = max(-self.w_max, w - self.alpha_max * self.dt)   # 角速度下界
        w_hi = min(self.w_max, w + self.alpha_max * self.dt)    # 角速度上界
        return v_lo, v_hi, w_lo, w_hi                           # 返回窗口范围

    def sample_controls(self, v, w):  # 在窗口内采样速度
        """在动态窗口内离散采样 (v, w) 组合"""
        v_lo, v_hi, w_lo, w_hi = self.dynamic_window(v, w)      # 取当前动态窗口
        vs = np.arange(v_lo, v_hi + 1e-6, self.v_reso)          # 采样线速度序列
        ws = np.arange(w_lo, w_hi + 1e-6, self.w_reso)          # 采样角速度序列
        if len(vs) == 0:                                        # 上界没被覆盖
            vs = np.array([v_lo])                               # 至少保留一个值
        if len(ws) == 0:                                        # 上界没被覆盖
            ws = np.array([w_lo])                               # 至少保留一个值
        return vs, ws                                           # 返回采样序列

    def safe_lookahead(self, x, y, base):  # 避障感知的前瞻点
        """沿参考路径向前搜索，返回第一个远离障碍的安全点"""
        d = base                                                # 从基础前瞻距离开始
        p = self.u.path_lookahead(x, y, d)                      # 先取一个前瞻点
        for _ in range(self.lookahead_try):                     # 最多向前试探若干次
            if self.u.obstacle_distance(p[0], p[1]) > self.lookahead_margin:  # 该点安全
                return p                                        # 直接返回该点
            d += self.lookahead_step                            # 否则继续向前找
            p = self.u.path_lookahead(x, y, d)                  # 更新前瞻点
        return p                                                # 兜底返回最后一点

    def evaluate(self, traj, v, rem_now):  # 给一条轨迹打分
        """按朝向、避障、速度、路径贴合、前进五项加权给轨迹打分"""
        end = traj[-1]                                          # 轨迹末端状态
        look = self.safe_lookahead(end[0], end[1],              # 末端对应的安全前瞻点
                                   self.lookahead)              # 基础前瞻距离
        ang = math.atan2(look[1] - end[1], look[0] - end[0])    # 末端到前瞻点方位角
        diff = abs(self.u.angle_diff(ang, end[2]))              # 与车头朝向的夹角
        heading = (math.pi - diff) / math.pi                    # 朝向项归一化得分
        clear = min(self.u.obstacle_distance(p[0], p[1])        # 轨迹上最小障碍距
                    for p in traj)                              # 遍历轨迹点取最小
        clear = max(0.0, min(clear / self.clear_thresh, 1.0))   # 避障项归一化得分
        vel = max(0.0, v) / self.v_max                          # 速度项归一化得分
        _, d = self.u.nearest_on_path(end[0], end[1])           # 末端到参考路径距离
        path = 1.0 / (1.0 + d)                                  # 路径项归一化得分
        rem_end = self.u.path_remaining(end[0], end[1])         # 末端剩余弧长
        span = self.v_max * self.predict_time                   # 一个周期的最大前进
        prog = (rem_now - rem_end) / span                       # 前进项，倒退为负
        score = (self.w_heading * heading +                     # 累加朝向项
                 self.w_clear * clear +                         # 累加避障项
                 self.w_vel * vel +                             # 累加速度项
                 self.w_path * path +                           # 累加路径项
                 self.w_prog * prog)                            # 累加前进项
        return score                                           # 返回加权总分

    def plan_once(self, state, v, w):  # 规划一步控制量
        """推演所有候选轨迹并返回最优控制"""
        steps = int(self.predict_time / self.dt)                # 推演步数
        vs, ws = self.sample_controls(v, w)                     # 采样速度组合
        target = self.safe_lookahead(state[0], state[1],        # 避障感知的前瞻目标
                                     self.lookahead)            # 基础前瞻距离
        rem_now = self.u.path_remaining(state[0], state[1])     # 当前剩余弧长
        cands = []                                              # 候选轨迹列表
        for cv in vs:                                           # 遍历线速度
            for cw in ws:                                       # 遍历角速度
                traj = self.u.simulate(state, cv, cw,           # 推演候选轨迹
                                       self.dt, steps)          # 积分步数与步长
                if self.u.is_trajectory_collision(traj,         # 检查是否撞障
                                                  self.clear_margin):  # 用安全裕度
                    continue                                    # 撞障轨迹直接丢弃
                score = self.evaluate(traj, cv, rem_now)        # 计算五项加权得分
                cands.append((score, cv, cw, traj))             # 记录候选轨迹
        if not cands:                                           # 窗口内全部撞障
            safe = self.u.simulate(state, 0.0, 0.0,             # 原地停车作为兜底
                                   self.dt, steps)              # 推演停车轨迹
            return 0.0, 0.0, safe, [], target                   # 返回停车控制
        cands.sort(key=lambda c: c[0], reverse=True)            # 按得分降序排列
        best = cands[0]                                         # 取最高分候选
        return best[1], best[2], best[3], cands, target         # 返回最优控制

    def draw_frame(self, state, v, w, steps, traj, cands, look, trail):  # 绘制当前帧
        """画候选轨迹、最优轨迹、机器人与状态文字"""
        p = self.plot                                           # 可视化对象简写
        p.clear()                                               # 清除上一帧图元
        for item in cands[:self.top_k]:                         # 只画前若干条候选
            t = item[3]                                         # 取出候选轨迹
            p.draw_line(t[:, 0], t[:, 1], color='gray',         # 浅灰细线画候选
                        lw=0.8, alpha=0.2)                      # 半透明不遮挡
        p.draw_points(np.asarray(trail), color='tab:orange',    # 画走过的轨迹
                      size=2.0, alpha=0.5)                      # 小点淡色
        p.draw_points(look.reshape(1, 2), color='tab:green',    # 画前瞻目标点
                      size=7.0)                                 # 绿色大点
        p.draw_line(traj[:, 0], traj[:, 1], color='tab:red',    # 醒目色画最优轨迹
                    lw=2.5, alpha=0.95)                         # 加粗便于观察
        p.draw_robot(state[0], state[1], state[2])              # 画当前机器人
        p.refresh(pause=0.02)                                   # 刷新形成动画

    def run(self, animate=True):  # 跑完整段规划
        """滚动执行 DWA 直到到达终点或超过步数上限"""
        if animate and self.plot is not None:                   # 需要动画时
            self.plot.init_figure("DWA - Dynamic Window Approach")   # 画静态环境
        state = np.array(self.env.start, dtype=float)           # 当前位姿
        goal = np.array(self.env.goal, dtype=float)             # 终点坐标
        v, w = 0.0, 0.0                                         # 当前速度控制
        reached = False                                         # 到达标志
        collided = False                                        # 碰撞标志
        trail = []                                              # 走过的点集
        steps = 0                                               # 实际步数
        for step in range(self.max_steps):                      # 逐步滚动规划
            v, w, traj, cands, look = self.plan_once(state, v, w)   # 求最优控制
            new_state = self.u.motion(state, v, w, self.dt)     # 执行一步运动
            if self.u.is_segment_collision(state, new_state,    # 检查真实碰撞
                                           0.0):                # 不留安全裕度
                collided = True                                 # 置碰撞标志
            state = new_state                                   # 更新当前位姿
            steps = step + 1                                    # 累计步数
            trail.append(state[:2].copy())                      # 记录走过位置
            if animate and self.plot is not None:               # 需要动画时
                if step % self.frame_skip == 0:                 # 按间隔抽帧
                    self.draw_frame(state, v, w, steps, traj,   # 绘制当前帧
                                    cands, look, trail)         # 传入轨迹与状态
            if self.u.dist(state, goal) < self.goal_tol:        # 到达终点判定
                reached = True                                  # 置到达标志
                break                                           # 结束规划循环
        if animate and self.plot is not None:                   # 需要动画时
            self.draw_frame(state, v, w, steps, traj,           # 补画最后一帧
                            cands, look, trail)                 # 展示终点位置
            self.plot.refresh(pause=0.5)                        # 停留便于观察
        return reached, collided, steps                         # 返回统计结果


def main():  # 程序入口
    env = Env()                                                 # 构造环境
    utils = Utils(env)                                          # 构造工具
    plot = Plotting(env, utils)                                 # 构造可视化
    dwa = DWA(env, utils, plot)                                 # 构造规划器
    t0 = time.time()                                            # 记录起始时间
    reached, collided, steps = dwa.run(animate=True)            # 执行规划
    print(f"Reached: {reached}  Collided: {collided}  Steps: {steps}  Time: {time.time()-t0:.1f}s")
    plot.hold()                                                 # 保持窗口直到 Ctrl+C


if __name__ == '__main__':  # 脚本直接运行时
    signal.signal(signal.SIGINT, signal.default_int_handler)   # 确保 Ctrl+C 一定有效
    try:                                                    # 捕获用户中断
        main()                                              # 调用入口函数
    except KeyboardInterrupt:                               # 用户按下 Ctrl+C
        print("\nInterrupted by Ctrl+C, exited.")           # 英文提示
        sys.exit(0)                                         # 正常退出进程
