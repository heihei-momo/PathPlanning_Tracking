"""
一键顺序播放仓库里的所有算法（单窗口模式）
所有算法跑在同一个子进程里、共用同一张画布，所以桌面上自始至终只有一个算法窗口，
窗口内容随算法切换；右侧实时显示终端输出。
动态全局规划算法（D* / LPA* / Dynamic-RRT）会用脚本注入障碍、逼它们重新规划。

用法：
    python3 run_all.py
"""

import json
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext

ROOT = os.path.dirname(os.path.abspath(__file__))   # 仓库根目录

# ============================ 任务清单 ============================
SEARCH_2D = ["bfs", "dfs", "Best_First", "Dijkstra", "Astar",      # 搜索式 2D 算法
             "Bidirectional_a_star", "ARAstar", "LRTAstar",        # 继续：双向 A*、ARA*、LRTA*
             "RTAAStar", "LPAstar", "D_star"]                      # 继续：RTAA*、LPA*、D*
RRT_2D = ["rrt", "rrt_connect", "extended_rrt", "dynamic_rrt",     # 采样式 2D 算法
          "rrt_star", "informed_rrt_star", "rrt_star_smart",       # 继续：RRT* 家族
          "fast_marching_trees", "batch_informed_trees",           # 继续：FMT*、BIT*
          "dubins_rrt_star"]                                       # 继续：Dubins-RRT*
LOCAL = ["dwa", "teb", "mpc"]                                      # 局部路径规划
TRACK = ["pure_pursuit", "lqr", "mpc"]                             # 轨迹跟踪

# ===================== 渲染器：跑在独立子进程里 =====================
# 它做三件事：
#   ① 把 matplotlib 的 figure()/subplots()/gcf() 全部打上补丁，
#      让所有算法都画到同一张画布上，于是全程只开一个窗口；
#   ② 依次加载并运行每个算法（显式调 main()，因为此时模块名不是 __main__）；
#   ③ 通过 stdout 打标记（@@TASK/@@DONE）向界面汇报进度，
#      通过 stdin 接收「skip / stop」指令。
RENDERER = r'''
import sys, os, json, time, signal, threading, importlib.util
import matplotlib

ROOT = os.getcwd()                                       # 仓库根目录
sys.path.insert(0, ROOT)                                 # 让 Sampling_based_Planning 可导入
sys.path.insert(0, os.path.join(ROOT, 'Search_based_Planning'))   # 让 Search_2D 可导入

import matplotlib.pyplot as plt                          # noqa: E402

plt.rcParams['figure.raise_window'] = False              # 关键：不让窗口反复抢焦点

SHARED = [None]                                          # 共用的那一张画布
_o_fig, _o_sub, _o_show = plt.figure, plt.subplots, plt.show    # 保存原始函数，避免递归


def shared_fig(*a, **k):                                 # 所有 figure() 都返回同一张
    if SHARED[0] is None or not plt.fignum_exists(SHARED[0].number):
        SHARED[0] = _o_fig(*a, **k)                      # 第一次才真的新建
    else:
        SHARED[0].clf()                                  # 之后只清空重用
    _o_fig(SHARED[0].number)                             # 用原始函数设为当前画布
    return SHARED[0]                                     # 返回共用画布


def shared_sub(nrows=1, ncols=1, **k):                   # 老算法靠 subplots 开坐标轴
    figsize = k.pop('figsize', None)                     # figsize 是画布级参数
    dpi = k.pop('dpi', None)                             # dpi 同理
    f = shared_fig()                                     # 取共用画布
    if figsize:                                          # 有要求就调尺寸
        f.set_size_inches(*figsize)                      # 设置画布尺寸
    if dpi:                                              # 有要求就调分辨率
        f.set_dpi(dpi)                                   # 设置 dpi
    f.clf()                                              # 清空画布
    ax = f.subplots(nrows, ncols, **k)                   # 在它上面建坐标轴
    _o_fig(f.number)                                     # 设为当前
    return f, ax                                         # 返回画布与坐标轴


def shared_gcf():                                        # 很多算法靠 plt.plot 隐式取画布
    if SHARED[0] is None or not plt.fignum_exists(SHARED[0].number):
        SHARED[0] = _o_fig()                             # 第一次才真的新建
    return SHARED[0]                                     # 之后都返回同一张


plt.figure, plt.subplots, plt.gcf = shared_fig, shared_sub, shared_gcf
plt.show = lambda *a, **k: None                          # 不让任何算法阻塞在 show 上

PLAIN = ('env', 'plotting', 'utils', 'queue', 'dwa', 'teb', 'mpc',   # 可能重名的顶层模块
         'pure_pursuit', 'lqr')


def prep(folder):                                        # 每个算法开跑前的环境隔离
    for p in list(sys.path):                             # 清掉上一次加的目录
        if os.path.basename(p.rstrip('/')) in ('Local_Planning', 'Tracking'):
            sys.path.remove(p)                           # 移除旧目录
    if folder:                                           # 加上本次需要的目录
        sys.path.insert(0, folder)                       # 插到最前
    for n in PLAIN:                                      # 清掉可能重名的模块缓存
        sys.modules.pop(n, None)                         # 让下次重新导入


def load_and_run(path, folder):                          # 跑一个普通算法
    prep(folder)                                         # 先隔离环境
    name = '_algo_' + os.path.basename(path)[:-3]        # 唯一的模块名
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, path))
    mod = importlib.util.module_from_spec(spec)          # 造模块对象
    sys.modules[name] = mod                              # 注册进去
    spec.loader.exec_module(mod)                         # 执行文件（此时不是 __main__）
    if not hasattr(mod, 'main'):                         # 没有入口就没法自动跑
        raise RuntimeError('该文件没有 main()')
    if hasattr(mod, 'Plotting'):                         # 屏蔽 hold()，否则会一直等 Ctrl+C
        mod.Plotting.hold = lambda self: None            # 换成空函数
    mod.main()                                           # 显式调用入口


class _Ev:                                               # 伪造的鼠标事件对象
    pass


def _click(cb, x, y):                                    # 注入一次“点击”
    e = _Ev()                                            # 造一个空对象
    e.xdata, e.ydata = float(x), float(y)                # 填上坐标
    cb(e)                                                # 调用真正的回调


def run_dynamic(which):                                  # 跑一个动态重规划示例
    if which == 'dstar':                                 # ---- D* ----
        prep(os.path.join(ROOT, 'Search_based_Planning'))    # 让 Search_2D 可导入
        from Search_2D.D_star import DStar               # 导入 D*
        d = DStar((5, 5), (45, 25))                      # 构造规划器
        d.run((5, 5), (45, 25))                          # 首次规划
        print('>>> 首次规划完成，路径长度 =', len(d.path))
        for frac in (0.50, 0.60, 0.70):                  # 依次挡住当前路径
            i = max(1, min(len(d.path) - 2, int(len(d.path) * frac)))   # 取路径上的点
            x, y = d.path[i]                             # 该点坐标
            print('>>> 第 %d%% 处注入障碍：(%d, %d)' % (frac * 100, x, y))
            _click(d.on_press, x, y)                     # 触发重新规划
            if display(1.6):                             # 停顿看动画
                return
            print('>>> 重规划后路径长度 =', len(d.path))
    elif which == 'lpa':                                 # ---- LPA* ----
        prep(os.path.join(ROOT, 'Search_based_Planning'))    # 让 Search_2D 可导入
        from Search_2D.LPAstar import LPAStar            # 导入 LPA*
        lp = LPAStar((5, 5), (45, 25), 'euclidean')      # 构造规划器
        lp.run()                                         # 首次计算
        path = lp.extract_path()                         # 取出当前路径
        print('>>> 首次规划完成，路径长度 =', len(path))
        for frac in (0.50, 0.60, 0.70):                  # 依次挡住当前路径
            i = max(1, min(len(path) - 2, int(len(path) * frac)))        # 取路径上的点
            x, y = path[i]                               # 该点坐标
            print('>>> 第 %d%% 处注入障碍：(%d, %d)' % (frac * 100, x, y))
            _click(lp.on_press, x, y)                    # 触发增量重规划
            if display(1.6):                             # 停顿看动画
                return
            path = lp.extract_path()                     # 取重规划后的新路径
            print('>>> 重规划后路径长度 =', len(path))
    else:                                                # ---- Dynamic-RRT ----
        from Sampling_based_Planning.rrt_2D.dynamic_rrt import DynamicRrt   # 导入动态 RRT
        dr = DynamicRrt((2, 2), (49, 24), 0.5, 0.1, 0.6, 5000)   # 构造规划器
        dr.planning()                                    # 首次生长随机树
        print('>>> 首次规划完成，路径点数 =', len(dr.path))
        for frac in (0.45, 0.55, 0.65):                  # 依次挡住当前路径
            i = max(1, min(len(dr.path) - 1, int(len(dr.path) * frac)))  # 取路径上的点
            x, y = int(dr.path[i][0]), int(dr.path[i][1])    # 取整作为圆心
            print('>>> 第 %d%% 处注入圆形障碍：圆心 (%d, %d) 半径 2' % (frac * 100, x, y))
            _click(dr.on_press, x, y)                    # 触发重规划
            if display(1.6):                             # 停顿看动画
                return
            print('>>> 重规划后路径点数 =', len(dr.path))


class _Interrupt(Exception):                             # 用户要求中断当前算法
    pass


_skip_flag = [False]                                     # 是否请求跳过
_stop_flag = [False]                                     # 是否请求停止


def _input_thread():                                     # 后台线程：一直读 stdin
    while True:                                          # 直到管道关闭
        line = sys.stdin.readline()                      # 读一行
        if line == '':                                   # 读到 EOF
            return                                       # 线程结束
        c = line.strip()                                 # 去掉空白
        if c == 'skip':                                  # 跳过指令
            _skip_flag[0] = True                         # 置跳过标志
        elif c == 'stop':                                # 停止指令
            _stop_flag[0] = True                         # 置停止标志


def _on_alarm(signum, frame):                            # 定时器回调：负责打断算法
    if _stop_flag[0] or _skip_flag[0]:                   # 用户下了指令
        raise _Interrupt()                               # 抛异常打断当前算法


threading.Thread(target=_input_thread, daemon=True).start()   # 启动输入线程
signal.signal(signal.SIGALRM, _on_alarm)                 # 装定时器回调
signal.setitimer(signal.ITIMER_REAL, 0.15, 0.15)         # 每 0.15 秒检查一次


_shown = [False]                                         # 窗口是否已经显示过


def display(seconds):                                    # 显示若干秒，期间可被打断
    fig = SHARED[0]                                      # 当前共用画布
    if fig is None:                                      # 还没画布就直接返回
        return None                                      # 无需显示
    if not _shown[0]:                                    # 第一次显示
        _o_show(block=False)                             # 真正把窗口显示出来
        _shown[0] = True                                 # 记住已显示
    t0 = time.time()                                     # 开始计时
    while time.time() - t0 < seconds:                    # 直到时间到
        fig.canvas.flush_events()                        # 处理 GUI 事件但不抬升窗口
        time.sleep(0.03)                                 # 降低 CPU 占用
        if _stop_flag[0] or _skip_flag[0]:               # 用户下了指令
            return 'stop' if _stop_flag[0] else 'skip'   # 立刻返回给上层
    return None                                          # 正常结束


tasks = json.loads(sys.argv[1])                          # 任务清单
for idx, task in enumerate(tasks):                       # 逐个算法播放
    if idx > 0 and SHARED[0] is not None:                # 不是第一个算法时
        SHARED[0].clf()                                  # 先清空画布，避免与上一个重叠
    print('@@TASK|%d|%s' % (idx, task['name']), flush=True)     # 汇报开始
    ok = True                                            # 是否成功
    try:
        if task['kind'] == 'dynamic':                    # 动态重规划示例
            run_dynamic(task['target'])                  # 跑内嵌的示例逻辑
        else:                                            # 普通算法
            load_and_run(task['path'], task.get('folder'))    # 加载并运行
    except _Interrupt:                                   # 被 skip/stop 打断
        print('（已按指令中断当前算法）', flush=True)          # 提示
    except Exception as e:                               # 单个算法出错不影响后面
        ok = False                                       # 记失败
        print('!! 该算法出错：%s: %s' % (type(e).__name__, e), flush=True)
    if _stop_flag[0]:                                    # 用户要停止
        sys.exit(0)                                      # 直接退出（窗口随之关闭）
    print('@@RESULT|%d|%s' % (idx, 'ok' if ok else 'fail'), flush=True)   # 汇报结果
    try:
        display(BUDGET)                                  # 展示规定时长
    except _Interrupt:                                   # 展示期间被打断
        pass                                             # 直接进入下一轮
    if _stop_flag[0]:                                    # 用户要停止
        sys.exit(0)                                      # 退出
    _skip_flag[0] = False                                # 清掉跳过标志
    print('@@DONE|%d' % idx, flush=True)                 # 汇报收尾（画面留着给下一位覆盖）

print('@@FINISHED', flush=True)                          # 全部播完
while True:                                              # 保持窗口不退出
    if SHARED[0] is not None:                            # 有画布才刷新
        SHARED[0].canvas.flush_events()                  # 处理 GUI 事件
    time.sleep(0.2)                                      # 降低 CPU 占用
    if _stop_flag[0]:                                    # 收到停止指令
        sys.exit(0)                                      # 退出，窗口关闭
'''.replace('BUDGET', 'float(sys.argv[2])')              # 把占位符换成真实时长


def build_tasks():
    """按分组组织出完整的任务清单"""
    groups = []                                                  # 分组列表
    groups.append(("搜索式规划", [                                # 搜索式分组
        {"name": n, "kind": "script",                             # 按脚本运行
         "path": "Search_based_Planning/Search_2D/%s.py" % n,     # 脚本路径
         "folder": None}                                          # 不需要额外 sys.path
        for n in SEARCH_2D]))                                     # 逐个算法生成
    groups.append(("采样式规划", [                                # 采样式分组
        {"name": n, "kind": "script",                             # 同样按脚本运行
         "path": "Sampling_based_Planning/rrt_2D/%s.py" % n,      # 脚本路径
         "folder": None}                                          # 不需要额外 sys.path
        for n in RRT_2D]))                                        # 逐个算法生成
    groups.append(("局部路径规划", [                              # 局部规划分组
        {"name": n, "kind": "script",                             # 按脚本运行
         "path": "Local_Planning/%s.py" % n,                      # 脚本路径
         "folder": "Local_Planning"}                              # 需要把该目录加进 sys.path
        for n in LOCAL]))                                         # 逐个算法生成
    groups.append(("轨迹跟踪", [                                  # 轨迹跟踪分组
        {"name": n, "kind": "script",                             # 按脚本运行
         "path": "Tracking/%s.py" % n,                            # 脚本路径
         "folder": "Tracking"}                                    # 需要把该目录加进 sys.path
        for n in TRACK]))                                         # 逐个算法生成
    groups.append(("动态重规划示例", [                            # 动态示例分组
        {"name": "D* 动态重规划", "kind": "dynamic", "target": "dstar"},      # D*
        {"name": "LPA* 增量重规划", "kind": "dynamic", "target": "lpa"},      # LPA*
        {"name": "Dynamic-RRT 重规划", "kind": "dynamic", "target": "drrt"},  # Dynamic-RRT
    ]))                                                           # 三个示例
    return groups                                                 # 返回分组清单


class Player:
    """顺序播放器：只启动一个渲染器子进程，它内部依次跑完所有算法"""

    def __init__(self, tasks, budget, on_log, on_state):
        self.tasks = tasks                                        # 待播任务列表
        self.budget = budget                                      # 每个任务的展示时长
        self.on_log = on_log                                      # 输出回调（线程安全）
        self.on_state = on_state                                  # 状态回调（线程安全）
        self.results = {}                                         # 下标 -> 是否成功
        self.stop_flag = False                                    # 停止标志
        self.proc = None                                          # 渲染器子进程
        self.thread = None                                        # 后台线程

    def start(self):
        """启动播放线程"""
        self.thread = threading.Thread(target=self._run, daemon=True)   # 守护线程
        self.thread.start()                                       # 开始跑

    def skip(self):
        """请求跳过当前算法（渲染器会立刻进入下一个）"""
        self._send('skip')                                        # 发指令

    def stop(self):
        """请求停止，并关掉那个唯一的算法窗口"""
        self.stop_flag = True                                     # 置停止标志
        self._send('stop')                                        # 先发温和指令
        time.sleep(0.2)                                           # 给对方一点反应时间
        self._kill()                                              # 还没退就强杀

    def close_all(self):
        """关掉渲染器子进程（等于关闭那个唯一窗口）"""
        self.stop_flag = True                                     # 置停止标志
        self._kill()                                              # 结束子进程

    def _send(self, cmd):
        """往渲染器的 stdin 写一行指令"""
        try:
            if self.proc and self.proc.stdin:                     # 子进程还活着
                self.proc.stdin.write(cmd + '\n')                 # 写指令
                self.proc.stdin.flush()                           # 立刻刷新
        except Exception:                                         # 管道断了就算了
            pass                                                  # 忽略

    def _kill(self):
        """结束渲染器子进程"""
        p = self.proc                                             # 取子进程
        if p is None or p.poll() is not None:                     # 已经结束
            return                                                # 不用管
        p.terminate()                                             # 先温和结束
        try:
            p.wait(timeout=2)                                     # 最多等 2 秒
        except subprocess.TimeoutExpired:                         # 赖着不走
            p.kill()                                              # 强制杀掉

    def _run(self):
        """播放线程主体：启动渲染器并解析它的输出"""
        total = len(self.tasks)                                   # 总任务数
        env = dict(os.environ)                                    # 复制环境变量
        env.pop("MPLBACKEND", None)                               # 用默认交互后端才能弹窗
        env.setdefault("MPLCONFIGDIR", "/tmp/mplcfg_runner")       # 保证有可写缓存目录
        try:
            os.makedirs(env["MPLCONFIGDIR"], exist_ok=True)        # 建缓存目录
        except OSError:                                           # 建不了就算了
            pass                                                  # 忽略
        cmd = [sys.executable, "-u", "-c", RENDERER,              # 启动渲染器
               json.dumps(self.tasks), str(self.budget)]          # 传任务清单与时长
        self.proc = subprocess.Popen(                             # 起子进程
            cmd, cwd=ROOT, env=env,                               # 在仓库根目录运行
            stdin=subprocess.PIPE,                                # 用来收 skip/stop
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,     # 输出合并
            text=True, bufsize=1)                                 # 按行读
        out_q = queue.Queue()                                     # 读取线程用的队列

        def reader():                                             # 后台读取输出
            for line in self.proc.stdout:                         # 逐行读
                out_q.put(line)                                   # 送进队列
            out_q.put(None)                                       # 结束标记

        threading.Thread(target=reader, daemon=True).start()      # 启动读取线程
        while True:                                               # 一边读一边解析
            try:
                line = out_q.get(timeout=0.1)                     # 最多等 0.1 秒
            except queue.Empty:                                   # 暂时没输出
                if self.stop_flag:                                # 用户要停
                    break                                         # 退出
                continue                                          # 继续等
            if line is None:                                      # 子进程结束
                break                                             # 退出
            self._handle(line)                                    # 解析这一行
        self._kill()                                              # 确保子进程结束
        self.on_state(0, total, None, "finished")                 # 上报全部结束

    def _handle(self, line):
        """解析渲染器的一行输出：既可能是标记，也可能是算法自己的打印"""
        s = line.rstrip("\n")                                     # 去掉换行
        if s.startswith("@@TASK|"):                               # 开始某个算法
            _, idx, name = s.split("|", 2)                        # 拆出下标与名字
            i = int(idx)                                          # 转数字
            self.on_log("\n" + "=" * 68 + "\n")                   # 打印分隔线
            self.on_log("[%d/%d] %s\n" % (i + 1, len(self.tasks), name))   # 打印任务标题
            self.on_log("=" * 68 + "\n")                          # 打印分隔线
            self.on_state(i + 1, len(self.tasks), self.tasks[i], "running")   # 上报状态
        elif s.startswith("@@RESULT|"):                           # 某个算法的结果
            _, idx, res = s.split("|", 2)                         # 拆出下标与结果
            self.results[int(idx)] = (res == "ok")                # 记下来
        elif s.startswith("@@DONE|"):                             # 某个算法播完
            i = int(s.split("|")[1])                              # 取下标
            ok = self.results.get(i, True)                        # 取结果
            self.on_state(i + 1, len(self.tasks), self.tasks[i],  # 上报完成
                          "done" if ok else "fail")               # 成功或失败
        elif s.startswith("@@FINISHED"):                          # 全部播完
            self.on_log("\n全部演示完毕，算法窗口保持打开；"
                        "点「停止」可关闭。\n")                    # 提示
        else:                                                     # 普通输出
            self.on_log(line)                                     # 原样显示


class App:
    """Tkinter 界面"""

    def __init__(self, root):
        self.root = root                                          # 主窗口
        root.title("算法演示播放器 — 单窗口轮流播放 + 实时终端输出")   # 窗口标题
        root.geometry("1180x760")                                 # 初始尺寸
        root.protocol("WM_DELETE_WINDOW", self.on_close)           # 关主窗口时收尾

        self.log_q = queue.Queue()                                # 界面输出队列
        self.state_q = queue.Queue()                              # 界面状态队列
        self.player = None                                        # 播放器对象
        self.rows = {}                                            # 任务键 -> 表格行号
        self.groups = build_tasks()                               # 任务清单

        top = ttk.Frame(root, padding=8)                          # 顶部控制条
        top.pack(fill="x")                                        # 顶部停靠
        ttk.Label(top, text="每个算法展示:").pack(side="left")     # 提示文字
        self.budget = tk.IntVar(value=15)                         # 默认每个展示 15 秒
        ttk.Spinbox(top, from_=5, to=180, width=5,                # 秒数输入框
                    textvariable=self.budget).pack(side="left", padx=4)
        ttk.Label(top, text="秒").pack(side="left")               # 单位
        self.btn_start = ttk.Button(top, text="开始播放",         # 开始按钮
                                    command=self.on_start)        # 绑定回调
        self.btn_start.pack(side="left", padx=12)                 # 放置按钮
        self.btn_skip = ttk.Button(top, text="跳过当前",          # 跳过按钮
                                   command=self.on_skip, state="disabled")
        self.btn_skip.pack(side="left", padx=4)                   # 放置按钮
        self.btn_stop = ttk.Button(top, text="停止",              # 停止按钮
                                   command=self.on_stop, state="disabled")
        self.btn_stop.pack(side="left", padx=4)                   # 放置按钮
        self.progress = ttk.Label(top, text="就绪")               # 进度文字
        self.progress.pack(side="left", padx=16)                  # 放置文字

        mid = ttk.PanedWindow(root, orient="horizontal")          # 左右分栏
        mid.pack(fill="both", expand=True, padx=8, pady=6)        # 填满剩余空间

        left = ttk.Frame(mid)                                     # 左侧容器
        mid.add(left, weight=1)                                   # 左侧占 1 份宽
        ttk.Label(left, text="播放内容（勾选分组）").pack(anchor="w")   # 小标题
        self.checks = {}                                          # 分组名 -> 勾选变量
        for gname, _ in self.groups:                              # 每个分组一个复选框
            var = tk.BooleanVar(value=True)                       # 默认全选
            self.checks[gname] = var                              # 记下来
            ttk.Checkbutton(left, text=gname,                     # 复选框
                            variable=var).pack(anchor="w")        # 左对齐

        ttk.Label(left, text="任务清单").pack(anchor="w", pady=(10, 2))   # 小标题
        self.tree = ttk.Treeview(left, columns=("s",),            # 任务表格
                                 show="tree headings", height=22)  # 显示树形与表头
        self.tree.heading("#0", text="算法 / 示例")                # 第一列表头
        self.tree.heading("s", text="状态")                       # 状态列表头
        self.tree.column("#0", width=230, anchor="w")             # 第一列宽度
        self.tree.column("s", width=90, anchor="center")          # 状态列宽度
        for gname, tasks in self.groups:                          # 按分组填充表格
            node = self.tree.insert("", "end", text=gname, open=True)    # 分组行
            for task in tasks:                                    # 组内每个任务
                iid = self.tree.insert(node, "end", text="  " + task["name"],   # 任务行
                                       values=("待播放",))           # 初始状态
                task["key"] = gname + "/" + task["name"]           # 唯一键（同名不撞车）
                self.rows[task["key"]] = iid                       # 记下行号
        self.tree.pack(fill="both", expand=True)                  # 放置表格

        right = ttk.Frame(mid)                                    # 右侧容器
        mid.add(right, weight=2)                                  # 右侧占 2 份宽
        ttk.Label(right, text="终端输出（实时）").pack(anchor="w")  # 小标题
        self.log = scrolledtext.ScrolledText(right, wrap="word")  # 可滚动文本区
        self.log.pack(fill="both", expand=True)                   # 填满
        self.log.configure(state="disabled")                      # 先设为只读

        root.after(50, self._pump)                                # 启动队列轮询
        self._append("说明：所有算法在同一个窗口里轮流播放（桌面只开一个算法窗口）。\n"
                     "点「开始播放」后按清单顺序运行，终端输出实时显示在右边；\n"
                     "动态重规划示例会自动注入障碍、逼它们重新规划。\n"
                     "「跳过当前」立即切下一个；「停止」结束播放并关闭算法窗口。\n")

    # ---------------- 界面辅助 ----------------

    def _append(self, text, tag=None):
        """往日志区追加文本"""
        self.log.configure(state="normal")                        # 解锁写入
        self.log.insert("end", text, tag)                         # 追加内容
        self.log.see("end")                                       # 滚到最底
        self.log.configure(state="disabled")                      # 重新只读

    def _pump(self):
        """主线程轮询队列，把后台消息搬进界面"""
        try:
            while True:                                           # 尽量排空输出队列
                self._append(self.log_q.get_nowait())             # 追加一行输出
        except queue.Empty:                                       # 队列空了
            pass                                                  # 结束
        try:
            while True:                                           # 尽量排空状态队列
                idx, total, task, state = self.state_q.get_nowait()   # 取一条状态
                if state == "finished":                           # 全部结束
                    self.progress.config(text="全部播放完成")      # 更新进度文字
                    self._set_buttons(False)                      # 恢复按钮
                elif state == "running":                          # 正在运行
                    self.progress.config(text="正在运行 [%d/%d]：%s" % (idx, total, task["name"]))
                    self._set_row(task.get("key"), "运行中")        # 更新该行状态
                else:                                             # 单个任务结束
                    self._set_row(task.get("key"), "完成" if state == "done" else "异常")
        except queue.Empty:                                       # 状态队列空了
            pass                                                  # 结束
        self.root.after(50, self._pump)                            # 50ms 后再轮询

    def _set_row(self, key, text):
        """更新任务表格里某一行的状态列"""
        iid = self.rows.get(key)                                  # 取行号
        if iid:                                                   # 存在才更新
            self.tree.set(iid, "s", text)                         # 写状态
            self.tree.see(iid)                                    # 滚动到可见

    def _set_buttons(self, running):
        """按运行状态切换按钮可用性"""
        self.btn_start.config(state="disabled" if running else "normal")   # 运行中禁用开始
        self.btn_skip.config(state="normal" if running else "disabled")    # 运行中启用跳过
        self.btn_stop.config(state="normal" if running else "disabled")    # 运行中启用停止

    def _collect(self):
        """按勾选情况收集要播放的任务"""
        tasks = []                                                # 结果列表
        for gname, group_tasks in self.groups:                    # 遍历分组
            var = self.checks.get(gname)                          # 取该分组的勾选状态
            if var is not None and var.get():                     # 该分组被勾选
                tasks.extend(group_tasks)                         # 收进列表
        return tasks                                              # 返回任务列表

    # ---------------- 按钮回调 ----------------

    def on_start(self):
        """开始播放"""
        tasks = self._collect()                                   # 收集任务
        if not tasks:                                             # 一个都没勾
            self._append("注意：没有勾选任何分组。\n")               # 提示
            return                                                # 直接返回
        for iid in self.rows.values():                            # 重置所有行状态
            self.tree.set(iid, "s", "待播放")                      # 回到初始文字
        self._append("\n" + "#" * 68 + "\n")                      # 打印开始分隔
        self._append("开始播放，共 %d 个算法/示例，每个展示 %d 秒（全程只有一个窗口）\n"
                     % (len(tasks), self.budget.get()))           # 打印总览
        self._append("#" * 68 + "\n")                             # 打印分隔
        self.player = Player(tasks, self.budget.get(),            # 构造播放器
                             self.log_q.put,                      # 输出直接进队列
                             lambda *a: self.state_q.put(a))      # 状态打包成元组再进队列
        self._set_buttons(True)                                   # 切换按钮状态
        self.player.start()                                       # 启动后台播放

    def on_skip(self):
        """跳过当前算法"""
        if self.player:                                           # 有播放器在跑
            self._append("\n手动跳过当前算法\n")                    # 提示
            self.player.skip()                                    # 发跳过指令

    def on_stop(self):
        """停止播放，并关闭算法窗口"""
        if self.player:                                           # 有播放器在跑
            self._append("\n已请求停止，正在关闭算法窗口\n")        # 提示
            self.player.stop()                                    # 停止播放 + 关窗

    def on_close(self):
        """关闭主窗口时，把算法窗口一起收掉"""
        if self.player:                                           # 有播放器在跑
            self.player.close_all()                               # 关掉渲染器
        self.root.destroy()                                       # 销毁主窗口


def main():
    root = tk.Tk()                                                # 创建主窗口
    App(root)                                                     # 构造界面
    root.mainloop()                                               # 进入事件循环


if __name__ == '__main__':
    main()
