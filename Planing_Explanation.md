# 2D 路径规划算法详解

覆盖 `Search_based_Planning/Search_2D/` 与 `Sampling_based_Planning/rrt_2D/` 下全部 **23 个 2D 算法**，每个算法一段伪代码。

## 通用符号

| 符号 | 含义 |
|---|---|
| `g` | 从起点到当前点已经花掉的代价 |
| `h` | 当前点到终点的估计代价 |
| `f = g + h` | 总估计代价，越小越优先扩展 |
| `OPEN` | 待扩展节点集合（一般按 `f` 取最小） |
| `CLOSED` | 已扩展节点集合 |
| `PARENT` | 父指针，用于回溯路径 |
| `rhs` | 节点的"一步前瞻"`g` 值，LPA\*/D\* 系专用 |

两大流派：**搜索式**在 51×31 的离散网格上逐格扩展；**采样式**在 50×30 的连续平面上随机撒点、连成树。

伪代码约定：`←` 表示赋值，`=` 表示比较，关键字用英文，行尾 `#` 为注释。

## 目录

- [通用符号](#通用符号)
- [搜索式规划（13 个）](#搜索式规划13-个)
  - [1. BFS 广度优先搜索 · `bfs.py`](#1-bfs-广度优先搜索-bfspy)
  - [2. DFS 深度优先搜索 · `dfs.py`](#2-dfs-深度优先搜索-dfspy)
  - [3. Best-First 最佳优先搜索 · `Best_First.py`](#3-best-first-最佳优先搜索-best_firstpy)
  - [4. Dijkstra 算法 · `Dijkstra.py`](#4-dijkstra-算法-dijkstrapy)
  - [5. A\* 算法 · `Astar.py`](#5-a-算法-astarpy)
  - [6. 双向 A\* · `Bidirectional_a_star.py`](#6-双向-a-bidirectional_a_starpy)
  - [7. ARA\* · `ARAstar.py`](#7-ara-arastarpy)
  - [8. LRTA\* · `LRTAstar.py`](#8-lrta-lrtastarpy)
  - [9. RTAA\* · `RTAAStar.py`](#9-rtaa-rtaastarpy)
  - [10. LPA\* · `LPAstar.py`](#10-lpa-lpastarpy)
  - [11. D\* · `D_star.py`](#11-d-d_starpy)
  - [12. D\* Lite · `D_star_Lite.py`](#12-d-lite-d_star_litepy)
  - [13. Anytime D\* · `Anytime_D_star.py`](#13-anytime-d-anytime_d_starpy)
- [采样式规划（10 个）](#采样式规划10-个)
  - [14. RRT · `rrt.py`](#14-rrt-rrtpy)
  - [15. RRT-Connect · `rrt_connect.py`](#15-rrt-connect-rrt_connectpy)
  - [16. Extended-RRT · `extended_rrt.py`](#16-extended-rrt-extended_rrtpy)
  - [17. Dynamic-RRT · `dynamic_rrt.py`](#17-dynamic-rrt-dynamic_rrtpy)
  - [18. RRT\* · `rrt_star.py`](#18-rrt-rrt_starpy)
  - [19. Informed RRT\* · `informed_rrt_star.py`](#19-informed-rrt-informed_rrt_starpy)
  - [20. RRT\*-Smart · `rrt_star_smart.py`](#20-rrt-smart-rrt_star_smartpy)
  - [21. FMT\*（快速行进树） · `fast_marching_trees.py`](#21-fmt快速行进树-fast_marching_treespy)
  - [22. BIT\*（批处理知情树） · `batch_informed_trees.py`](#22-bit批处理知情树-batch_informed_treespy)
  - [23. Dubins-RRT\* · `dubins_rrt_star.py`](#23-dubins-rrt-dubins_rrt_starpy)
- [附录 A：算法对比](#附录-a算法对比)
  - [搜索式（13 个）](#搜索式13-个)
  - [采样式（10 个）](#采样式10-个)
  - [其他事实](#其他事实)
- [附录 B：运行命令](#附录-b运行命令)
---

## 搜索式规划（13 个）

### 1. BFS 广度优先搜索 · `bfs.py`

靠队尾入堆模拟先进先出，逐层扩散，与 DFS 相反。

```text
BFS(s_start, s_goal):
    PARENT[s_start] ← s_start
    g[s_start] ← 0
    g[s_goal] ← ∞
    OPEN ← 优先队列，压入 (0, s_start)
    CLOSED ← []
    while OPEN ≠ ∅:
        _, s ← OPEN.pop()
        CLOSED.append(s)
        if s = s_goal: break
        for s_n ∈ 邻居(s):
            new_cost ← g[s] + cost(s, s_n)
            if s_n ∉ g: g[s_n] ← ∞
            if new_cost < g[s_n]:
                g[s_n] ← new_cost
                PARENT[s_n] ← s
                prior ← OPEN 尾元素优先级 + 1      # 插到队尾，近似先进先出
                OPEN.push((prior, s_n))
    return 回溯(PARENT), CLOSED
```

### 2. DFS 深度优先搜索 · `dfs.py`

新节点插到队首，后进先出，与 BFS 的逐层扩散相反。

```text
DFS(s_start, s_goal):
    PARENT[s_start] ← s_start
    g[s_start] ← 0
    g[s_goal] ← ∞
    OPEN ← 优先队列，压入 (0, s_start)
    CLOSED ← []
    while OPEN ≠ ∅:
        _, s ← OPEN.pop()
        CLOSED.append(s)
        if s = s_goal: break
        for s_n ∈ 邻居(s):
            new_cost ← g[s] + cost(s, s_n)
            if s_n ∉ g: g[s_n] ← ∞
            if new_cost < g[s_n]:
                g[s_n] ← new_cost
                PARENT[s_n] ← s
                prior ← OPEN 首元素优先级 - 1      # 插到队首，近似后进先出
                OPEN.push((prior, s_n))
    return 回溯(PARENT), CLOSED
```

### 3. Best-First 最佳优先搜索 · `Best_First.py`

只按启发值排序，不看已走代价，贪心快但不最优。

```text
BestFirst(s_start, s_goal):
    PARENT[s_start] ← s_start
    g[s_start] ← 0
    g[s_goal] ← ∞
    OPEN ← 优先队列，压入 (h(s_start), s_start)
    CLOSED ← []
    while OPEN ≠ ∅:
        _, s ← OPEN.pop()
        CLOSED.append(s)
        if s = s_goal: break
        for s_n ∈ 邻居(s):
            new_cost ← g[s] + cost(s, s_n)
            if s_n ∉ g: g[s_n] ← ∞
            if new_cost < g[s_n]:                  # g 只用于门控，不入优先级
                g[s_n] ← new_cost
                PARENT[s_n] ← s
                OPEN.push((h(s_n), s_n))
    return 回溯(PARENT), CLOSED
```

### 4. Dijkstra 算法 · `Dijkstra.py`

按累计代价 g 扩展，不用启发值，即 h=0 的 A\*。

```text
Dijkstra(s_start, s_goal):
    PARENT[s_start] ← s_start
    g[s_start] ← 0
    g[s_goal] ← ∞
    OPEN ← 优先队列，压入 (0, s_start)
    CLOSED ← []
    while OPEN ≠ ∅:
        _, s ← OPEN.pop()                          # 不判 CLOSED，节点可能重复扩展
        CLOSED.append(s)
        if s = s_goal: break
        for s_n ∈ 邻居(s):
            new_cost ← g[s] + cost(s, s_n)
            if s_n ∉ g: g[s_n] ← ∞
            if new_cost < g[s_n]:
                g[s_n] ← new_cost
                PARENT[s_n] ← s
                OPEN.push((new_cost, s_n))         # 优先级就是 g
    return 回溯(PARENT), CLOSED
```

### 5. A\* 算法 · `Astar.py`

按 f=g+h 排序，兼顾已走与剩余代价，比 Dijkstra 扩展少。

```text
AStar(s_start, s_goal):
    PARENT[s_start] ← s_start
    g[s_start] ← 0
    g[s_goal] ← ∞
    OPEN ← 优先队列，压入 (g[s_start] + h(s_start), s_start)
    CLOSED ← []
    while OPEN ≠ ∅:
        _, s ← OPEN.pop()
        CLOSED.append(s)
        if s = s_goal: break
        for s_n ∈ 邻居(s):
            new_cost ← g[s] + cost(s, s_n)         # 碰撞时 cost 为 ∞
            if s_n ∉ g: g[s_n] ← ∞
            if new_cost < g[s_n]:
                g[s_n] ← new_cost
                PARENT[s_n] ← s
                OPEN.push((g[s_n] + h(s_n), s_n))
    return 回溯(PARENT), CLOSED
```

### 6. 双向 A\* · `Bidirectional_a_star.py`

起点终点同时扩展，两侧相遇即停，比单向 A\* 更快。

```text
BiAStar(s_start, s_goal):
    g_fore[s_start] ← 0, PARENT_fore[s_start] ← s_start
    g_back[s_goal] ← 0, PARENT_back[s_goal] ← s_goal, s_meet ← s_start
    OPEN_fore.push((h(s_start, s_goal), s_start))
    OPEN_back.push((h(s_goal, s_start), s_goal))
    while OPEN_fore ≠ ∅ and OPEN_back ≠ ∅:         # 两侧都非空才交替扩展
        s_fore ← OPEN_fore.pop()
        if s_fore ∈ PARENT_back: s_meet ← s_fore, break   # 撞上后向标记即相遇
        for s_n ∈ 邻居(s_fore):
            if s_n ∉ g_fore: g_fore[s_n] ← ∞
            new_cost ← g_fore[s_fore] + cost(s_fore, s_n)
            if new_cost < g_fore[s_n]:
                g_fore[s_n] ← new_cost
                PARENT_fore[s_n] ← s_fore
                OPEN_fore.push((g_fore[s_n] + h(s_n, s_goal), s_n))
        s_back ← OPEN_back.pop()
        if s_back ∈ PARENT_fore: s_meet ← s_back, break   # 撞上前向标记即相遇
        for s_n ∈ 邻居(s_back):
            if s_n ∉ g_back: g_back[s_n] ← ∞
            new_cost ← g_back[s_back] + cost(s_back, s_n)
            if new_cost < g_back[s_n]:
                g_back[s_n] ← new_cost
                PARENT_back[s_n] ← s_back
                OPEN_back.push((g_back[s_n] + h(s_n, s_start), s_n))
    return 回溯(s_meet, PARENT_fore) + 回溯(s_meet, PARENT_back)   # 起点→相遇点→终点
```

### 7. ARA\* · `ARAstar.py`

先用大权重 e 快速出解，再减小 e 反复改进，随时可停。

```text
ARAStar(s_start, s_goal, e):
    g[s_start] ← 0, g[s_goal] ← ∞, PARENT[s_start] ← s_start
    OPEN ← {s_start: g[s_start] + e * h(s_start)}
    path ← []
    while True:                                    # 外层：权重递减
        while True:                                # 内层：ImprovePath
            s, f_small ← OPEN 中 f 最小者
            if g[s_goal] + e * h(s_goal) ≤ f_small: break   # 不是弹出终点才停
            OPEN.remove(s), CLOSED.add(s)
            for s_n ∈ 邻居(s):
                if s_n ∈ 障碍: continue
                new_cost ← g[s] + cost(s, s_n)
                if s_n ∉ g or new_cost < g[s_n]:
                    g[s_n] ← new_cost, PARENT[s_n] ← s
                    if s_n ∈ CLOSED: INCONS[s_n] ← 0.0     # g 下降导致不一致
                    else: OPEN[s_n] ← g[s_n] + e * h(s_n)
        path.append(回溯(PARENT))
        if update_e() ≤ 1: break                   # 次优下界降到 1 才结束
        e ← e - 0.4
        OPEN ← OPEN ∪ INCONS                       # 不一致节点并回 OPEN
        OPEN ← {s: g[s] + e * h(s) | s ∈ OPEN}     # 按新权重刷新 f
        INCONS ← ∅, CLOSED ← ∅
    return path, visited
```

### 8. LRTA\* · `LRTAstar.py`

每轮只扩展 N 个节点，并把学到的 h 值留到下一轮复用。

```text
searching(s_start):
    init()                                     # 用初始启发式填满全局 h 表
    while True:
        OPEN, CLOSED ← AStar(s_start, N)        # 有界 A* 每轮只扩展 N 个节点
        if OPEN = "FOUND":
            path.append(CLOSED)
            break                              # 已到终点，结束
        h_value ← iteration(CLOSED)             # 每点取 min(cost + h)，迭代到收敛
        for x ∈ h_value:
            h_table[x] ← h_value[x]             # 学到的 h 写回全局表
        s_start, path_k ← extract_path_in_CLOSE(s_start, h_value)
        path.append(path_k)                     # 沿 h 最小的邻居走到 OPEN 边缘
```

### 9. RTAA\* · `RTAAStar.py`

用 OPEN 的最小 f 值反推 h 表，比 LRTA\* 收敛更快。

```text
searching(s_start):
    init()                                     # 用初始启发式填满全局 h 表
    while True:
        OPEN, CLOSED, g_table, PARENT ← Astar(s_start, N)   # 每轮只扩展 N 个节点
        if OPEN = "FOUND":
            path.append(CLOSED)
            break                              # 已到终点，结束
        for x ∈ OPEN:
            v_open[x] ← g_table[PARENT[x]] + 1 + h_table[x]  # +1 是源码写死的步长
        s_open ← argmin(v_open)                 # 估计最小的 OPEN 节点
        f_min ← v_open[s_open]
        for x ∈ CLOSED:
            h_value[x] ← f_min - g_table[x]      # 用 f_min 反推 h
            h_table[x] ← h_value[x]              # 写回全局 h 表
        s_start, path_k ← extract_path_in_CLOSE(s_start, s_open, h_value)
        path.append(path_k)
```

### 10. LPA\* · `LPAstar.py`

起点固定，障碍变化时复用 g 与 rhs 做增量重算。

```text
ComputeShortestPath():
    while True:
        s, v ← TopKey()                        # 取 U 中键最小的节点
        if v ≥ CalculateKey(s_goal) and rhs[s_goal] = g[s_goal]:
            break                              # 目标已局部一致则收敛
        U.pop(s)
        if g[s] > rhs[s]:                      # 过一致：rhs 减小
            g[s] ← rhs[s]
        else:                                  # 欠一致：原 g 失效
            g[s] ← ∞
            UpdateVertex(s)
        for s_n ∈ 邻居(s):
            UpdateVertex(s_n)                  # 只需重算邻居的 rhs 与键
```

### 11. D\* · `D_star.py`

反向搜索，用 k_old 判断 RAISE/LOWER 并重新入队。

```text
process_state():
    s ← min_state()                            # OPEN 中 k 最小的状态
    if s = None:
        return -1                              # OPEN 为空，搜索失败
    k_old ← get_k_min()
    delete(s)                                  # 移出 OPEN 并置为 CLOSED
    if k_old < h[s]:                           # RAISE：h 上升
        for s_n ∈ 邻居(s):
            if h[s_n] ≤ k_old and h[s] > h[s_n] + cost(s_n, s):
                PARENT[s] ← s_n                # 就近改父以降低 h[s]
                h[s] ← h[s_n] + cost(s_n, s)
    if k_old = h[s]:                           # LOWER：h 下降
        for s_n ∈ 邻居(s):
            if t[s_n] = 'NEW' or (PARENT[s_n] = s and h[s_n] ≠ h[s] + cost(s, s_n)) or (PARENT[s_n] ≠ s and h[s_n] > h[s] + cost(s, s_n)):
                PARENT[s_n] ← s
                insert(s_n, h[s] + cost(s, s_n))
    else:
        for s_n ∈ 邻居(s):
            if t[s_n] = 'NEW' or (PARENT[s_n] = s and h[s_n] ≠ h[s] + cost(s, s_n)):
                PARENT[s_n] ← s
                insert(s_n, h[s] + cost(s, s_n))
            else:
                if PARENT[s_n] ≠ s and h[s_n] > h[s] + cost(s, s_n):
                    insert(s, h[s])            # OPEN 中 s 代价下降，需重扩
                else:
                    if PARENT[s_n] ≠ s and h[s] > h[s_n] + cost(s_n, s) and t[s_n] = 'CLOSED' and h[s_n] > k_old:
                        insert(s_n, h[s_n])    # CLOSED 邻居代价下降，重入队
    return get_k_min()
```

### 12. D\* Lite · `D_star_Lite.py`

起点移动时用 km 修正键值，无需从头重搜。

```text
ComputePath():
    while True:
        s, v ← TopKey()                        # U 中键最小的节点
        if v ≥ CalculateKey(s_start) and rhs[s_start] = g[s_start]:   # 键含 km 修正
            break                              # 起点已局部一致则收敛
        k_old ← v
        U.pop(s)
        if k_old < CalculateKey(s):            # 键变大：按新键重新入队
            U[s] ← CalculateKey(s)
        elif g[s] > rhs[s]:                    # 过一致：g 可降为 rhs
            g[s] ← rhs[s]
            for x ∈ 邻居(s):
                UpdateVertex(x)
        else:                                  # 欠一致：g 置 ∞ 后重算
            g[s] ← ∞
            UpdateVertex(s)
            for x ∈ 邻居(s):
                UpdateVertex(x)
```

### 13. Anytime D\* · `Anytime_D_star.py`

用 eps 膨胀启发式，逐步收紧到最优解。

```text
ComputeOrImprovePath():
    while True:
        s, v ← TopKey()                        # 键含 eps 膨胀，run 中逐步减到 1
        if v ≥ Key(s_start) and rhs[s_start] = g[s_start]:
            break                              # 起点一致则本轮收敛
        OPEN.pop(s)
        if g[s] > rhs[s]:                      # LOWER：代价下降
            g[s] ← rhs[s]
            CLOSED.add(s)                      # 记入 CLOSED，供 INCONS 判断
            for sn ∈ 邻居(s):
                UpdateState(sn)
        else:                                  # RAISE：代价上升
            g[s] ← ∞
            for sn ∈ 邻居(s):
                UpdateState(sn)
            UpdateState(s)                     # 最后重算 s 自身的 rhs
```

## 采样式规划（10 个）

### 14. RRT · `rrt.py`

单向随机树，每轮只朝采样点扩一步，不选父不重连。

```text
RRT(s_start, s_goal, step_len, goal_sample_rate, iter_max):
    V ← {s_start}
    for i ← 1 to iter_max:
        x_rand ← 采样(goal_sample_rate)          # 小概率直接取 s_goal，是为了有方向性的往终点延展
        x_near ← 最近邻(V, x_rand)
        x_new ← 朝 x_near 走一步(x_rand, step_len)  #取到goal就会往goal走一步
        if x_new ≠ ∅ and 无碰撞(x_near, x_new):
            V.add(x_new)
            if 距离(x_new, s_goal) ≤ step_len and 无碰撞(x_new, s_goal):
                朝 s_goal 扩展一步(x_new)        # 返回值未用，靠父指针回溯
                return 回溯(x_new)
    return ∅
```

**概述：已知障碍物，起点，终点。在迭代次数范围内，随机在地图上采样新点，找到距离最近的节点作为旧点，经过碰撞检测后，旧点向新点方向接近一个步长，有一定几率采样点为终点，以此循环直到到达终点，回溯得到路径。**

### 15. RRT-Connect · `rrt_connect.py`

双向两棵树贪心对冲，碰头即拼接路径，比 RRT 快。

```text
RRT_Connect(s_start, s_goal, step_len, goal_sample_rate, iter_max):
    V1 ← {s_start}, V2 ← {s_goal}
    for i ← 1 to iter_max:
        x_rand ← 采样(goal_sample_rate)          # 交换后仍朝 s_goal 偏置
        x_near ← 最近邻(V1, x_rand)
        x_new ← 朝 x_near 走一步(x_rand, step_len)
        if x_new ≠ ∅ and 无碰撞(x_near, x_new):
            V1.add(x_new)
            x_near2 ← 最近邻(V2, x_new)
            x_new2 ← 朝 x_near2 走一步(x_new, step_len)
            if x_new2 ≠ ∅ and 无碰撞(x_near2, x_new2):
                V2.add(x_new2)
                while not 重合(x_new2, x_new):   # connect：连续直冲
                    x_tmp ← 朝 x_new2 走一步(x_new, step_len)
                    if x_tmp = ∅ or 有碰撞(x_new2, x_tmp):
                        break
                    V2.add(x_tmp)
                    x_new2 ← x_tmp               # 游标前移，父指针重建
            if 重合(x_new2, x_new):              # 坐标严格相等才算相遇
                return 回溯拼接(x_new, x_new2)
        if |V2| < |V1|:
            V1 ↔ V2                              # 下一轮扩展更小的树
    return ∅
```

**概述：已知起点，终点，障碍物，在地图上随机选一个点（有几率选到终点），查找离点最近的起点树节点，起点树节点往选中的点延伸一定的单位，检测无碰撞后选中的点作为起点树的一个新的节点，在终点树中找离这个新节点最近的节点，终点树节点往起点树节点方向延伸一定距离，检测碰撞，若没有碰撞就贪心继续延伸，直到无法前进，如果终点树比起点树短，那么两颗树互换，优先扩展短的树，直到两树相遇，回溯得到完整路径**

### 16. Extended-RRT · `extended_rrt.py`

加障碍后重建整棵树，用旧路径路点偏置采样。

```text
Extended_RRT(s_start, s_goal, step_len, goal_sample_rate, waypoint_sample_rate, iter_max):
    V ← {s_start}, waypoint ← ∅
    while True:                                  # 每次点击加障碍后重来
        for i ← 1 to iter_max:
            if waypoint = ∅:
                x_rand ← 采样(goal_sample_rate)          # 首轮只偏置目标
            else:
                x_rand ← 偏置采样(goal_sample_rate, waypoint_sample_rate)   # 目标 / 路点 / 随机；下标误用 len(path)
            x_near ← 最近邻(V, x_rand)
            x_new ← 朝 x_near 走一步(x_rand, step_len)
            if x_new ≠ ∅ and 无碰撞(x_near, x_new):
                V.add(x_new)
                if 距离(x_new, s_goal) ≤ step_len:       # 末段不查碰撞
                    朝 s_goal 扩展一步(x_new)
                    path ← 回溯(x_new)
                    waypoint ← 回溯节点(x_new)
                    break
        if 未点击加障碍():
            return path
        V ← {s_start}                            # 树整个重建，路点保留
```

**概述：初始规划与rrt相同，但相比于rrt主要是多了重规划部分，地图临时添加障碍物触发重规划，搜索树重置为仅含起点，重规划的采样不再是随机采样，而是按照一定概率分别“朝终点采样”“沿旧路径采样”“随机探索”，随后的步骤与rrt相同（节点搜索、扩展、碰撞检测、到达终点判断）**

### 17. Dynamic-RRT · `dynamic_rrt.py`

保留旧树，标记并剪掉失效枝，只增量续长。

```text
Dynamic_RRT(s_start, s_goal, step_len, goal_sample_rate, waypoint_sample_rate, iter_max):
    V ← {s_start}, E ← ∅, waypoint ← ∅
    while True:                                  # 每次点击加障碍后触发
        for i ← 1 to iter_max:
            x_rand ← 采样(goal_sample_rate)       # 重规划时采样含路点偏置
            x_near ← 最近邻(V, x_rand)
            x_new ← 朝 x_near 走一步(x_rand, step_len)
            if x_new ≠ ∅ and 无碰撞(x_near, x_new):
                V.add(x_new), E.add(边(x_near, x_new))
                if 距离(x_new, s_goal) ≤ step_len:
                    朝 s_goal 扩展一步(x_new)
                    path ← 回溯(x_new)
                    waypoint ← 回溯节点(x_new)
                    break
        if 未点击加障碍():
            return path
        for e ∈ E:                               # 只拿新障碍查一次
            if 与圆相交(e.parent, e.child, obs_add):
                e.child.flag ← "INVALID"         # 只标子节点，边不标
        TrimRRT()                                # 父失效则子树全失效，删掉
        if path 上存在失效路点:
            从残树继续生长()                       # 保留有效旧树，只增量续长
```

**概述：初始规划与rrt相同，相比于rrt主要是多了重规划部分，地图临时添加障碍物触发重规划，在重规划过程中，首先标记被障碍物破坏的节点，如果当前规划出来的路径被障碍物破坏了，就要重新规划路径。首先剔除被破坏的节点，搜索树重置只去掉被破坏的节点，保留其他原有节点，重规划的采样不再是随机采样，而是按照一定概率分别“朝终点采样”“沿旧路径采样”“随机探索”，随后的步骤与rrt相同（节点搜索、扩展、碰撞检测、到达终点判断）**

### 18. RRT\* · `rrt_star.py`

RRT 加"选父"和"重连"两步，采样越多路径越接近最优。

```text
RRT_star(s_start, s_goal, step_len, iter_max):
    V ← {s_start}
    for i ← 1 to iter_max:
        x_rand ← 采样()
        x_near ← 最近邻(V, x_rand)
        x_new ← 朝 x_near 走一步(x_rand, step_len)
        if 无碰撞(x_near, x_new):
            X_near ← 半径 r 内的邻居
            x_new.parent ← argmin_{x ∈ X_near} (x.cost + 距离(x, x_new))   # 选父
            V.add(x_new)
            for x ∈ X_near:                    # 重连
                if x_new.cost + 距离(x_new, x) < x.cost:
                    x.parent ← x_new
    return 回溯(离目标最近且可直连的节点)
```

**概述：已知起点、终点和障碍物，进行随机采样，有概率采样到终点，找到树中离采样点最近的点，进行碰撞检测，无碰撞就查找新节点（新采样点）的邻域，保留半径内且无碰撞的节点，返回符合条件的邻域节点，然后重新选择父节点，选邻域内累计代价（路径长度）最小的父节点，迭代完后，选择可以直达目标点的节点（也就是终点周围的节点），进行碰撞检测，选择代价最小的节点并回溯**

### 19. Informed RRT\* · `informed_rrt_star.py`

找到首条路径后，只在以起点终点为焦点的椭球内采样。

```text
IRrtStar(s_start, s_goal, iter_max):
    V ← {s_start}
    X_soln ← ∅
    c_best ← ∞
    for i ← 1 to iter_max:
        if X_soln ≠ ∅:                             # 有解才更新椭球
            x_best ← X_soln 中 Cost 最小的节点
            c_best ← Cost(x_best)
        x_rand ← 若 c_best = ∞ 则全局采样，否则椭球内采样(c_best)
        x_near ← 最近邻(V, x_rand)
        x_new ← 朝 x_near 走一步(x_rand)
        if x_new and 无碰撞(x_near, x_new):
            X_near ← 近邻(V, x_new)                # 半径 50√(log n/n) 且无碰撞
            V.add(x_new)
            x_new.parent ← X_near 中 Cost + 边长最小的  # 选父
            重连(X_near)                            # 更省则改父
            if 到目标距离 < step_len and 无碰撞(x_new, s_goal):
                X_soln.add(x_new)                  # 下轮迭代才收缩椭球
    return 回溯(x_best)
```

**概述：在没有存在可行解（能够连接到终点的候选节点集合）的时候，先随机采样，有概率采到终点，查找最近邻节点，朝采样点方向扩展一步，检查碰撞，先当作新节点，并计算代价，查找新节点的近邻集合，遍历近邻挑选更优父节点（代价更小），选的是从哪个点连接到新节点代价最小（选新节点的父节点）。如果其他邻域节点把新节点作为父节点时，比原来的代价低，就重新连接。如果新节点进入终点的距离小于步长，检测碰撞，则把这个新节点作为可行解（能够连接到终点的候选节点集合中的一个解）。当存在可行解的时候（能够连接到终点的候选节点集合），遍历每一个候选解节点，计算它从起点走到这个节点的累计路径代价，选取代价最小的节点，因为代价就是路径长度，在构造椭球子集时，椭球的长轴一定比（起点与终点的距离）长，所以将==最小代价作为椭球的长轴==，==短半轴公式==为：**
$$
b = \frac{\sqrt{c_{\text{best最小代价}}^2-c_{起点到终点的距离}^2}}{2}
$$
**之后采样点变为在椭球内采样，进行新一轮循环，随着找到更短的路径，最小代价不断减小，椭球也会随之收缩，搜索范围进一步缩小，从而提高路径优化效率。**

### 20. RRT\*-Smart · `rrt_star_smart.py`

找到初始路径后贪心拉直，并在障碍拐角信标处加密采样。

```text
RrtStarSmart(s_start, s_goal, iter_max):
    V ← {s_start}
    beacons ← ∅
    n ← 0                                          # 初始路径出现的迭代号
    InitPathFlag ← False
    obs_vertex ← 展开障碍顶点()
    for k ← 1 to iter_max:
        x_rand ← 若 (k - n) % 2 = 0 and beacons ≠ ∅ 则信标邻域采样，否则全局采样
        x_near ← 最近邻(V, x_rand)
        x_new ← 朝 x_near 走一步(x_rand)
        if x_new and 无碰撞(x_near, x_new):
            X_near ← 近邻(V, x_new)
            V.add(x_new)
            if X_near ≠ ∅:
                x_new.parent ← X_near 中代价最小的   # 选父
                重连(X_near)                        # 更省则改父
            if not InitPathFlag and 到目标距离 < step_len:
                InitPathFlag ← True
                n ← k
            if InitPathFlag:
                路径优化(x_new)                      # 贪心直连，变短才更新信标
    return 回溯(s_goal)
```

**概述：在没有找到一条能从起点到终点的初始路径时，首先进行带有偏置的全局随机采样，找到最近邻节点，并朝采样点扩散，检查碰撞，将新节点加入树，找新节点的近邻节点，如果存在近邻就选代价最小的近邻为父节点，计算新节点代价，并把周围的节点尝试链接到新节点，如果代价更低就设置为新节点为周围某节点的父节点，如果节点距终点小于步长，就算时找到初始路径。在找到初始路径后，沿着节点往前回溯，如果两个节点直连没有碰上障碍物，就跳过中间节点，将这两个节点直接相连，直到有障碍物阻碍位置，累加代价，就这样继续往前回溯直到回溯到起点，如果代价小于之前算出来的代价（若不是第一次路径回溯），就记录新代价，从终点回溯最优路径，找距离路径节点 3 以内的障碍物顶点，把这些障碍物顶点记录为信标，更新信标集合，只后的全局采样按照周期随机选的信标周围半径的圆形区域内进行，其余迭代仍然进行全局采样，不断优化路径。**

### 21. FMT\*（快速行进树） · `fast_marching_trees.py`

先批量撒点，再沿代价最小前沿做惰性动态规划。

```text
FMT(s_start, s_goal, search_radius):
    V_unvisited ← 自由空间均匀采样(1000)            # 含 s_goal
    V ← V_unvisited ∪ {s_start}
    V_open ← {s_start}
    s_start.cost ← 0
    rn ← search_radius * sqrt(log n / n)           # n 为采样规模
    z ← s_start
    while z ≠ s_goal:
        V_open_new ← ∅
        X_near ← 半径 rn 内的未访问邻居(z)
        for x in X_near:
            Y_near ← 半径 rn 内的开集邻居(x)
            y_min ← Y_near 中 y.cost + Cost(y, x) 最小的  # Y_near 空则报错
            if 无碰撞(y_min, x):
                x.parent ← y_min
                x.cost ← y_min.cost + Cost(y_min, x)
                V_open_new.add(x)
                V_unvisited.remove(x)
        V_open ← V_open ∪ V_open_new - {z}         # z 移入闭集
        if V_open = ∅: break                       # 开集空即失败
        z ← V_open 中 cost 最小的节点
    return 回溯(s_goal)
```

概述：先生成随机自由采样点集合，从起点节点开始，先取未访问集合一定邻域内的采样点，记为邻居点X，记录本次扩展的节点，找到邻居点X在开集中的邻居，即为邻居点Y，计算从起点到邻居点Y的代价加上从邻居点Y到邻居点X的代价（代价小为强者），找到代价最小的节点，检测邻居点X和代价最小点的碰撞，记录该节点为邻居点X的父节点（前驱节点），将该邻居点X加入开集前沿（暂存），将邻居点X从未访问集合中移除，更新邻居点X的累计代价。就这样遍历完所有邻居点X后，把开集前沿中的邻居点X，将本次扩展的节点移出开集，加入闭集合，然后再选择开集合中代价最小的节点继续扩展（通俗的来说就像是一个感染源感染周围一圈的节点，任务完成感染源被耗尽，被感染的节点中的强者作为新的感染源继续去感染周围的节点，然后耗尽，再从没耗尽的感染源中选最强者，以此类推），直到扩展到目标点进行回溯

### 22. BIT\*（批处理知情树） · `batch_informed_trees.py`

批量采样与启发式队列交替，用椭球约束增量搜索。

```text
BITStar(s_start, s_goal, iter_max):
    V ← {s_start}; X_sample ← {s_goal}
    QV ← QE ← ∅
    g_T ← {s_start: 0, s_goal: ∞}
    初始化椭球(cMin, xCenter, C)
    for k ← 0 to 499:                              # 轮数硬编码，iter_max 未用
        if QV = ∅ and QE = ∅:                      # 队列空则开新批次
            m ← 若 k = 0 则 350 否则 200
            剪枝(g_T[s_goal])                       # 按当前最优代价剪枝
            X_sample ← X_sample ∪ 采样(m, g_T[s_goal], 椭球)
            V_old ← V; QV ← V
        while 顶点队列最优值() ≤ 边队列最优值():
            扩展顶点(顶点队列最优顶点())              # 半径固定 4.0，radius() 未调用
        vm, xm ← 边队列最优边()
        if g_T[vm] + dist(vm, xm) + h(xm) < g_T[s_goal]:
            c ← 边真实代价(vm, xm)
            if g_est(vm) + c + h(xm) < g_T[s_goal] and g_T[vm] + c < g_T[xm]:
                if xm ∈ V: 删除所有指向 xm 的旧边
                else: X_sample.remove(xm); V.add(xm); QV.add(xm)
                g_T[xm] ← g_T[vm] + c; E.add((vm, xm)); xm.parent ← vm
                从 QE 剔除无改进的 (v, xm)
        else:
            QV ← QE ← ∅                            # 整批作废，下轮重采样
    return 回溯(s_goal)
```

### 23. Dubins-RRT\* · `dubins_rrt_star.py`

用满足曲率约束的 Dubins 曲线代替直线扩展。

```text
DubinsRRTStar(s_start, s_goal, iter_max):
    V ← {s_start}
    for i ← 1 to iter_max:
        x_rand ← 若未命中目标偏置则随机位姿，否则 s_goal
        x_near ← 最近邻(V, x_rand)
        x_new ← Dubins 曲线连接(x_near, x_rand)      # 终点为新节点，代价加曲线长
        if x_new and 无碰撞(x_new):                  # 碰撞检测只查圆障碍
            X_near ← 近邻(V, x_new)                  # 半径 min(50√(log n/n), step_len)
            x_new ← 选父(x_new, X_near)               # 近邻都连不上则丢弃
            if x_new:
                V.add(x_new)
                重连(x_new, X_near)                  # 更省则替换近邻并传播代价
    i_goal ← 距目标 step_len 内且可安全连接的代价最小节点索引
    if i_goal = None: return 空                      # 源码会索引 None 报错
    return 回溯(i_goal)
```

---

## 附录 A：算法对比

### 搜索式（13 个）

| # | 算法 | 最优性 | 重规划 | 一句话 |
|---|---|---|---|---|
| 1 | BFS | 步数最少 | ✗ | 逐层扩散 |
| 2 | DFS | 不保证 | ✗ | 一条道走到黑 |
| 3 | Best-First | 不保证 | ✗ | 只看 `h`，贪心 |
| 4 | Dijkstra | ✅ 最优 | ✗ | 只看 `g`，均匀铺开 |
| 5 | A\* | ✅ 最优 | ✗ | `g + h`，兼顾两边 |
| 6 | 双向 A\* | ✅ 最优 | ✗ | 两头同时搜，中间碰头 |
| 7 | ARA\* | 有界次优 | ✗ | 权重递减，先快后精 |
| 8 | LRTA\* | 不保证 | 边走边学 | 每步只搜一点，修正 `h` |
| 9 | RTAA\* | 不保证 | 边走边学 | LRTA\* 改进，一次多搜几步 |
| 10 | LPA\* | ✅ 最优 | ✅ 起点变 | 复用上次结果，只改局部 |
| 11 | D\* | ✅ 最优 | ✅ 地图变 | 从终点反向搜，遇障即修 |
| 12 | D\* Lite | ✅ 最优 | ✅ 地图变 | D\* 的简化版 |
| 13 | Anytime D\* | 有界次优 | ✅ 地图变 | D\* Lite + 随时出解 |

### 采样式（10 个）

| # | 算法 | 最优性 | 重规划 | 一句话 |
|---|---|---|---|---|
| 14 | RRT | 不保证 | ✗ | 随机撒点长树 |
| 15 | RRT-Connect | 不保证 | ✗ | 两棵树对着长 |
| 16 | Extended-RRT | 不保证 | ✅ | 一次朝一个方向连走几步 |
| 17 | Dynamic-RRT | 不保证 | ✅ | 环境变了砍枝重长 |
| 18 | RRT\* | 渐近最优 | ✗ | 加"选父"和"重连" |
| 19 | Informed RRT\* | 渐近最优 | ✗ | 只在椭球内采样 |
| 20 | RRT\*-Smart | 渐近最优 | ✗ | 路径优化 + 拐角采样 |
| 21 | FMT\* | 渐近最优 | ✗ | 批量采样，一次连图 |
| 22 | BIT\* | 渐近最优 | ✗ | 边采样边搜索 |
| 23 | Dubins-RRT\* | 渐近最优 | ✗ | 用 Dubins 曲线连接 |

> **渐近最优**：采样越多越接近最优，但有限时间内不保证最优。

### 其他事实

- `rrt_2D/` 下有 3 个 0 字节空文件，没有实现：`adaptively_informed_trees.py`（AIT\*）、`advanced_batch_informed_trees.py`（ABIT\*）、`rrt_sharp.py`（RRT#）。
- README 列的 *Anytime RRT\*、Closed-Loop RRT\*、Spline-RRT\** 在仓库里并不存在；实际存在的 **Dubins-RRT\*** 反而没被列出。
- `env.py`、`plotting.py`、`queue.py`、`utils.py` 是公共工具，不是算法。

---

## 附录 B：运行命令

在项目根目录执行，弹窗后按 `Esc` 关闭。

**搜索式**（直接跑脚本）：

```bash
python3 Search_based_Planning/Search_2D/Astar.py          # 换成任意文件名
```

**采样式**（必须用 `-m`，因为源码里是绝对包名导入）：

```bash
python3 -m Sampling_based_Planning.rrt_2D.rrt_star        # 换成任意文件名
```

等价写法：`PYTHONPATH=. python3 Sampling_based_Planning/rrt_2D/rrt_star.py`

**存成图片**（无图形界面时）：

```bash
MPLBACKEND=Agg python3 -c '
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.show = lambda *a, **k: plt.savefig("out.png", dpi=95, bbox_inches="tight")
import runpy; runpy.run_path("Sampling_based_Planning/rrt_2D/rrt_star.py", run_name="__main__")
'
```

**依赖**：`pip install numpy matplotlib scipy`（3D 另需 `pyrr`，2D 用不到）
