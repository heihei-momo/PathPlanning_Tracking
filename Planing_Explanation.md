

# 常见路径规划算法

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

## 搜索式规划

### BFS 广度优先搜索 · `bfs.py`

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

**概述：首次访问的邻居代价都会初始化为无穷，取起点（访问点）的八领域点，下一步珊格的代价为自身已有代价+到新邻域的欧式距离代价，碰撞代价为无穷，更新邻居点的代价，记录父节点，加入列表队尾。再按顺序弹出最早入队的节点作为访问点，继续八邻域扩散，一直往四周扩散直到遇到终点，回溯路径。**

### DFS 深度优先搜索 · `dfs.py`

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

**概述：bfs是先进先出，所以和波浪一样均匀向外传播，dfs其他都一样，但是是后进先出所以不会像波浪，像一根藤蔓沿着一个方向不断生长，直到扩展到终点，再回溯。**

### Best-First 最佳优先搜索 · `Best_First.py`

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

**概述：和dfs和bfs的区别是把优先级改为了启发式，曼哈顿距离或者欧式距离作为代价，代价小的放优先级的前面，先取出扩展，直到终点回溯。**

### Dijkstra 算法 · `Dijkstra.py`

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

**概述：和Best-First的区别是Best-First只计算入队点离终点的曼哈顿作为优先代价，Dijkstra按照代价来排优先级，如果每走一步的代价都完全相同，那么 BFS 和 Dijkstra完全相同。**

### A\* 算法 · `Astar.py`

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

**概述：把Dijkstra的代价变为了g+h，往前走一步的代价+曼哈顿/欧式距离启发式代价（g+h），以代价最小进行扩展，直到到终点再回溯。**

### 双向 A\* · `Bidirectional_a_star.py`

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

**概述：相当于两个A*，一个从起点触发，一个从终点触发，都用对方当启发式目标，若两个列表里有重合的点，代表相遇，然后回溯获得完整路径。**

### ARA\* · `ARAstar.py`

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

**概述：先类似A_star用启发式代价进行扩展，但是给h加了一个系数：f=g+e*h。先把e往大了取，故意把每个候选节点的代价放大，可能提前忽略实际上能产生更优路径的节点，在搜索过程中路径会更快速的朝着目标走，会更快的到达目标，但是是次优解。第一次寻找路径相比A_star会少扩展很多节点。在有一条次优路径后，降低权重e，把之前发现但没重新搜索的节点放回来，重新计算优先级，继续搜索，直到update_e() <=1（根据当前搜索边界的信息，已经没有必要继续进行 e>1 的优化阶段；在满足 ARA_star 的启发式条件时，继续到 e=1 就可以获得最优性保证），结束。**

**A* 适合“我要最终最优解”。ARA* 适合“我要尽快有一个能用的解，并且有时间就继续优化”。**

### LRTA\* · `LRTAstar.py`

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

**概述：先用启发式代价更新一张全局的h表，用A_star更新N步，现在已经有节点表中了，需要更新这些节点的h值，进行迭代，对于当前节点 `s`，把它所有“下一步能走的邻居”都算一遍“走到邻居的代价 + 邻居到终点的估计代价”，然后取最小值，作为 `s` 新的 h 值。这一轮更新之后，所有节点的 `h` 值都没有再发生变化，那么就说明本轮学习已经收敛，可以停止迭代。把学到的 h 值写回全局表，在当前查询点邻域查找，找到表中最小的h值的点作为新一轮的起点，再从起点出发进行扩展，直到找到终点，通过代价找到代价最小的一条路径作为路径。**

### RTAA\* · `RTAAStar.py`

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

**概述：先A_star进行扩展一波，在open列表中找到一个代价最小的节点，将它作为基准（标记点），用 OPEN 中最小的 `f` 值（标记点）减去每个 CLOSED 节点自己的 `g` 值（因为希望更新后满足 `g+h=f_min`），将更新完的h值写回全局 h 表，再从标记点按最大的h（代表g小）回溯路径到上一个起点，然后标记点作为新起点。**

### LPA\* · `LPAstar.py`

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

**概述：LPA*第一次寻找最短路径时，先将起点的 rhs=0 放入OPEN(U)，然后不断从U中取出Key最小的节点进行处理；如果该节点的rhs比当前g更小，就用rhs更新g，并根据这个新的g去更新所有邻居的rhs，把受到影响且g≠rhs的节点重新放入U，这样搜索信息就从起点一层层向目标传播；当目标节点已经满足g=rhs，且U中不存在Key更小、可能继续改善目标的节点时，搜索停止，此时目标的g就代表从起点到目标的最短代价，最后再从目标出发，每次选择g最小的邻居反向回溯到起点，得到完整的最短路径。**

**在动态的加入障碍物后，更新受影响的节点，重新计算它们的 rhs和 Key，并将不一致的节点放入 U，先处理U中代价最小的节点，从受影响区域向外传播修复，当目标节点已经达到一致状态 g=rhs，同时 U 中已经没有 Key 比目标更小、能够继续改善目标的节点时，修复结束。**

**g      → 当前保存的路径代价**
**rhs    → 根据当前邻居计算出来的“邻居的代价g+到自己的路径代价”**
**U      → g 和 rhs 不一致、需要处理的节点集合**

### D\* · `D_star.py`

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

**概述：从终点开始反向搜索，先将终点放入 OPEN，然后逐步取出代价最小的节点进行处理。处理当前节点 `s` 时，遍历它的所有可行邻居 `s_n`，逐个判断邻居的状态，并计算“经过当前节点到达邻居”的代价，即当前节点的 `h` 值加上 `s` 到 `s_n` 的移动代价，再与邻居原来的 `h` 值进行比较。如果邻居还没有被处理，或者经过当前节点可以得到更小的代价，就将当前节点作为邻居的父节点，更新邻居的 `h` 值，并将邻居加入 OPEN，使新的代价继续向外传播。这样从终点逐层向外扩展，最终将代价传播到起点，并通过父节点关系得到从起点到终点的最短路径。**

**当原来的路径上出现新的障碍物导致部分路径失效时，D* 不需要重新从头搜索，而是对受到影响的节点进行局部修复。算法重新检查受影响节点及其邻居，判断原来的父节点是否仍然可用，并重新比较经过其他邻居的路径代价。如果发现原来的路径代价变大或者路径已经无法通行，就寻找其他能够提供更小代价的邻居作为新的父节点，并更新对应的 `h` 值；如果某些已经处理过的节点也受到影响，就将它们重新放入 OPEN，让代价变化继续向周围传播。最终只修复受到障碍物变化影响的区域，并重新得到一条可行的最短路径。**

**h是一个节点到终点的“当前路径代价”**

## 采样式规划

### RRT · `rrt.py`

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

### RRT-Connect · `rrt_connect.py`

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

### Extended-RRT · `extended_rrt.py`

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

### Dynamic-RRT · `dynamic_rrt.py`

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

### RRT\* · `rrt_star.py`

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

### Informed RRT\* · `informed_rrt_star.py`

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

### RRT\*-Smart · `rrt_star_smart.py`

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

### FMT\*（快速行进树） · `fast_marching_trees.py`

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

**概述：先生成随机自由采样点集合，从起点节点开始，先取未访问集合一定邻域内的采样点，记为邻居点X，记录本次扩展的节点，找到邻居点X在开集中的邻居，即为邻居点Y，计算从起点到邻居点Y的代价加上从邻居点Y到邻居点X的代价（代价小为强者），找到代价最小的节点，检测邻居点X和代价最小点的碰撞，记录该节点为邻居点X的父节点（前驱节点），将该邻居点X加入开集前沿（暂存），将邻居点X从未访问集合中移除，更新邻居点X的累计代价。就这样遍历完所有邻居点X后，把开集前沿中的邻居点X，将本次扩展的节点移出开集，加入闭集合，然后再选择开集合中代价最小的节点继续扩展（通俗的来说就像是一个感染源感染周围一圈的节点，任务完成感染源被耗尽，被感染的节点中的强者作为新的感染源继续去感染周围的节点，然后耗尽，再从没耗尽的感染源中选最强者，以此类推），直到扩展到目标点进行回溯。**

### BIT\*（批处理知情树） · `batch_informed_trees.py`

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

**概述：==如果当前没有一条到终点的可行路径==，先全局随机采一批点，刚开始只有起点队列，QV代价为起点到终点的欧式距离，也没有QE待处理边所以先直接扩展起点，找到待扩展点半径内的采样点，如果代价小于当前终点的代价，先将这些采样点的代价设置为无穷，作为候选边入队，如果本次扩展的顶点没有在旧顶点集里面，就需要找半径内的节点，如果加上这个新的节点后，与周围的节点相连，代价比可行路径（终点的代价）小，并且新点的代价加上边的代价，比原来这个邻居的节点的代价还小，就加入候选边队列，如果这个过关的邻居还没有代价就先设置为无穷大。遍历候选边队列，选择返回当前扩展点真实代价+c+h （当前扩展点真实代价+边长+与终点的欧式距离）最小的边，把对应的端点从边队列中移除，<u>[如果 待扩展点真实代价+c+h小于终点的代价，就说明这条路有希望比终点的代价更低，检测边端点连线的碰撞，再判断通过 vm 到达 xm（两个端点，vm是扩展点，xm是邻居点），会不会比 xm 当前的路径更短，如果更短说明可以优化，，再看邻居点xm是不是已经在树顶点集合，如果在的话既然已经发现了更好的父节点vm，就要删除xm的旧边，如果xm不在树顶点队列里面，就把他加入树顶点和树待扩展点的队列，更新xm的真实代价，新边入树，连接到vm。因为xm的真实代价被更新了，所以一些连接xm的，比这个真实代价大的边需要去除，已经不可能比xm现有的连接更好了]</u>，<u>[待扩展点真实代价+c+h大于终点的代价，就丢弃]</u>。因为终点也被放入采样点里面，所以等到xm是终点的时候，终点就入树顶点了，也就有真实代价了。**

**==如果找到了一条到终点的可行路径==，先剪枝，剔除无望采样点、顶点、边。采样将不在是全局随机采样，而是被圈定在一个椭球内，和Informed RRT\*一样，找到可行路径以后，BIT* 不是停止从起点扩展，而是进入“基于已有树的优化阶段”。它仍然会扩展新采样点，但主要目标变成优化已有树的连接关系，降低路径代价。**

### Dubins-RRT\* · `dubins_rrt_star.py`

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

**概述：与之前的rrt_star不一样的是它的起点和终点都是有方向的，把直线连接替换成了Dubins曲线，但在优化近邻父节点的时候只用Dubins检测了碰撞，代价更新用的是直线距离，找到终点的安全候选点后，取代价最小的安全候选点，终点作为最后一个点，再从最小的安全候选点用Dubins曲线回溯，形成完整路径。**

**流程与rrt_star完全一样，除了优化近邻父节点的时候只用Dubins检测了碰撞，代价更新用的是直线距离（复用rrt_star了），其他把直线连接的地方全换Dubins曲线就是Dubins-RRT_star**


## 附录 A：算法对比

### 搜索式

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

### 采样式

| # | 算法 | 最优性 | 重规划 | 一句话 |
|---|---|---|---|---|
| 12 | RRT | 不保证 | ✗ | 随机撒点长树 |
| 13 | RRT-Connect | 不保证 | ✗ | 两棵树对着长 |
| 14 | Extended-RRT | 不保证 | ✅ | 一次朝一个方向连走几步 |
| 15 | Dynamic-RRT | 不保证 | ✅ | 环境变了砍枝重长 |
| 16 | RRT\* | 渐近最优 | ✗ | 加"选父"和"重连" |
| 17 | Informed RRT\* | 渐近最优 | ✗ | 只在椭球内采样 |
| 18 | RRT\*-Smart | 渐近最优 | ✗ | 路径优化 + 拐角采样 |
| 19 | FMT\* | 渐近最优 | ✗ | 批量采样，一次连图 |
| 20 | BIT\* | 渐近最优 | ✗ | 边采样边搜索 |
| 21 | Dubins-RRT\* | 渐近最优 | ✗ | 用 Dubins 曲线连接 |

> **渐近最优**：采样越多越接近最优，但有限时间内不保证最优。

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
