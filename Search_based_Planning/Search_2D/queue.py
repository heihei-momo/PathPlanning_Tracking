import collections  # 提供双端队列
import heapq  # 提供堆优先队列


class QueueFIFO:  # 先进先出队列，供 BFS 使用
    """
    Class: QueueFIFO
    Description: QueueFIFO is designed for First-in-First-out rule.
    """

    def __init__(self):  # 初始化空队列
        self.queue = collections.deque()  # 存储待扩展节点

    def empty(self):  # 判断队列是否为空
        return len(self.queue) == 0  # 为空返回 True

    def put(self, node):  # 节点从队尾入队
        self.queue.append(node)  # enter from back

    def get(self):  # 节点从队首出队
        return self.queue.popleft()  # leave from front


class QueueLIFO:  # 后进先出队列，供 DFS 使用
    """
    Class: QueueLIFO
    Description: QueueLIFO is designed for Last-in-First-out rule.
    """

    def __init__(self):  # 初始化空队列
        self.queue = collections.deque()  # 存储待扩展节点

    def empty(self):  # 判断队列是否为空
        return len(self.queue) == 0  # 为空返回 True

    def put(self, node):  # 节点从队尾入队
        self.queue.append(node)  # enter from back

    def get(self):  # 节点从队尾出队，实现后进先出
        return self.queue.pop()  # leave from back


class QueuePrior:  # 优先队列，按优先级排序
    """
    Class: QueuePrior
    Description: QueuePrior reorders elements using value [priority]
    """

    def __init__(self):  # 初始化空堆
        self.queue = []  # 堆中存放(优先级, 元素)

    def empty(self):  # 判断优先队列是否为空
        return len(self.queue) == 0  # 为空返回 True

    def put(self, item, priority):  # 按优先级插入元素
        heapq.heappush(self.queue, (priority, item))  # reorder s using priority

    def get(self):  # 弹出优先级最小的元素
        return heapq.heappop(self.queue)[1]  # pop out the smallest item

    def enumerate(self):  # 返回内部堆列表
        return self.queue  # 便于外部查看队列内容
