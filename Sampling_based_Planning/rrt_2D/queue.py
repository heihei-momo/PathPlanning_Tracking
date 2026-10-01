import collections
import heapq


class QueueFIFO:  # 先进先出队列
    """
    Class: QueueFIFO
    Description: QueueFIFO is designed for First-in-First-out rule.
    """

    def __init__(self):  # 初始化双端队列
        self.queue = collections.deque()  # 用双端队列存节点

    def empty(self):  # 队列是否为空
        return len(self.queue) == 0  # 空则返回真

    def put(self, node):  # 入队
        self.queue.append(node)  # enter from back

    def get(self):  # 出队
        return self.queue.popleft()  # leave from front


class QueueLIFO:  # 后进先出队列（栈）
    """
    Class: QueueLIFO
    Description: QueueLIFO is designed for Last-in-First-out rule.
    """

    def __init__(self):  # 初始化双端队列
        self.queue = collections.deque()  # 用双端队列存节点

    def empty(self):  # 队列是否为空
        return len(self.queue) == 0  # 空则返回真

    def put(self, node):  # 入栈
        self.queue.append(node)  # enter from back

    def get(self):  # 出栈
        return self.queue.pop()  # leave from back


class QueuePrior:  # 优先队列（最小堆）
    """
    Class: QueuePrior
    Description: QueuePrior reorders elements using value [priority]
    """

    def __init__(self):  # 初始化堆列表
        self.queue = []  # 最小堆存 (优先级, 元素)

    def empty(self):  # 队列是否为空
        return len(self.queue) == 0  # 空则返回真

    def put(self, item, priority):  # 按优先级入堆
        heapq.heappush(self.queue, (priority, item))  # reorder s using priority

    def get(self):  # 取出优先级最小者
        return heapq.heappop(self.queue)[1]  # pop out the smallest item

    def enumerate(self):  # 返回内部堆列表
        return self.queue  # 供外部遍历
