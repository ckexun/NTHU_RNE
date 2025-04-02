import cv2
import sys
sys.path.append("..") # 將該路徑加入sys.path中，使得可以import該路徑下的module
import PathPlanning.utils as utils
from PathPlanning.planner import Planner


class PlannerAStar(Planner):
    def __init__(self, m, inter=10):
        super().__init__(m)
        self.inter = inter
        self.initialize()

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.g = {} # Distance from start to node   (path cost)
        self.h = {} # Distance from node to goal    (heuristic)
        self.goal_node = None

    """     Append    """
    def _check_collision(self, p1):
        # 檢查點p1是否在地圖外或是碰到障礙物
        if p1[0]<0 or p1[0]>=self.map.shape[1]:
            return True
        elif p1[1]<0 or p1[1]>=self.map.shape[0]:
            return True
        elif self.map[p1[1], p1[0]]!=1:
            return True
        return False

    def _queue_insert(self, node):
        if len(self.queue) == 0:
            self.queue.append(node)
            return

        f = self.g[node] + self.h[node]
        for i in range(len(self.queue)):
            n = self.queue[i]
            if (self.g[n] + self.h[n]) > f:
                self.queue.insert(i, node)
                return
        self.queue.append(node)


    def planning(self, start=(100,200), goal=(375,520), inter=None, img=None):
        if inter is None:
            inter = self.inter
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        # Initialize 
        self.initialize()
        self.queue.append(start)
        self.parent[start] = None
        self.g[start] = 0
        self.h[start] = utils.distance(start, goal)

        while(1):
            # TODO: A Star Algorithm

            if len(self.queue) == 0:
                break

            current_node = self.queue.pop(0)
            if current_node == goal:
                self.goal_node = current_node
                break


            # 8-neighbors
            neighbors = [(-1, 1), (0, 1), (1, 1),
                        (-1, 0),         (1, 0),
                        (-1, -1), (0, -1), (1, -1)]
            for n in neighbors:
                neighbor_node = (current_node[0]+n[0]*inter, current_node[1]+n[1]*inter)
                # 坐標系為正常的坐標系(img先經過flip處理)
                if self._check_collision(neighbor_node):
                    continue
                cost = self.g[current_node] + utils.distance(current_node, neighbor_node)
                if neighbor_node not in self.g or self.g[neighbor_node] > cost:
                    self.g[neighbor_node] = cost
                    self.h[neighbor_node] = utils.distance(neighbor_node, goal)
                    self.parent[neighbor_node] = current_node
                    self._queue_insert(neighbor_node)
        
        # Extract path
        path = []
        p = self.goal_node
        if p is None:
            return path
        while(True):
            path.insert(0,p)
            if self.parent[p] is None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        """     Append    """
        print("Total path cost:", self.g[path[-1]])
        """     Append    """
        return path
