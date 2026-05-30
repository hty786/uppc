#!/usr/bin/python3
"""
R2 树林路径规划器
基于 KFS 布局 (从 R1 红外信息获得) 规划 R2 在树林方块上的最优收集路径
"""
import heapq
from typing import List, Dict, Set, Tuple, Optional
import numpy as np


class ForestPlanner:
    """
    树林方块图结构:
      入口区
      ┌─┬─┬─┬─┐
      │1│2│3│4│    编号方案: 1-12 或自定义
      │5│6│7│8│    方块间有公共边即为相邻
      │9│10│11│12│
      └─┴─┴─┴─┘
    """
    FOREST_LAYOUT: Dict[int, List[int]] = {
        1: [2, 5],         2: [1, 3, 6],      3: [2, 4, 7],
        4: [3, 8],         5: [1, 6, 9],       6: [2, 5, 7, 10],
        7: [3, 6, 8, 11],  8: [4, 7, 12],      9: [5, 10],
        10: [6, 9, 11],    11: [7, 10, 12],    12: [8, 11],
    }

    ENTRANCE_BLOCKS: List[int] = [1, 2, 3]

    def __init__(self, adjacency: dict = None):
        self.graph = adjacency or self.FOREST_LAYOUT

    def shortest_path(self, start: int, goal: int) -> List[int]:
        """Dijkstra 最短路径"""
        if start == goal:
            return [start]
        dist = {start: 0}
        prev: Dict[int, Optional[int]] = {start: None}
        pq = [(0, start)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, float('inf')):
                continue
            if u == goal:
                break
            for v in self.graph[u]:
                nd = d + 1
                if nd < dist.get(v, float('inf')):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        if goal not in prev:
            return []
        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = prev[cur]
        path.reverse()
        return path

    def plan_route(self, start: int, targets: List[int]) -> List[int]:
        """
        规划收集路线: 从 start 出发，访问所有 targets (R2 KFS 所在方块)
        使用最近邻贪心算法
        """
        unvisited = set(targets)
        route = []
        cur = start
        while unvisited:
            best_target = min(unvisited, key=lambda t: len(self.shortest_path(cur, t)))
            path_seg = self.shortest_path(cur, best_target)
            if not path_seg:
                break
            route.extend(path_seg[:-1])  # 不重复最后方块
            cur = best_target
            unvisited.remove(cur)
        # 加上最终目的地（最近的出口）
        exits = self.ENTRANCE_BLOCKS
        exit_block = min(exits, key=lambda e: len(self.shortest_path(cur, e)))
        exit_path = self.shortest_path(cur, exit_block)
        if exit_path:
            route.extend(exit_path)
        # 去重连续重复
        deduped = [route[0]]
        for b in route[1:]:
            if b != deduped[-1]:
                deduped.append(b)
        return deduped

    def block_distance(self, a: int, b: int) -> int:
        """两方块间曼哈顿块数"""
        path = self.shortest_path(a, b)
        return max(0, len(path) - 1)


def test():
    planner = ForestPlanner()
    print(planner.shortest_path(1, 12))
    print(planner.plan_route(1, [6, 10, 3]))
    print(planner.block_distance(1, 12))


if __name__ == '__main__':
    test()
