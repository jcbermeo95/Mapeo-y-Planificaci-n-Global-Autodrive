
"""
@file: d_star.py
@author: Wu Maojia and Gemini Google
@update: 2025.10.6
"""
from typing import Union, List, Tuple, Dict, Any
import heapq

from python_motion_planning.common import Node
from python_motion_planning.path_planner.graph_search.dijkstra import Dijkstra


class DNode(Node):
    """
    Class for D* nodes compatible with the updated graph search framework.
    """
    def __init__(self, current: tuple, parent: tuple, t: str, g: float, h: float, k: float) -> None:
        super().__init__(current, parent, g, h)
        self.t = t
        self.k = k

    def __lt__(self, other):
        return self.k < other.k


class DStar(Dijkstra):
    """
    Class for D* path planner updated to match the current framework structure.
    """
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    def __str__(self) -> str:
        return "Dynamic A*(D*)"

    def plan(self) -> Union[List[Tuple[float, ...]], Dict[str, Any]]:
        """
        Interface for D* planning.
        """
        # Initialize map nodes graph dictionary using integer-converted bounds
        self.map_nodes = {}
        x_min, x_max = int(self.map_.bounds[0][0]), int(self.map_.bounds[0][1])
        y_min, y_max = int(self.map_.bounds[1][0]), int(self.map_.bounds[1][1])

        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                pt = (x, y)
                self.map_nodes[pt] = DNode(pt, None, 'NEW', float("inf"), self.get_heuristic(pt), float("inf"))

        start_node = self.map_nodes[self.start]
        goal_node = self.map_nodes[self.goal]

        start_node.h = self.get_heuristic(self.start)
        goal_node.h = 0
        goal_node.g = 0

        OPEN = []
        self.insert(OPEN, goal_node, 0)

        while OPEN:
            k_old, node = self.min_state(OPEN)
            if node is None:
                break
            
            self.delete(OPEN, node)

            if k_old < node.h:
                for node_n in self.get_neighbors(node):
                    if node_n.h <= k_old and node.h > node_n.h + self.get_cost(node.current, node_n.current):
                        node.parent = node_n.current
                        node.h = node_n.h + self.get_cost(node.current, node_n.current)

            if k_old == node.h:
                for node_n in self.get_neighbors(node):
                    if node_n.t == 'NEW' or \
                       (node_n.parent == node.current and node_n.h != node.h + self.get_cost(node.current, node_n.current)) or \
                       (node_n.parent != node.current and node_n.h > node.h + self.get_cost(node.current, node_n.current)):
                        node_n.parent = node.current
                        self.insert(OPEN, node_n, node.h + self.get_cost(node.current, node_n.current))
            else:
                for node_n in self.get_neighbors(node):
                    if node_n.t == 'NEW' or \
                       (node_n.parent == node.current and node_n.h != node.h + self.get_cost(node.current, node_n.current)):
                        node_n.parent = node.current
                        self.insert(OPEN, node_n, node.h + self.get_cost(node.current, node_n.current))
                    else:
                        if node_n.parent != node.current and \
                           node_n.h > node.h + self.get_cost(node.current, node_n.current):
                            self.insert(OPEN, node, node.h)
                        else:
                            if node_n.parent != node.current and \
                               node.h > node_n.h + self.get_cost(node.current, node_n.current) and \
                               node_n.t == 'CLOSED' and \
                               node_n.h > k_old:
                                self.insert(OPEN, node_n, node_n.h)

            if start_node.t == 'CLOSED':
                break

        CLOSED = {pos: nd for pos, nd in self.map_nodes.items() if nd.t == 'CLOSED'}
        CLOSED[self.goal] = goal_node

        path = []
        cost = 0
        curr = self.start
        
        while curr != self.goal and curr in self.map_nodes:
            path.append(curr)
            node_obj = self.map_nodes[curr]
            if node_obj.parent is None:
                break
            next_curr = node_obj.parent
            cost += self.get_cost(curr, next_curr)
            curr = next_curr
        path.append(self.goal)

        length = len(path)

        return path, {
            "success": len(path) > 1,
            "start": self.start,
            "goal": self.goal,
            "length": length,
            "cost": cost,
            "expand": CLOSED
        }

    def get_neighbors(self, node: DNode) -> list:
        neighbors = []
        for n_base in self.map_.get_neighbors(node, diagonal=self.diagonal):
            if n_base.current in self.map_nodes:
                neighbors.append(self.map_nodes[n_base.current])
        return neighbors

    def min_state(self, OPEN: list) -> tuple:
        if not OPEN:
            return -1, None
        node = min(OPEN, key=lambda n: n.k)
        return node.k, node

    def insert(self, OPEN: list, node: DNode, h_new: float) -> None:
        if node.t == 'NEW':
            node.k = h_new
        elif node.t == 'OPEN':
            node.k = min(node.k, h_new)
        elif node.t == 'CLOSED':
            node.k = min(node.h, h_new)
        node.h, node.t = h_new, 'OPEN'
        if node in OPEN:
            OPEN.remove(node)
        heapq.heappush(OPEN, node)

    def delete(self, OPEN: list, node: DNode) -> None:
        if node.t == 'OPEN':
            node.t = 'CLOSED'
        if node in OPEN:
            OPEN.remove(node)
            heapq.heapify(OPEN)
