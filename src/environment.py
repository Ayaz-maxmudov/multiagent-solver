import random
from typing import List, Tuple
from src.graph import Graph
from src.config import MAX_PICKUP_COORD, MAX_DELIVERY_COORD

class Environment:
    def __init__(self):
        self.graph = Graph()
        self.coordinates = []
        self._build_grid_graph()
        self.blocked_edges: List[Tuple[Tuple[int,int], float, float]] = []

    def _build_grid_graph(self):
        size_x = MAX_PICKUP_COORD + 1
        size_y = MAX_DELIVERY_COORD + 1
        self.coord_map = {}
        self.reverse_map = {}
        vertex_id = 0
        for x in range(size_x):
            for y in range(size_y):
                self.coord_map[vertex_id] = (x, y)
                self.reverse_map[(x, y)] = vertex_id
                self.coordinates.append((x, y))
                vertex_id += 1
        for vid, (x, y) in self.coord_map.items():
            if x + 1 < size_x:
                u = vid
                v = self.reverse_map[(x + 1, y)]
                self.graph.add_edge(u, v, 1)
            if y + 1 < size_y:
                u = vid
                v = self.reverse_map[(x, y + 1)]
                self.graph.add_edge(u, v, 1)

    def random_block_edge(self, current_time: float, duration: float = 10.0):
        all_edges = []
        for u in self.graph.adj:
            for v, _ in self.graph.adj[u]:
                if u < v:
                    all_edges.append((u, v))
        if not all_edges:
            return
        edge = random.choice(all_edges)
        self.blocked_edges.append((edge, current_time, current_time + duration))

    def is_edge_blocked(self, u: int, v: int, time: float) -> bool:
        e = (u, v) if u < v else (v, u)
        for (edge_uv, t_start, t_end) in self.blocked_edges:
            if edge_uv == e and t_start <= time <= t_end:
                return True
        return False

    def get_neighbors(self, u: int, time: float) -> List[Tuple[int, float]]:
        nbrs = []
        for v, w in self.graph.neighbors(u):
            if not self.is_edge_blocked(u, v, time):
                nbrs.append((v, w))
        return nbrs

    def shortest_path(self, start: int, target: int, time: float):
        INF = float('inf')
        dist = {v: INF for v in self.graph.adj}
        prev = {v: None for v in self.graph.adj}
        dist[start] = 0
        import heapq
        heap = [(0, start)]
        while heap:
            d_u, u = heapq.heappop(heap)
            if d_u > dist[u]:
                continue
            if u == target:
                break
            for v, w in self.get_neighbors(u, time):
                alt = d_u + w
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u
                    heapq.heappush(heap, (alt, v))
        if dist[target] == INF:
            return None, INF
        path = []
        cur = target
        while cur is not None:
            path.append(cur)
            cur = prev[cur]
        path.reverse()
        return path, dist[target]
