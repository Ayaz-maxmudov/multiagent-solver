import heapq

class Graph:
    def __init__(self):
        self.adj = {}

    def add_edge(self, u, v, w):
        if u not in self.adj:
            self.adj[u] = []
        if v not in self.adj:
            self.adj[v] = []
        self.adj[u].append((v, w))
        self.adj[v].append((u, w))

    def neighbors(self, u):
        return self.adj.get(u, [])

    def vertices(self):
        return list(self.adj.keys())

    def dijkstra(self, start):
        INF = float('inf')
        dist = {v: INF for v in self.adj}
        prev = {v: None for v in self.adj}
        dist[start] = 0
        heap = [(0, start)]
        while heap:
            d_u, u = heapq.heappop(heap)
            if d_u > dist[u]:
                continue
            for v, w in self.neighbors(u):
                alt = d_u + w
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u
                    heapq.heappush(heap, (alt, v))
        return dist, prev

    def shortest_path(self, start, target):
        dist, prev = self.dijkstra(start)
        if dist.get(target, float('inf')) == float('inf'):
            return None, float('inf')
        path = []
        cur = target
        while cur is not None:
            path.append(cur)
            cur = prev[cur]
        path.reverse()
        return path, dist[target]
