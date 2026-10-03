import heapq, time
from .distance_service import haversine
from .dijkstra import path

def solve(adj, nodes, s, t):
    """A* with f = g + h, h = Haversine distance (admissible: road length >= straight line)."""
    t0 = time.perf_counter()
    g, prev, closed, order, trace, edges = {s: 0.0}, {}, set(), [], [], 0
    h = lambda n: haversine(nodes[n], nodes[t])
    pq = [(h(s), s)]
    while pq:
        _, u = heapq.heappop(pq)
        if u in closed: continue
        closed.add(u); order.append(u); trace.append(["v", u])
        if u == t: break
        for v, w in adj[u]:
            edges += 1; trace.append(["e", u, v])
            if g[u] + w < g.get(v, float("inf")):
                g[v] = g[u] + w; prev[v] = u; heapq.heappush(pq, (g[v] + h(v), v))
    ms = (time.perf_counter() - t0) * 1000
    if t not in g: raise ValueError(f"No road path from {s} to {t}")
    return {"algorithm": "A*", "path": path(prev, s, t), "distance_km": round(g[t], 3),
            "nodes_explored": len(order), "edges_checked": edges, "execution_ms": round(ms, 3),
            "trace": trace, "time_complexity": "O(E log V) worst case; depends on heuristic",
            "space_complexity": "O(V)"}
