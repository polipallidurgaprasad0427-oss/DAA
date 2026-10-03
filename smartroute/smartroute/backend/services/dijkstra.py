import heapq, time

def run(adj, s, t=None):
    """Binary-heap Dijkstra. Returns dist, prev, visit order, edges checked, trace."""
    dist, prev, done, order, trace, edges = {s: 0.0}, {}, set(), [], [], 0
    pq = [(0.0, s)]
    while pq:
        d, u = heapq.heappop(pq)
        if u in done: continue
        done.add(u); order.append(u); trace.append(["v", u])
        if u == t: break
        for v, w in adj[u]:
            edges += 1; trace.append(["e", u, v])
            if d + w < dist.get(v, float("inf")):
                dist[v] = d + w; prev[v] = u; heapq.heappush(pq, (d + w, v))
    return dist, prev, order, edges, trace

def path(prev, s, t):
    p = [t]
    while p[-1] != s: p.append(prev[p[-1]])
    return p[::-1]

def solve(adj, s, t):
    t0 = time.perf_counter()
    dist, prev, order, edges, trace = run(adj, s, t)
    ms = (time.perf_counter() - t0) * 1000
    if t not in dist: raise ValueError(f"No road path from {s} to {t}")
    return {"algorithm": "Dijkstra", "path": path(prev, s, t), "distance_km": round(dist[t], 3),
            "nodes_explored": len(order), "edges_checked": edges, "execution_ms": round(ms, 3),
            "trace": trace, "time_complexity": "O((V + E) log V)", "space_complexity": "O(V + E)"}
