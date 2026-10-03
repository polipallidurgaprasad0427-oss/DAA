import time

def solve(nodes, D, n_buses, capacity):
    """Greedy nearest-feasible-stop heuristic. Not guaranteed globally optimal."""
    t0 = time.perf_counter()
    remaining = {n: v["students"] for n, v in nodes.items() if v["students"] > 0}
    routes, steps = [], []
    for b in range(n_buses):
        cur, load, stops = "COLLEGE", 0, []
        while remaining and load < capacity:
            cands = sorted((D[cur][n], n) for n in remaining)
            steps.append({"bus": b + 1, "at": cur, "candidates": [c[1] for c in cands[:3]], "chosen": cands[0][1]})
            n = cands[0][1]
            take = min(remaining[n], capacity - load)
            stops.append((n, take)); load += take; remaining[n] -= take
            if remaining[n] == 0: del remaining[n]
            cur = n
        if stops: routes.append({"bus": f"B{b + 1}", "stops": stops})
    if remaining: raise ValueError("Fleet capacity too small for all students")
    return routes, {"execution_ms": round((time.perf_counter() - t0) * 1000, 3), "steps": steps,
                    "time_complexity": "O(B * n^2) for n pickups (candidate scan per step)",
                    "space_complexity": "O(n)"}
