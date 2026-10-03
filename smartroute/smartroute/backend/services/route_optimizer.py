import time
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from .distance_service import SPEED_KMH, DWELL_MIN

def vrp(nodes, D, n_buses, capacity, time_limit=1):
    """Capacitated VRP with OR-Tools (distance objective + fixed cost per bus used)."""
    t0 = time.perf_counter()
    ids = list(nodes)
    idx = {n: i for i, n in enumerate(ids)}
    m = pywrapcp.RoutingIndexManager(len(ids), n_buses, 0)
    r = pywrapcp.RoutingModel(m)
    cb = r.RegisterTransitCallback(lambda a, b: int(D[ids[m.IndexToNode(a)]][ids[m.IndexToNode(b)]] * 1000))
    r.SetArcCostEvaluatorOfAllVehicles(cb)
    r.SetFixedCostOfAllVehicles(5000)
    dm = r.RegisterUnaryTransitCallback(lambda i: nodes[ids[m.IndexToNode(i)]]["students"])
    r.AddDimensionWithVehicleCapacity(dm, 0, [capacity] * n_buses, True, "Cap")
    p = pywrapcp.DefaultRoutingSearchParameters()
    p.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    p.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    p.time_limit.seconds = time_limit
    sol = r.SolveWithParameters(p)
    if not sol: raise ValueError("VRP infeasible: increase buses or capacity")
    routes = []
    for v in range(n_buses):
        i, stops = r.Start(v), []
        while not r.IsEnd(i):
            n = ids[m.IndexToNode(i)]
            if n != "COLLEGE": stops.append((n, nodes[n]["students"]))
            i = sol.Value(r.NextVar(i))
        if stops: routes.append({"bus": f"B{v + 1}", "stops": stops})
    return routes, {"execution_ms": round((time.perf_counter() - t0) * 1000, 3),
                    "time_complexity": "NP-hard in general; OR-Tools uses heuristic construction + local search",
                    "space_complexity": "O(n^2) distance matrix"}

def evaluate(D, routes):
    out, tot_d, wait, studs, tot_t = [], 0.0, 0.0, 0, 0.0
    for r in routes:
        cur, d, stops = "COLLEGE", 0.0, []
        for k, (n, c) in enumerate(r["stops"], 1):
            d += D[cur][n]
            eta = d / SPEED_KMH * 60 + DWELL_MIN * k
            stops.append({"node": n, "students": c, "eta_min": round(eta, 1)})
            wait += eta * c; studs += c; cur = n
        d += D[cur]["COLLEGE"]
        t = d / SPEED_KMH * 60 + DWELL_MIN * len(stops)
        tot_d += d; tot_t += t
        out.append({"bus": r["bus"], "stops": stops, "distance_km": round(d, 2),
                    "students": sum(s["students"] for s in stops), "travel_min": round(t, 1)})
    return {"routes": out, "total_distance_km": round(tot_d, 2),
            "average_waiting_minutes": round(wait / studs, 2) if studs else 0,
            "buses_used": len(out), "total_travel_time": round(tot_t, 1), "students_served": studs}

def baseline(nodes, n_buses, capacity):
    """Un-optimised plan: pickups in ID order, buses filled sequentially."""
    routes, b, load, stops = [], 1, 0, []
    for n, v in nodes.items():
        left = v["students"]
        while left:
            if load == capacity:
                routes.append({"bus": f"B{b}", "stops": stops}); b += 1; load, stops = 0, []
            take = min(left, capacity - load); stops.append((n, take)); load += take; left -= take
    if stops: routes.append({"bus": f"B{b}", "stops": stops})
    return routes

def expand(PREV, seq):
    """Turn a stop sequence into the full road-node path (for map drawing)."""
    full, seq = [], ["COLLEGE"] + seq + ["COLLEGE"]
    for a, b in zip(seq, seq[1:]):
        p = [b]
        while p[-1] != a: p.append(PREV[a][p[-1]])
        full += p[::-1][1:] if full else p[::-1]
    return full
