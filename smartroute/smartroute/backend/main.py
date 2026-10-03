import os
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from services import distance_service as ds, dijkstra, astar, greedy, route_optimizer as ro

app = FastAPI(title="SmartRoute")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

class Req(BaseModel):
    pickups: int = Field(15, ge=3, le=30)
    buses: int = Field(5, ge=1, le=10)
    capacity: int = Field(15, ge=5, le=60)
    start: str | None = None
    dest: str = "COLLEGE"
    blocked: list[list[str]] = []

def ctx(q: Req):
    if q.buses * q.capacity < ds.TOTAL_STUDENTS:
        raise HTTPException(400, "Fleet capacity is below 50 students")
    g = ds.build_graph(q.pickups)
    adj = ds.adjacency(g, q.blocked)
    D, PREV = ds.all_pairs(adj)
    if any(len(D[n]) < len(adj) for n in adj):
        raise HTTPException(400, "Blocked roads disconnect the network")
    start = q.start or max((n for n in g["nodes"] if n != "COLLEGE"), key=lambda n: D["COLLEGE"][n])
    return g, adj, D, PREV, start

def guard(fn):
    try: return fn()
    except ValueError as e: raise HTTPException(400, str(e))

def fleet(name, routes, meta, D, PREV):
    res = ro.evaluate(D, routes)
    for r, raw in zip(res["routes"], routes):
        r["path"] = ro.expand(PREV, [s[0] for s in raw["stops"]])
    return {"algorithm": name, **res, **meta}

@app.get("/api/graph")
def graph(pickups: int = 15):
    return ds.build_graph(max(3, min(pickups, 30)))

@app.post("/api/algorithms/dijkstra")
def r_dijkstra(q: Req):
    g, adj, *_, start = ctx(q); return guard(lambda: dijkstra.solve(adj, start, q.dest))

@app.post("/api/algorithms/astar")
def r_astar(q: Req):
    g, adj, *_, start = ctx(q); return guard(lambda: astar.solve(adj, g["nodes"], start, q.dest))

@app.post("/api/algorithms/greedy")
def r_greedy(q: Req):
    g, adj, D, PREV, _ = ctx(q)
    routes, meta = guard(lambda: greedy.solve(g["nodes"], D, q.buses, q.capacity))
    return fleet("Greedy", routes, meta, D, PREV)

@app.post("/api/algorithms/vrp")
def r_vrp(q: Req):
    g, adj, D, PREV, _ = ctx(q)
    routes, meta = guard(lambda: ro.vrp(g["nodes"], D, q.buses, q.capacity))
    return fleet("VRP (OR-Tools)", routes, meta, D, PREV)

@app.post("/api/routes/optimize")
def optimize(q: Req):
    g, adj, D, PREV, _ = ctx(q)
    before = ro.evaluate(D, ro.baseline(g["nodes"], q.buses, q.capacity))
    routes, meta = guard(lambda: ro.vrp(g["nodes"], D, q.buses, q.capacity))
    after = fleet("VRP (OR-Tools)", routes, meta, D, PREV)
    keys = ["total_distance_km", "average_waiting_minutes", "buses_used", "total_travel_time"]
    return {"before": before, "after": after, "difference": {k: round(before[k] - after[k], 2) for k in keys}}

@app.post("/api/algorithms/compare")
def compare(q: Req):
    g, adj, D, PREV, start = ctx(q)
    d, a = dijkstra.solve(adj, start, q.dest), astar.solve(adj, g["nodes"], start, q.dest)
    gr = fleet("Greedy", *guard(lambda: greedy.solve(g["nodes"], D, q.buses, q.capacity)), D, PREV)
    vr = fleet("VRP (OR-Tools)", *guard(lambda: ro.vrp(g["nodes"], D, q.buses, q.capacity)), D, PREV)
    for x in (d, a): x.pop("trace")
    return {"start": start, "dest": q.dest, "dijkstra": d, "astar": a, "greedy": gr, "vrp": vr}

@app.post("/api/algorithms/scale")
def scale(q: Req):
    rows = []
    for n in (5, 10, 15, 20):
        r = compare(q.model_copy(update={"pickups": n, "start": None, "blocked": []}))
        rows.append({"pickups": n, "dijkstra_ms": r["dijkstra"]["execution_ms"], "astar_ms": r["astar"]["execution_ms"],
                     "dijkstra_nodes": r["dijkstra"]["nodes_explored"], "astar_nodes": r["astar"]["nodes_explored"],
                     "greedy_ms": r["greedy"]["execution_ms"], "greedy_km": r["greedy"]["total_distance_km"],
                     "vrp_ms": r["vrp"]["execution_ms"], "vrp_km": r["vrp"]["total_distance_km"],
                     "greedy_buses": r["greedy"]["buses_used"], "vrp_buses": r["vrp"]["buses_used"]})
    return rows

app.mount("/", StaticFiles(directory=os.path.join(os.path.dirname(__file__), "..", "frontend"), html=True))
