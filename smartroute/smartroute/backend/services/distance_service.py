import math, random
from . import dijkstra

SPEED_KMH = 30.0
DWELL_MIN = 2.0
TOTAL_STUDENTS = 50

def haversine(a, b):
    R = 6371.0
    p1, p2 = math.radians(a["lat"]), math.radians(b["lat"])
    dp, dl = p2 - p1, math.radians(b["lon"] - a["lon"])
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(h))

def build_graph(n_pickups=15, seed=7):
    """Deterministic demo road network: college + pickups, k-nearest roads, connected."""
    rnd = random.Random(seed)
    nodes = {"COLLEGE": {"lat": 17.385, "lon": 78.486, "students": 0, "name": "Main Campus"}}
    for i in range(1, n_pickups + 1):
        ang, rad = rnd.uniform(0, 2 * math.pi), rnd.uniform(3, 14)
        nodes[f"P{i:02d}"] = {
            "lat": 17.385 + rad * math.sin(ang) / 111.0,
            "lon": 78.486 + rad * math.cos(ang) / (111.0 * math.cos(math.radians(17.385))),
            "students": TOTAL_STUDENTS // n_pickups + (1 if i <= TOTAL_STUDENTS % n_pickups else 0),
            "name": f"Pickup {i:02d}",
        }
    ids = list(nodes)
    edges = {}
    def add(a, b):
        k = tuple(sorted((a, b)))
        if k not in edges:
            edges[k] = round(haversine(nodes[a], nodes[b]) * rnd.uniform(1.2, 1.5), 3)
    for a in ids:
        for b in sorted((x for x in ids if x != a), key=lambda x: haversine(nodes[a], nodes[x]))[:3]:
            add(a, b)
    comp = {n: n for n in ids}
    def find(x):
        while comp[x] != x: x = comp[x]
        return x
    for a, b in edges: comp[find(a)] = find(b)
    while len({find(n) for n in ids}) > 1:
        a, b = min(((x, y) for x in ids for y in ids if find(x) != find(y)),
                   key=lambda p: haversine(nodes[p[0]], nodes[p[1]]))
        add(a, b); comp[find(a)] = find(b)
    return {"nodes": nodes, "edges": [[a, b, w] for (a, b), w in edges.items()]}

def adjacency(g, blocked=()):
    bl = {tuple(sorted(b)) for b in blocked}
    adj = {n: [] for n in g["nodes"]}
    for a, b, w in g["edges"]:
        if tuple(sorted((a, b))) in bl: continue
        adj[a].append((b, w)); adj[b].append((a, w))
    return adj

def all_pairs(adj):
    D, PREV = {}, {}
    for s in adj:
        dist, prev, *_ = dijkstra.run(adj, s)
        D[s], PREV[s] = dist, prev
    return D, PREV
