/**
 * SmartRoute DAA Core Engine
 * Client-side implementation of Dijkstra, A*, Greedy, and Capacitated VRP
 */

const SPEED_KMH = 30.0;
const DWELL_MIN = 2.0;
const TOTAL_STUDENTS = 50;

const COLORS = [
  "#38bdf8", // Sky Blue
  "#f43f5e", // Electric Coral
  "#10b981", // Emerald
  "#a855f7", // Purple
  "#f59e0b", // Amber
  "#ec4899", // Pink
  "#14b8a6", // Teal
  "#6366f1", // Indigo
  "#eab308", // Yellow
  "#06b6d4"  // Cyan
];

const COMPLEXITIES = {
  dijkstra: {
    time: "O((V + E) log V)",
    space: "O(V + E)",
    note: "Binary heap priority queue expands closest unsettled node; optimal for non-negative road weights."
  },
  astar: {
    time: "O(E log V) worst case",
    space: "O(V)",
    note: "Orders nodes by f = g + h with Haversine distance as admissible heuristic; prunes search space."
  },
  greedy: {
    time: "O(B · n²)",
    space: "O(n)",
    note: "Iteratively picks nearest feasible pickup stop with remaining capacity. Fast heuristic."
  },
  vrp: {
    time: "NP-hard (Clarke-Wright + 2-Opt)",
    space: "O(n²)",
    note: "Capacitated Vehicle Routing Problem solved with Clarke-Wright savings and intra-route 2-Opt local search."
  }
};

// State
let G = null;
let blocked = [];
let pos = {};
let currentAnimation = null;

const $ = id => document.getElementById(id);
const NS = "http://www.w3.org/2000/svg";

// Haversine distance
function haversine(a, b) {
  const R = 6371.0;
  const p1 = (a.lat * Math.PI) / 180, p2 = (b.lat * Math.PI) / 180;
  const dp = p2 - p1, dl = ((b.lon - a.lon) * Math.PI) / 180;
  const h = Math.sin(dp / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(h));
}

// Fallback runtime graph generator
function buildGraphFallback(nPickups = 15, seed = 7) {
  let s = seed;
  const rnd = () => {
    s = (s * 9301 + 49297) % 233280;
    return s / 233280;
  };
  const nodes = { COLLEGE: { lat: 17.385, lon: 78.486, students: 0, name: "Main Campus" } };
  for (let i = 1; i <= nPickups; i++) {
    const ang = rnd() * 2 * Math.PI;
    const rad = 3 + rnd() * 11;
    const pad = i < 10 ? "0" + i : "" + i;
    nodes["P" + pad] = {
      lat: 17.385 + (rad * Math.sin(ang)) / 111.0,
      lon: 78.486 + (rad * Math.cos(ang)) / (111.0 * Math.cos((17.385 * Math.PI) / 180)),
      students: Math.floor(TOTAL_STUDENTS / nPickups) + (i <= (TOTAL_STUDENTS % nPickups) ? 1 : 0),
      name: "Pickup " + pad
    };
  }
  const ids = Object.keys(nodes);
  const edgeMap = {};
  const add = (a, b) => {
    const k = [a, b].sort().join("-");
    if (!edgeMap[k]) edgeMap[k] = Math.round(haversine(nodes[a], nodes[b]) * (1.2 + rnd() * 0.3) * 1000) / 1000;
  };
  ids.forEach(a => {
    ids.filter(x => x !== a)
      .sort((x, y) => haversine(nodes[a], nodes[x]) - haversine(nodes[a], nodes[y]))
      .slice(0, 3)
      .forEach(b => add(a, b));
  });
  const edges = Object.entries(edgeMap).map(([k, w]) => [...k.split("-"), w]);
  return { nodes, edges };
}

function getGraph(n) {
  if (window.PRECOMPUTED_GRAPHS && window.PRECOMPUTED_GRAPHS[String(n)]) {
    return JSON.parse(JSON.stringify(window.PRECOMPUTED_GRAPHS[String(n)]));
  }
  return buildGraphFallback(Number(n));
}

function buildAdjacency(g, blockedList = []) {
  const bl = new Set(blockedList.map(b => [...b].sort().join("-")));
  const adj = {};
  for (const n of Object.keys(g.nodes)) adj[n] = [];
  for (const [u, v, w] of g.edges) {
    const key = [u, v].sort().join("-");
    if (bl.has(key)) continue;
    adj[u].push([v, w]);
    adj[v].push([u, w]);
  }
  return adj;
}

// Min-heap for Dijkstra & A*
class MinHeap {
  constructor() { this.data = []; }
  push(item) {
    this.data.push(item);
    let i = this.data.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.data[p][0] <= this.data[i][0]) break;
      [this.data[p], this.data[i]] = [this.data[i], this.data[p]];
      i = p;
    }
  }
  pop() {
    if (!this.data.length) return null;
    const top = this.data[0];
    const bottom = this.data.pop();
    if (this.data.length > 0) {
      this.data[0] = bottom;
      let i = 0;
      while (true) {
        let left = 2 * i + 1, right = 2 * i + 2, minIdx = i;
        if (left < this.data.length && this.data[left][0] < this.data[minIdx][0]) minIdx = left;
        if (right < this.data.length && this.data[right][0] < this.data[minIdx][0]) minIdx = right;
        if (minIdx === i) break;
        [this.data[i], this.data[minIdx]] = [this.data[minIdx], this.data[i]];
        i = minIdx;
      }
    }
    return top;
  }
  get size() { return this.data.length; }
}

function dijkstraSolve(adj, s, t) {
  const t0 = performance.now();
  const dist = { [s]: 0 }, prev = {}, done = new Set(), order = [], trace = [];
  let edges = 0;
  const pq = new MinHeap();
  pq.push([0, s]);

  while (pq.size > 0) {
    const [d, u] = pq.pop();
    if (done.has(u)) continue;
    done.add(u);
    order.push(u);
    trace.push(["v", u]);
    if (u === t) break;
    for (const [v, w] of adj[u] || []) {
      edges++;
      trace.push(["e", u, v]);
      if (d + w < (dist[v] ?? Infinity)) {
        dist[v] = d + w;
        prev[v] = u;
        pq.push([d + w, v]);
      }
    }
  }
  const ms = performance.now() - t0;
  if (!(t in dist)) throw new Error("No road path from " + s + " to " + t);
  const path = [t];
  while (path[path.length - 1] !== s) path.push(prev[path[path.length - 1]]);
  path.reverse();

  return {
    algorithm: "Dijkstra",
    path,
    distance_km: Math.round(dist[t] * 1000) / 1000,
    nodes_explored: order.length,
    edges_checked: edges,
    execution_ms: Math.round(ms * 1000) / 1000,
    trace,
    time_complexity: COMPLEXITIES.dijkstra.time,
    space_complexity: COMPLEXITIES.dijkstra.space
  };
}

function astarSolve(adj, nodes, s, t) {
  const t0 = performance.now();
  const g = { [s]: 0 }, prev = {}, closed = new Set(), order = [], trace = [];
  let edges = 0;
  const h = n => haversine(nodes[n], nodes[t]);
  const pq = new MinHeap();
  pq.push([h(s), s]);

  while (pq.size > 0) {
    const [_, u] = pq.pop();
    if (closed.has(u)) continue;
    closed.add(u);
    order.push(u);
    trace.push(["v", u]);
    if (u === t) break;
    for (const [v, w] of adj[u] || []) {
      edges++;
      trace.push(["e", u, v]);
      if (g[u] + w < (g[v] ?? Infinity)) {
        g[v] = g[u] + w;
        prev[v] = u;
        pq.push([g[v] + h(v), v]);
      }
    }
  }
  const ms = performance.now() - t0;
  if (!(t in g)) throw new Error("No road path from " + s + " to " + t);
  const path = [t];
  while (path[path.length - 1] !== s) path.push(prev[path[path.length - 1]]);
  path.reverse();

  return {
    algorithm: "A*",
    path,
    distance_km: Math.round(g[t] * 1000) / 1000,
    nodes_explored: order.length,
    edges_checked: edges,
    execution_ms: Math.round(ms * 1000) / 1000,
    trace,
    time_complexity: COMPLEXITIES.astar.time,
    space_complexity: COMPLEXITIES.astar.space
  };
}

function allPairs(adj) {
  const D = {}, PREV = {};
  for (const s of Object.keys(adj)) {
    const dist = { [s]: 0 }, prev = {}, done = new Set();
    const pq = new MinHeap();
    pq.push([0, s]);
    while (pq.size > 0) {
      const [d, u] = pq.pop();
      if (done.has(u)) continue;
      done.add(u);
      for (const [v, w] of adj[u] || []) {
        if (d + w < (dist[v] ?? Infinity)) {
          dist[v] = d + w;
          prev[v] = u;
          pq.push([d + w, v]);
        }
      }
    }
    D[s] = dist;
    PREV[s] = prev;
  }
  return { D, PREV };
}

function greedySolve(nodes, D, n_buses, capacity) {
  const t0 = performance.now();
  const remaining = {};
  for (const [n, v] of Object.entries(nodes)) {
    if (v.students > 0) remaining[n] = v.students;
  }
  const routes = [], steps = [];
  for (let b = 0; b < n_buses; b++) {
    let cur = "COLLEGE", load = 0, stops = [];
    while (Object.keys(remaining).length > 0 && load < capacity) {
      const cands = Object.keys(remaining).map(n => [D[cur][n], n]).sort((a, b) => a[0] - b[0]);
      const chosen = cands[0][1];
      steps.push({ bus: b + 1, at: cur, candidates: cands.slice(0, 3).map(c => c[1]), chosen });
      const take = Math.min(remaining[chosen], capacity - load);
      stops.push([chosen, take]);
      load += take;
      remaining[chosen] -= take;
      if (remaining[chosen] === 0) delete remaining[chosen];
      cur = chosen;
    }
    if (stops.length > 0) routes.push({ bus: "B" + (b + 1), stops });
  }
  if (Object.keys(remaining).length > 0) throw new Error("Fleet capacity too small for all students");
  const ms = performance.now() - t0;
  return {
    routes,
    meta: {
      execution_ms: Math.round(ms * 1000) / 1000,
      steps,
      time_complexity: COMPLEXITIES.greedy.time,
      space_complexity: COMPLEXITIES.greedy.space
    }
  };
}

function vrpSolve(nodes, D, n_buses, capacity) {
  const t0 = performance.now();
  const customerIds = Object.keys(nodes).filter(n => n !== "COLLEGE");
  let routes = customerIds.map(n => ({
    load: nodes[n].students,
    stops: [n]
  }));

  const savings = [];
  for (let i = 0; i < customerIds.length; i++) {
    for (let j = i + 1; j < customerIds.length; j++) {
      const u = customerIds[i], v = customerIds[j];
      const s = D["COLLEGE"][u] + D["COLLEGE"][v] - D[u][v];
      savings.push({ u, v, s });
    }
  }
  savings.sort((a, b) => b.s - a.s);

  for (const { u, v } of savings) {
    const rU = routes.find(r => r.stops[0] === u || r.stops[r.stops.length - 1] === u);
    const rV = routes.find(r => r.stops[0] === v || r.stops[r.stops.length - 1] === v);
    if (!rU || !rV || rU === rV) continue;
    if (rU.load + rV.load > capacity) continue;

    if (rU.stops[rU.stops.length - 1] === u && rV.stops[0] === v) {
      rU.stops = rU.stops.concat(rV.stops);
    } else if (rU.stops[0] === u && rV.stops[rV.stops.length - 1] === v) {
      rU.stops = rV.stops.concat(rU.stops);
    } else if (rU.stops[rU.stops.length - 1] === u && rV.stops[rV.stops.length - 1] === v) {
      rU.stops = rU.stops.concat(rV.stops.slice().reverse());
    } else if (rU.stops[0] === u && rV.stops[0] === v) {
      rU.stops = rU.stops.slice().reverse().concat(rV.stops);
    } else {
      continue;
    }
    rU.load += rV.load;
    routes = routes.filter(r => r !== rV);
  }

  // 2-Opt local search
  const routeDist = stops => {
    let d = D["COLLEGE"][stops[0]];
    for (let i = 0; i < stops.length - 1; i++) d += D[stops[i]][stops[i + 1]];
    d += D[stops[stops.length - 1]]["COLLEGE"];
    return d;
  };

  routes.forEach(r => {
    let improved = true;
    while (improved) {
      improved = false;
      const bestDist = routeDist(r.stops);
      for (let i = 0; i < r.stops.length - 1; i++) {
        for (let j = i + 1; j < r.stops.length; j++) {
          const newStops = r.stops.slice(0, i).concat(r.stops.slice(i, j + 1).reverse()).concat(r.stops.slice(j + 1));
          if (routeDist(newStops) < bestDist - 1e-6) {
            r.stops = newStops;
            improved = true;
            break;
          }
        }
        if (improved) break;
      }
    }
  });

  if (routes.length > n_buses) {
    throw new Error("VRP infeasible: increase buses or capacity");
  }

  const outRoutes = routes.map((r, i) => ({
    bus: "B" + (i + 1),
    stops: r.stops.map(n => [n, nodes[n].students])
  }));

  const ms = performance.now() - t0;
  return {
    routes: outRoutes,
    meta: {
      execution_ms: Math.round(ms * 1000) / 1000,
      time_complexity: COMPLEXITIES.vrp.time,
      space_complexity: COMPLEXITIES.vrp.space
    }
  };
}

function evaluateRoutes(D, rawRoutes) {
  let tot_d = 0.0, wait = 0.0, studs = 0, tot_t = 0.0;
  const routes = [];
  for (const r of rawRoutes) {
    let cur = "COLLEGE", d = 0.0, stops = [];
    for (let k = 0; k < r.stops.length; k++) {
      const [n, c] = r.stops[k];
      d += D[cur][n];
      const eta = (d / SPEED_KMH) * 60 + DWELL_MIN * (k + 1);
      stops.push({ node: n, students: c, eta_min: Math.round(eta * 10) / 10 });
      wait += eta * c;
      studs += c;
      cur = n;
    }
    d += D[cur]["COLLEGE"];
    const t = (d / SPEED_KMH) * 60 + DWELL_MIN * stops.length;
    tot_d += d;
    tot_t += t;
    routes.push({
      bus: r.bus,
      stops,
      distance_km: Math.round(d * 100) / 100,
      students: stops.reduce((acc, s) => acc + s.students, 0),
      travel_min: Math.round(t * 10) / 10
    });
  }
  return {
    routes,
    total_distance_km: Math.round(tot_d * 100) / 100,
    average_waiting_minutes: studs ? Math.round((wait / studs) * 100) / 100 : 0,
    buses_used: routes.length,
    total_travel_time: Math.round(tot_t * 10) / 10,
    students_served: studs
  };
}

function baselinePlan(nodes, n_buses, capacity) {
  const routes = [];
  let b = 1, load = 0, stops = [];
  for (const [n, v] of Object.entries(nodes)) {
    if (n === "COLLEGE") continue;
    let left = v.students;
    while (left > 0) {
      if (load === capacity) {
        routes.push({ bus: "B" + b, stops });
        b++;
        load = 0;
        stops = [];
      }
      const take = Math.min(left, capacity - load);
      stops.push([n, take]);
      load += take;
      left -= take;
    }
  }
  if (stops.length > 0) routes.push({ bus: "B" + b, stops });
  return routes;
}

function expandStops(PREV, stopNames) {
  const full = [];
  const seq = ["COLLEGE", ...stopNames, "COLLEGE"];
  for (let i = 0; i < seq.length - 1; i++) {
    const a = seq[i], b = seq[i + 1];
    const p = [b];
    while (p[p.length - 1] !== a) {
      p.push(PREV[a][p[p.length - 1]]);
    }
    const leg = p.reverse();
    if (full.length === 0) full.push(...leg);
    else full.push(...leg.slice(1));
  }
  return full;
}

// UI & Drawing
function el(tag, attrs, parent) {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  parent.appendChild(e);
  return e;
}

function drawNetwork(routes) {
  const svg = $("map");
  svg.innerHTML = "";
  const N = G.nodes;
  const v = Object.values(N);
  const lats = v.map(x => x.lat), lons = v.map(x => x.lon);
  const minLat = Math.min(...lats), maxLat = Math.max(...lats);
  const minLon = Math.min(...lons), maxLon = Math.max(...lons);

  for (const k in N) {
    pos[k] = [
      60 + ((N[k].lon - minLon) / (maxLon - minLon || 1)) * 880,
      640 - ((N[k].lat - minLat) / (maxLat - minLat || 1)) * 580
    ];
  }

  // Draw Edges
  G.edges.forEach(([x, y]) => {
    const isBlk = blocked.some(p => p.includes(x) && p.includes(y));
    const line = el("line", {
      x1: pos[x][0], y1: pos[x][1],
      x2: pos[y][0], y2: pos[y][1],
      class: "road" + (isBlk ? " blk" : ""),
      id: `e-${x}-${y}`,
      style: "cursor:pointer"
    }, svg);

    line.onclick = () => {
      if (isBlk) {
        blocked = blocked.filter(p => !(p.includes(x) && p.includes(y)));
      } else {
        blocked.push([x, y]);
      }
      renderBlockedPills();
      drawNetwork(routes);
    };
  });

  // Draw Active Routes
  if (routes && routes.length) {
    routes.forEach((r, i) => {
      if (!r.path) return;
      el("polyline", {
        class: "route-poly",
        stroke: COLORS[i % COLORS.length],
        "stroke-width": 5,
        points: r.path.map(n => pos[n].join(",")).join(" ")
      }, svg);
    });
  }

  // Draw Nodes
  for (const k in N) {
    const [x, y] = pos[k];
    const isCollege = k === "COLLEGE";
    const g = el("g", { class: "nd-node", id: "node-group-" + k }, svg);

    if (isCollege) {
      el("rect", {
        x: x - 13, y: y - 13, width: 26, height: 26, rx: 5,
        class: "nd-college", id: "n-" + k
      }, g);
    } else {
      el("circle", {
        cx: x, cy: y, r: 10,
        class: "nd-circle", id: "n-" + k
      }, g);
    }

    const txt = el("text", {
      x: x + 14, y: y + 4, class: "lbl"
    }, g);
    txt.textContent = isCollege ? "COLLEGE" : `${k} (${N[k].students})`;
  }
}

function renderBlockedPills() {
  const container = $("blocked-tags");
  if (!blocked.length) {
    container.innerHTML = `<span style="font-size:11px;color:var(--text-dim)">No blocked roads. Click any road on the map to block.</span>`;
    return;
  }
  container.innerHTML = blocked.map(([u, v]) => `
    <span class="blocked-pill" onclick="unblockRoad('${u}', '${v}')">
      ${u} ✕ ${v}
    </span>
  `).join("");
}

window.unblockRoad = (u, v) => {
  blocked = blocked.filter(p => !(p.includes(u) && p.includes(v)));
  renderBlockedPills();
  drawNetwork();
};

$("clear-blk").onclick = () => {
  blocked = [];
  renderBlockedPills();
  drawNetwork();
};

function animatePath(trace, finalPath) {
  if (currentAnimation) clearInterval(currentAnimation);
  return new Promise(resolve => {
    let i = 0, prev = null;
    currentAnimation = setInterval(() => {
      if (i >= trace.length) {
        clearInterval(currentAnimation);
        currentAnimation = null;
        finalPath.forEach((n, idx) => {
          const elNode = $("n-" + n);
          if (elNode) elNode.setAttribute("class", "nd-circle nd-fin");
          if (idx > 0) highlightLine(finalPath[idx - 1], n);
        });
        return resolve();
      }
      const [type, a, b] = trace[i++];
      if (type === "v") {
        if (prev) {
          const pEl = $("n-" + prev);
          if (pEl && !pEl.classList.contains("nd-college")) pEl.setAttribute("class", "nd-circle nd-vis");
        }
        const curEl = $("n-" + a);
        if (curEl && !curEl.classList.contains("nd-college")) curEl.setAttribute("class", "nd-circle nd-cur");
        prev = a;
      } else {
        const edge = $(`e-${a}-${b}`) || $(`e-${b}-${a}`);
        if (edge) edge.classList.add("chk");
      }
    }, 45);
  });
}

function highlightLine(a, b) {
  const edge = $(`e-${a}-${b}`) || $(`e-${b}-${a}`);
  if (edge) {
    edge.setAttribute("stroke", "#06b6d4");
    edge.setAttribute("stroke-width", "6");
  }
}

function updateKPI(dist, time, wait, exec) {
  $("kpi-dist").innerHTML = `${dist} <span class="kpi-unit">km</span>`;
  $("kpi-time").innerHTML = `${time} <span class="kpi-unit">min</span>`;
  $("kpi-wait").innerHTML = `${wait} <span class="kpi-unit">min</span>`;
  $("kpi-exec").innerHTML = `${exec} <span class="kpi-unit">ms</span>`;
}

function updateComplexityUI() {
  const alg = $("alg").value;
  const c = COMPLEXITIES[alg];
  $("cx-time").textContent = c.time;
  $("cx-space").textContent = c.space;
  $("cx-note").textContent = c.note;
}

$("alg").onchange = updateComplexityUI;

function loadNetwork() {
  const n = $("n").value;
  $("n-val").textContent = n;
  G = getGraph(n);

  const ids = Object.keys(G.nodes);
  $("s").innerHTML = ids.map(i => `<option value="${i}">${i}</option>`).join("");
  $("d").innerHTML = ids.map(i => `<option value="${i}">${i}</option>`).join("");

  $("s").value = ids[ids.length - 1];
  $("d").value = "COLLEGE";
  blocked = [];
  renderBlockedPills();
  drawNetwork();
  updateComplexityUI();
}

$("n").onchange = loadNetwork;

// Run Algorithm
$("run").onclick = async () => {
  const alg = $("alg").value;
  const s = $("s").value;
  const d = $("d").value;
  const buses = +$("nb").value;
  const cap = +$("cap").value;

  drawNetwork();
  const adj = buildAdjacency(G, blocked);
  const { D, PREV } = allPairs(adj);

  if (Object.values(adj).some(edges => edges.length === 0)) {
    alert("Blocked roads disconnect nodes in the network!");
    return;
  }

  if (alg === "dijkstra") {
    try {
      const res = dijkstraSolve(adj, s, d);
      updateKPI(res.distance_km, Math.round((res.distance_km / SPEED_KMH) * 60 * 10) / 10, "—", res.execution_ms);
      $("routes-list").innerHTML = `
        <div class="bus-card" style="border-left-color: #06b6d4;">
          <div class="bus-header">
            <span>Dijkstra Shortest Path</span>
            <span class="bus-load">${res.nodes_explored} nodes</span>
          </div>
          <div class="bus-path">${res.path.join(" → ")}</div>
        </div>
      `;
      await animatePath(res.trace, res.path);
    } catch (e) {
      alert(e.message);
    }
  } else if (alg === "astar") {
    try {
      const res = astarSolve(adj, G.nodes, s, d);
      updateKPI(res.distance_km, Math.round((res.distance_km / SPEED_KMH) * 60 * 10) / 10, "—", res.execution_ms);
      $("routes-list").innerHTML = `
        <div class="bus-card" style="border-left-color: #06b6d4;">
          <div class="bus-header">
            <span>A* Shortest Path</span>
            <span class="bus-load">${res.nodes_explored} nodes</span>
          </div>
          <div class="bus-path">${res.path.join(" → ")}</div>
        </div>
      `;
      await animatePath(res.trace, res.path);
    } catch (e) {
      alert(e.message);
    }
  } else if (alg === "greedy") {
    try {
      const { routes, meta } = greedySolve(G.nodes, D, buses, cap);
      const evalRes = evaluateRoutes(D, routes);
      evalRes.routes.forEach((r, idx) => {
        r.path = expandStops(PREV, routes[idx].stops.map(x => x[0]));
      });
      drawNetwork(evalRes.routes);
      updateKPI(evalRes.total_distance_km, evalRes.total_travel_time, evalRes.average_waiting_minutes, meta.execution_ms);
      $("routes-list").innerHTML = evalRes.routes.map((r, i) => `
        <div class="bus-card" style="border-left-color: ${COLORS[i % COLORS.length]};">
          <div class="bus-header">
            <span>${r.bus} (${r.students}/${cap} students)</span>
            <span class="bus-load">${r.distance_km} km</span>
          </div>
          <div class="bus-path">COLLEGE → ${r.stops.map(s => s.node + " (" + s.students + ")").join(" → ")} → COLLEGE</div>
        </div>
      `).join("");
    } catch (e) {
      alert(e.message);
    }
  } else if (alg === "vrp") {
    try {
      const { routes, meta } = vrpSolve(G.nodes, D, buses, cap);
      const evalRes = evaluateRoutes(D, routes);
      evalRes.routes.forEach((r, idx) => {
        r.path = expandStops(PREV, routes[idx].stops.map(x => x[0]));
      });
      drawNetwork(evalRes.routes);
      updateKPI(evalRes.total_distance_km, evalRes.total_travel_time, evalRes.average_waiting_minutes, meta.execution_ms);
      $("routes-list").innerHTML = evalRes.routes.map((r, i) => `
        <div class="bus-card" style="border-left-color: ${COLORS[i % COLORS.length]};">
          <div class="bus-header">
            <span>${r.bus} (${r.students}/${cap} students)</span>
            <span class="bus-load">${r.distance_km} km</span>
          </div>
          <div class="bus-path">COLLEGE → ${r.stops.map(s => s.node + " (" + s.students + ")").join(" → ")} → COLLEGE</div>
        </div>
      `).join("");
    } catch (e) {
      alert(e.message);
    }
  }
};

// Compare All
$("cmp").onclick = () => {
  const s = $("s").value, d = $("d").value;
  const buses = +$("nb").value, cap = +$("cap").value;
  const adj = buildAdjacency(G, blocked);
  const { D } = allPairs(adj);

  const dijk = dijkstraSolve(adj, s, d);
  const ast = astarSolve(adj, G.nodes, s, d);
  const gr = greedySolve(G.nodes, D, buses, cap);
  const grEval = evaluateRoutes(D, gr.routes);
  const vrp = vrpSolve(G.nodes, D, buses, cap);
  const vrpEval = evaluateRoutes(D, vrp.routes);

  $("out-panel").style.display = "block";
  $("out-title").textContent = "📊 DAA Comprehensive Algorithm Benchmark";
  $("out-content").innerHTML = `
    <table>
      <thead>
        <tr>
          <th>Algorithm</th>
          <th>Type</th>
          <th>Distance (km)</th>
          <th>Travel Time (min)</th>
          <th>Nodes Explored</th>
          <th>Buses</th>
          <th>Avg Wait (min)</th>
          <th>Runtime (ms)</th>
        </tr>
      </thead>
      <tbody>
        <tr>
          <td><b>Dijkstra</b></td>
          <td>Exact Shortest Path</td>
          <td>${dijk.distance_km}</td>
          <td>${Math.round((dijk.distance_km / SPEED_KMH) * 60 * 10) / 10}</td>
          <td>${dijk.nodes_explored}</td>
          <td>1</td>
          <td>—</td>
          <td>${dijk.execution_ms}</td>
        </tr>
        <tr>
          <td><b>A* Search</b></td>
          <td>Heuristic Shortest Path</td>
          <td>${ast.distance_km}</td>
          <td>${Math.round((ast.distance_km / SPEED_KMH) * 60 * 10) / 10}</td>
          <td>${ast.nodes_explored}</td>
          <td>1</td>
          <td>—</td>
          <td>${ast.execution_ms}</td>
        </tr>
        <tr>
          <td><b>Greedy</b></td>
          <td>Route Construction</td>
          <td>${grEval.total_distance_km}</td>
          <td>${grEval.total_travel_time}</td>
          <td>—</td>
          <td>${grEval.buses_used}</td>
          <td>${grEval.average_waiting_minutes}</td>
          <td>${gr.meta.execution_ms}</td>
        </tr>
        <tr>
          <td><b>Capacitated VRP</b></td>
          <td>Multi-Bus Optimization</td>
          <td>${vrpEval.total_distance_km}</td>
          <td>${vrpEval.total_travel_time}</td>
          <td>—</td>
          <td>${vrpEval.buses_used}</td>
          <td>${vrpEval.average_waiting_minutes}</td>
          <td>${vrp.meta.execution_ms}</td>
        </tr>
      </tbody>
    </table>
    <p style="font-size:12px;color:var(--text-dim);margin-top:8px;">
      * Path algorithms evaluated from <b>${s}</b> to <b>${d}</b>. Fleet algorithms cover all ${$("n").value} pickup stops under capacity ${cap}.
    </p>
  `;
  $("out-panel").scrollIntoView({ behavior: "smooth" });
};

// Before vs After Optimization
$("opt").onclick = () => {
  const buses = +$("nb").value, cap = +$("cap").value;
  const adj = buildAdjacency(G, blocked);
  const { D, PREV } = allPairs(adj);

  const baseRaw = baselinePlan(G.nodes, buses, cap);
  const baseEval = evaluateRoutes(D, baseRaw);

  const vrpRaw = vrpSolve(G.nodes, D, buses, cap);
  const vrpEval = evaluateRoutes(D, vrpRaw.routes);

  vrpEval.routes.forEach((r, idx) => {
    r.path = expandStops(PREV, vrpRaw.routes[idx].stops.map(x => x[0]));
  });
  drawNetwork(vrpEval.routes);

  const distSaved = Math.round((baseEval.total_distance_km - vrpEval.total_distance_km) * 100) / 100;
  const timeSaved = Math.round((baseEval.total_travel_time - vrpEval.total_travel_time) * 10) / 10;
  const waitSaved = Math.round((baseEval.average_waiting_minutes - vrpEval.average_waiting_minutes) * 100) / 100;

  $("out-panel").style.display = "block";
  $("out-title").textContent = "✨ Before vs After Optimization (VRP vs Baseline)";
  $("out-content").innerHTML = `
    <table>
      <thead>
        <tr>
          <th>Metric</th>
          <th>Before (ID-Order Plan)</th>
          <th>After (VRP Optimized)</th>
          <th>Improvement</th>
        </tr>
      </thead>
      <tbody>
        <tr>
          <td>Total Distance</td>
          <td>${baseEval.total_distance_km} km</td>
          <td>${vrpEval.total_distance_km} km</td>
          <td class="diff-positive">-${distSaved} km (${Math.round((distSaved / baseEval.total_distance_km) * 100)}%)</td>
        </tr>
        <tr>
          <td>Total Travel Time</td>
          <td>${baseEval.total_travel_time} min</td>
          <td>${vrpEval.total_travel_time} min</td>
          <td class="diff-positive">-${timeSaved} min</td>
        </tr>
        <tr>
          <td>Average Waiting Time</td>
          <td>${baseEval.average_waiting_minutes} min</td>
          <td>${vrpEval.average_waiting_minutes} min</td>
          <td class="diff-positive">-${waitSaved} min</td>
        </tr>
        <tr>
          <td>Buses Utilized</td>
          <td>${baseEval.buses_used}</td>
          <td>${vrpEval.buses_used}</td>
          <td class="diff-positive">${baseEval.buses_used - vrpEval.buses_used <= 0 ? "Optimal allocation" : `-${baseEval.buses_used - vrpEval.buses_used} buses`}</td>
        </tr>
      </tbody>
    </table>
  `;
  $("out-panel").scrollIntoView({ behavior: "smooth" });
};

// Scale Benchmark
$("scl").onclick = () => {
  const buses = +$("nb").value, cap = +$("cap").value;
  const sizes = [5, 10, 15, 20];
  const rows = [];

  for (const n of sizes) {
    const net = getGraph(n);
    const adj = buildAdjacency(net, []);
    const { D } = allPairs(adj);
    const ids = Object.keys(net.nodes).filter(x => x !== "COLLEGE");
    const startNode = ids[ids.length - 1];

    const dijk = dijkstraSolve(adj, startNode, "COLLEGE");
    const ast = astarSolve(adj, net.nodes, startNode, "COLLEGE");
    const gr = greedySolve(net.nodes, D, buses, cap);
    const grEval = evaluateRoutes(D, gr.routes);
    const vrp = vrpSolve(net.nodes, D, buses, cap);
    const vrpEval = evaluateRoutes(D, vrp.routes);

    rows.push({
      n,
      dijk_ms: dijk.execution_ms,
      dijk_nodes: dijk.nodes_explored,
      ast_ms: ast.execution_ms,
      ast_nodes: ast.nodes_explored,
      gr_ms: gr.meta.execution_ms,
      gr_km: grEval.total_distance_km,
      gr_buses: grEval.buses_used,
      vrp_ms: vrp.meta.execution_ms,
      vrp_km: vrpEval.total_distance_km,
      vrp_buses: vrpEval.buses_used
    });
  }

  const maxVrpKm = Math.max(...rows.map(r => r.vrp_km));
  const maxGrKm = Math.max(...rows.map(r => r.gr_km));

  $("out-panel").style.display = "block";
  $("out-title").textContent = "📈 Measured Scaling Across Pickup Sizes (5–20)";
  $("out-content").innerHTML = `
    <table>
      <thead>
        <tr>
          <th>Stops</th>
          <th>Dijkstra (ms / nodes)</th>
          <th>A* (ms / nodes)</th>
          <th>Greedy (ms / km / buses)</th>
          <th>VRP (ms / km / buses)</th>
        </tr>
      </thead>
      <tbody>
        ${rows.map(r => `
          <tr>
            <td><b>${r.n} stops</b></td>
            <td>${r.dijk_ms} ms / ${r.dijk_nodes} nd</td>
            <td>${r.ast_ms} ms / ${r.ast_nodes} nd</td>
            <td>${r.gr_ms} ms / ${r.gr_km} km / ${r.gr_buses} bus</td>
            <td>${r.vrp_ms} ms / ${r.vrp_km} km / ${r.vrp_buses} bus</td>
          </tr>
        `).join("")}
      </tbody>
    </table>

    <div style="margin-top:20px;">
      <h4 style="font-size:13px;color:var(--text-muted);margin-bottom:8px;">VRP Distance Comparison (km)</h4>
      ${rows.map(r => `
        <div class="bar-wrapper">
          <div class="bar-label"><span>${r.n} stops</span> <b>${r.vrp_km} km</b></div>
          <div class="bar-track"><div class="bar-fill" style="width:${(r.vrp_km / maxVrpKm) * 100}%"></div></div>
        </div>
      `).join("")}

      <h4 style="font-size:13px;color:var(--text-muted);margin-top:16px;margin-bottom:8px;">Greedy Route Distance (km)</h4>
      ${rows.map(r => `
        <div class="bar-wrapper">
          <div class="bar-label"><span>${r.n} stops</span> <b>${r.gr_km} km</b></div>
          <div class="bar-track"><div class="bar-fill" style="width:${(r.gr_km / maxGrKm) * 100}%;background:linear-gradient(90deg, #f59e0b, #f43f5e);"></div></div>
        </div>
      `).join("")}
    </div>
  `;
  $("out-panel").scrollIntoView({ behavior: "smooth" });
};

// Initialize
loadNetwork();
