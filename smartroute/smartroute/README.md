# SmartRoute – DAA core build
Run:
    cd backend && pip install -r requirements.txt && uvicorn main:app --reload
Open http://localhost:8000
Endpoints: GET /api/graph, POST /api/algorithms/{dijkstra,astar,greedy,vrp,compare,scale}, POST /api/routes/optimize
Body: {"pickups":15,"buses":5,"capacity":15,"start":null,"dest":"COLLEGE","blocked":[["P01","P02"]]}
All metrics are measured at run time. The graph is a deterministic synthetic road network (Hyderabad coordinates).
Not yet built: auth/JWT, PostgreSQL models, student/driver dashboards, live GPS, notifications, Leaflet map.
