from goof_an_odd_husky_common.obstacles import (
    Obstacle,
    CircleObstacle,
    LineObstacle,
)
from goof_an_odd_husky_viz.visualizer import PathRenderMode, _build_obstacle_polylines, _build_arrow_segments, _compute_arc_path, _get_vehicle_poly
import http.server
import socketserver
import json
import threading
import time
import numpy as np
from typing import Callable
from numpy.typing import NDArray

class WebVisualizerHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(self.server.visualizer._get_html().encode('utf-8'))
        elif self.path == '/state':
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            self.wfile.write(self.server.visualizer._get_state_json().encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length).decode('utf-8')
        
        try:
            if self.path == '/set_goal':
                data = json.loads(body)
                self.server.visualizer._on_set_goal_clicked(data.get('text', ''))
            elif self.path == '/cancel':
                self.server.visualizer._on_cancel_clicked()
            elif self.path == '/toggle_global':
                self.server.visualizer._toggle_global()
            elif self.path == '/click':
                data = json.loads(body)
                self.server.visualizer._handle_click(data['x'], data['y'], data['action'])
        except Exception as e:
            print(f"Error handling POST {self.path}: {e}")
            
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"status":"ok"}')


class WebVisualizerServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    allow_reuse_address = True
    def __init__(self, server_address, RequestHandlerClass, visualizer):
        super().__init__(server_address, RequestHandlerClass)
        self.visualizer = visualizer


class WebTrajectoryVisualizer:
    def __init__(
        self,
        use_gps: bool,
        x_lim: tuple[float, float] = (0, 10),
        y_lim: tuple[float, float] = (0, 10),
        title: str = "TEB Optimization",
        path_render_mode: PathRenderMode | str = PathRenderMode.STRAIGHT,
        interactive_obstacles: bool = True,
        use_global: bool = False,
        on_goal_set: Callable[[float, float], None] | None = None,
        on_cancel: Callable[[], None] | None = None,
        port: int = 8080
    ) -> None:
        self._is_open = True
        self.interactive_obstacles = interactive_obstacles
        self.obstacles = []
        self._current_obstacles = []
        self.obstacle_radius = 0.5
        self.start_pos = None
        self.goal_pos = None
        self.use_global = use_global
        self.use_gps = use_gps
        self.on_goal_set = on_goal_set
        self.on_cancel = on_cancel
        self.title = title
        self.x_lim = x_lim
        self.y_lim = y_lim
        
        if isinstance(path_render_mode, str):
            path_render_mode = PathRenderMode(path_render_mode)
        self.path_render_mode = path_render_mode
        
        self._robot_pose = (0.0, 0.0, 0.0)
        self._raw_trajectory = None
        self._raw_obstacles = None
        self._raw_start_goal = None
        self._raw_global_path = None
        
        self._disp_global_path = {"x": [], "y": []}
        self._disp_traj = {"x": [], "y": []}
        self._disp_traj_straight = {"x": [], "y": []}
        self._disp_arrows = {"x": [], "y": []}
        self._disp_start = {"x": [], "y": []}
        self._disp_goal = {"x": [], "y": []}
        self._disp_obstacles = {"x": [], "y": []}
        
        self._lock = threading.RLock()
        
        self.port = port
        for _ in range(10):
            try:
                self.server = WebVisualizerServer(('0.0.0.0', self.port), WebVisualizerHandler, self)
                break
            except OSError:
                self.port += 1
                
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        print(f"[{self.title}] Web visualizer available at http://localhost:{self.port}")

    @property
    def is_open(self) -> bool:
        return self._is_open

    def update_world_state(
        self,
        robot_pose: tuple[float, float, float] | list[float],
        trajectory: NDArray[np.floating] | None,
        global_path: NDArray[np.floating] | None,
        obstacles: list[Obstacle] | None,
        start_goal: tuple[list[float], list[float]] | None,
    ) -> None:
        with self._lock:
            self._robot_pose = robot_pose
            self._raw_trajectory = trajectory
            self._raw_global_path = global_path
            self._raw_obstacles = obstacles
            self._raw_start_goal = start_goal
            self._process_and_update()

    def _process_and_update(self) -> None:
        with self._lock:
            rx, ry, rtheta = self._robot_pose
            c, s = np.cos(rtheta), np.sin(rtheta)

            if self._raw_global_path is not None and len(self._raw_global_path) > 0:
                if self.use_global:
                    self.update_global_path(self._raw_global_path)
                else:
                    dx = self._raw_global_path[:, 0] - rx
                    dy = self._raw_global_path[:, 1] - ry
                    local_path = np.zeros_like(self._raw_global_path)
                    local_path[:, 0] = dx * c + dy * s
                    local_path[:, 1] = -dx * s + dy * c
                    self.update_global_path(local_path)
            else:
                self.update_global_path(None)

            if self._raw_trajectory is not None and len(self._raw_trajectory) > 0:
                if self.use_global:
                    gtraj = self._raw_trajectory.copy()
                    gtraj[:, 0] = rx + self._raw_trajectory[:, 0] * c - self._raw_trajectory[:, 1] * s
                    gtraj[:, 1] = ry + self._raw_trajectory[:, 0] * s + self._raw_trajectory[:, 1] * c
                    gtraj[:, 2] = self._raw_trajectory[:, 2] + rtheta
                    self.update_trajectory(gtraj)
                else:
                    self.update_trajectory(self._raw_trajectory)
            else:
                self.update_trajectory(None)

            if self._raw_obstacles is not None:
                if self.use_global:
                    graphical_obs = []
                    for o in self._raw_obstacles:
                        if isinstance(o, CircleObstacle):
                            graphical_obs.append(CircleObstacle(rx + o.x * c - o.y * s, ry + o.x * s + o.y * c, o.radius))
                        elif isinstance(o, LineObstacle):
                            graphical_obs.append(LineObstacle(
                                rx + o.x1 * c - o.y1 * s, ry + o.x1 * s + o.y1 * c,
                                rx + o.x2 * c - o.y2 * s, ry + o.x2 * s + o.y2 * c,
                            ))
                    self.set_obstacles(graphical_obs)
                else:
                    self.set_obstacles(self._raw_obstacles)

            if self._raw_start_goal is not None:
                st, gl = self._raw_start_goal
                if self.use_global:
                    gsx, gsy = rx + st[0] * c - st[1] * s, ry + st[0] * s + st[1] * c
                    if len(gl) >= 3:
                        ggx, ggy = rx + gl[0] * c - gl[1] * s, ry + gl[0] * s + gl[1] * c
                        self.set_start_goal([gsx, gsy, st[2] + rtheta], [ggx, ggy, gl[2] + rtheta])
                    else:
                        self.set_start_goal([gsx, gsy, st[2] + rtheta], [])
                else:
                    self.set_start_goal(st, gl)

    def _clean_arr(self, arr):
        if arr is None or len(arr) == 0:
            return []
        return [None if (v is None or np.isnan(v)) else float(v) for v in arr]

    def set_start_goal(self, start, goal) -> None:
        with self._lock:
            sx, sy = self._to_canvas(start[0], start[1])
            if len(start) >= 3:
                canvas_angle_rad = start[2]
                if not self.use_global:
                    canvas_angle_rad += np.pi / 2
                vx, vy = _get_vehicle_poly(sx, sy, canvas_angle_rad)
                self._disp_start = {"x": self._clean_arr(vx), "y": self._clean_arr(vy)}
            else:
                self._disp_start = {"x": [float(sx)], "y": [float(sy)]}

            if len(goal) >= 2:
                gx, gy = self._to_canvas(goal[0], goal[1])
                self._disp_goal = {"x": [float(gx)], "y": [float(gy)]}
            else:
                self._disp_goal = {"x": [], "y": []}

    def set_obstacles(self, obstacles: list[Obstacle]) -> None:
        if not self._is_open: return
        with self._lock:
            self._current_obstacles = obstacles
            self._redraw_obstacles()

    def update_trajectory(self, poses: NDArray[np.floating] | None) -> None:
        with self._lock:
            if poses is None or len(poses) == 0:
                self._disp_traj = self._disp_traj_straight = self._disp_arrows = {"x": [], "y": []}
                return

            transformed = poses.copy()
            if not self.use_global:
                transformed[:, 0], transformed[:, 1] = -poses[:, 1], poses[:, 0]
                transformed[:, 2] = poses[:, 2] + np.pi / 2

            if self.path_render_mode == PathRenderMode.STRAIGHT:
                self._disp_traj = {"x": self._clean_arr(transformed[:, 0]), "y": self._clean_arr(transformed[:, 1])}
                self._disp_traj_straight = {"x": [], "y": []}
            elif self.path_render_mode == PathRenderMode.ARC:
                ax, ay = _compute_arc_path(transformed)
                self._disp_traj = {"x": self._clean_arr(ax), "y": self._clean_arr(ay)}
                self._disp_traj_straight = {"x": [], "y": []}
            else:
                self._disp_traj_straight = {"x": self._clean_arr(transformed[:, 0]), "y": self._clean_arr(transformed[:, 1])}
                ax, ay = _compute_arc_path(transformed)
                self._disp_traj = {"x": self._clean_arr(ax), "y": self._clean_arr(ay)}

            arrow_xs, arrow_ys = _build_arrow_segments(transformed)
            self._disp_arrows = {"x": self._clean_arr(arrow_xs), "y": self._clean_arr(arrow_ys)}

    def update_global_path(self, points: NDArray[np.floating] | None) -> None:
        with self._lock:
            if points is None or len(points) == 0:
                self._disp_global_path = {"x": [], "y": []}
                return
            transformed = points.copy()
            if not self.use_global:
                transformed[:, 0], transformed[:, 1] = -points[:, 1], points[:, 0]
            self._disp_global_path = {"x": self._clean_arr(transformed[:, 0]), "y": self._clean_arr(transformed[:, 1])}

    def get_obstacles(self) -> list[Obstacle]:
        return list(self.obstacles)

    def draw(self, pause_time: float = 0.01) -> None:
        if not self._is_open:
            return
        time.sleep(pause_time)

    def _redraw_obstacles(self) -> None:
        with self._lock:
            obs = getattr(self, "_current_obstacles", None)
            if not obs:
                self._disp_obstacles = {"x": [], "y": []}
                return
            canvas_obs = []
            for o in obs:
                if isinstance(o, CircleObstacle):
                    cx, cy = self._to_canvas(o.x, o.y)
                    canvas_obs.append(CircleObstacle(cx, cy, o.radius))
                elif isinstance(o, LineObstacle):
                    cx1, cy1 = self._to_canvas(o.x1, o.y1)
                    cx2, cy2 = self._to_canvas(o.x2, o.y2)
                    canvas_obs.append(LineObstacle(cx1, cy1, cx2, cy2))

            xs, ys = _build_obstacle_polylines(canvas_obs)
            self._disp_obstacles = {"x": self._clean_arr(xs), "y": self._clean_arr(ys)}

    def _to_canvas(self, x: float, y: float) -> tuple[float, float]:
        return (x, y) if self.use_global else (-y, x)

    def _from_canvas(self, cx: float, cy: float) -> tuple[float, float]:
        return (cx, cy) if self.use_global else (cy, -cx)

    def _on_set_goal_clicked(self, text: str) -> None:
        if not text: return
        parts = [p.strip() for p in text.split(",")]
        if len(parts) != 2:
            print("Invalid format. Use: x, y or lat, lon")
            return
        try:
            if self.on_goal_set:
                self.on_goal_set(float(parts[0]), float(parts[1]))
        except ValueError:
            print("Invalid coordinates. Use numeric values.")

    def _on_cancel_clicked(self) -> None:
        if self.on_cancel: self.on_cancel()

    def _toggle_global(self) -> None:
        with self._lock:
            self.use_global = not self.use_global
            print(f"Visualization mode: {'Global' if self.use_global else 'Robot'}")
            self._process_and_update()
            
    def _handle_click(self, x: float, y: float, action: str) -> None:
        if not self.interactive_obstacles: return
        with self._lock:
            if action == 'add':
                robot_x, robot_y = self._from_canvas(x, y)
                self.obstacles.append(CircleObstacle(robot_x, robot_y, self.obstacle_radius))
                self._redraw_obstacles()
            elif action == 'remove':
                self._remove_nearest_obstacle(x, y)

    def _remove_nearest_obstacle(self, x: float, y: float) -> None:
        if not self.obstacles: return
        robot_x, robot_y = self._from_canvas(x, y)
        dists = []
        for obs in self.obstacles:
            if isinstance(obs, CircleObstacle):
                dists.append(np.hypot(obs.x - robot_x, obs.y - robot_y))
            elif isinstance(obs, LineObstacle):
                px = obs.x2 - obs.x1
                py = obs.y2 - obs.y1
                norm = px * px + py * py
                if norm == 0:
                    dx, dy = obs.x1 - robot_x, obs.y1 - robot_y
                else:
                    u = max(0.0, min(1.0, ((robot_x - obs.x1) * px + (robot_y - obs.y1) * py) / float(norm)))
                    dx, dy = obs.x1 + u * px - robot_x, obs.y1 + u * py - robot_y
                dists.append(np.hypot(dx, dy))
        if dists:
            idx = int(np.argmin(dists))
            if dists[idx] < 1.0:
                self.obstacles.pop(idx)
                self._redraw_obstacles()

    def _get_state_json(self):
        with self._lock:
            data = {
                "use_global": self.use_global,
                "mode_text": "Global" if self.use_global else "Robot",
                "global_path": self._disp_global_path,
                "traj": self._disp_traj,
                "traj_straight": self._disp_traj_straight,
                "arrows": self._disp_arrows,
                "start": self._disp_start,
                "goal": self._disp_goal,
                "obstacles": self._disp_obstacles,
                "x_lim": self.x_lim,
                "y_lim": self.y_lim
            }
            return json.dumps(data)

    def _get_html(self):
        label_text = "Goal (lat, lon):" if self.use_gps else "Goal (x, y):"
        placeholder = "e.g., 37.77, -122.41" if self.use_gps else "e.g., 5.0, 0.0"
        return f"""<!DOCTYPE html>
<html>
<head>
    <title>{self.title}</title>
    <!-- Critical Meta Tag for Mobile scaling -->
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 0; padding: 0; display: flex; flex-direction: column; height: 100vh; overflow: hidden; }}
        #controls {{ padding: 12px; background: #f8f9fa; border-bottom: 1px solid #ccc; display: flex; gap: 15px; flex-wrap: wrap; overflow-y: auto; max-height: 40vh; }}
        .input-group {{ display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }}
        #plot {{ flex: 1; width: 100%; min-height: 0; }}
        
        input[type="text"], button, select {{ 
            padding: 10px 12px; 
            font-size: 14px; 
            border: 1px solid #ccc; 
            border-radius: 6px; 
            box-sizing: border-box; 
        }}
        input[type="text"] {{ width: 220px; }}
        button, select {{ background-color: #fff; cursor: pointer; }}
        button:active {{ background-color: #e9ecef; }}
        .full-width {{ font-weight: bold; }}
        
        @media (max-width: 768px) {{
            #controls {{ flex-direction: column; gap: 12px; padding: 15px; }}
            .input-group {{ width: 100%; flex-direction: row; justify-content: space-between; }}
            .full-width {{ width: 100%; flex-basis: 100%; margin-bottom: 6px; }}
            input[type="text"] {{ width: 100%; flex-basis: 100%; margin-bottom: 6px; }}
            button, select {{ flex: 1; margin: 2px; text-align: center; }}
        }}
    </style>
</head>
<body>
    <div id="controls">
        <div class="input-group">
            <label class="full-width">{label_text}</label>
            <input type="text" id="coord_input" placeholder="{placeholder}">
            <button onclick="setGoal()">Set Goal</button>
            <button onclick="cancelGoal()">Cancel</button>
        </div>
        
        <div class="input-group">
            <span class="full-width">Mode: <b id="mode_label">Global</b></span>
            <button onclick="toggleGlobal()">Toggle (G)</button>
        </div>
        
        <div class="input-group">
            <span class="full-width">Click Action:</span>
            <select id="click_action">
                <option value="add">Add Obstacle</option>
                <option value="remove">Remove Nearest</option>
            </select>
        </div>
    </div>
    <div id="plot"></div>

    <script>
        const plotDiv = document.getElementById('plot');
        const modeLabel = document.getElementById('mode_label');
        let isInitialized = false;
        let lastStateStr = "";

        function initPlot(xlim, ylim) {{
            const isMobile = window.innerWidth <= 768;
            const marginSize = isMobile ? 15 : 40; 
            
            const layout = {{
                margin: {{ l: marginSize, r: marginSize, b: marginSize, t: marginSize }},
                xaxis: {{ range: xlim, constrain: 'domain', showgrid: true, zeroline: false }},
                yaxis: {{ range: ylim, scaleanchor: 'x', showgrid: true, zeroline: false }},
                showlegend: false, hovermode: 'closest', uirevision: 'true', dragmode: 'pan'
            }};
            Plotly.newPlot(plotDiv, [], layout, {{responsive: true}});
            
            plotDiv.on('plotly_click', function(data) {{
                fetch('/click', {{
                    method: 'POST',
                    headers: {{'Content-Type': 'application/json'}},
                    body: JSON.stringify({{x: data.points[0].x, y: data.points[0].y, action: document.getElementById('click_action').value}})
                }});
            }});
            isInitialized = true;
        }}

        function updatePlot() {{
            fetch('/state').then(res => res.text()).then(text => {{
                if (text === lastStateStr && isInitialized) return;
                lastStateStr = text;
                const data = JSON.parse(text);
                
                if (!isInitialized) initPlot(data.x_lim, data.y_lim);
                modeLabel.innerText = data.mode_text;
                
                const traces = [];
                if (data.global_path.x.length > 0) traces.push({{ x: data.global_path.x, y: data.global_path.y, mode: 'lines', line: {{ color: 'green', dash: 'dash', width: 2 }}, hoverinfo: 'none' }});
                if (data.traj_straight.x.length > 0) traces.push({{ x: data.traj_straight.x, y: data.traj_straight.y, mode: 'lines+markers', line: {{ color: 'blue', dash: 'dash', width: 1 }}, marker: {{ size: 4, color: 'blue' }}, hoverinfo: 'none' }});
                if (data.traj.x.length > 0) traces.push({{ x: data.traj.x, y: data.traj.y, mode: (data.traj_straight.x.length > 0) ? 'lines' : 'lines+markers', line: {{ color: 'blue', width: 2 }}, marker: {{ size: 4, color: 'blue' }}, hoverinfo: 'none' }});
                if (data.arrows.x.length > 0) traces.push({{ x: data.arrows.x, y: data.arrows.y, mode: 'lines', line: {{ color: 'rgba(0,0,200,0.55)', width: 1 }}, hoverinfo: 'none' }});
                if (data.start.x.length > 0) traces.push({{ x: data.start.x, y: data.start.y, mode: 'lines', line: {{ color: 'red', width: 3 }}, fill: 'toself', fillcolor: 'rgba(255,0,0,0.3)', hoverinfo: 'none' }});
                if (data.goal.x.length > 0) traces.push({{ x: data.goal.x, y: data.goal.y, mode: 'markers', marker: {{ symbol: 'star', size: 18, color: 'red' }}, hoverinfo: 'none' }});
                if (data.obstacles.x.length > 0) traces.push({{ x: data.obstacles.x, y: data.obstacles.y, mode: 'lines', line: {{ color: 'black', width: 1.5 }}, hoverinfo: 'none' }});
                
                if (isInitialized) Plotly.react(plotDiv, traces, plotDiv.layout);
            }}).catch(err => console.error(err));
        }}

        setInterval(updatePlot, 100);

        function setGoal() {{
            fetch('/set_goal', {{ method: 'POST', body: JSON.stringify({{text: document.getElementById('coord_input').value}}) }});
        }}
        
        document.getElementById('coord_input').addEventListener('keypress', function(e) {{
            if (e.key === 'Enter') setGoal();
        }});

        function cancelGoal() {{ fetch('/cancel', {{method: 'POST'}}); }}
        function toggleGlobal() {{ fetch('/toggle_global', {{method: 'POST'}}); }}
        
        document.addEventListener('keydown', function(e) {{
            if ((e.key === 'g' || e.key === 'G') && document.activeElement.id !== 'coord_input') toggleGlobal();
        }});
    </script>
</body>
</html>"""
